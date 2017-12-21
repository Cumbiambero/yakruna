
/**
 * @file        yakruna.cpp
 * @author      Eduardo Hahn Paredes <cumbiamberos@gmail.com>
 * @copyright   © 2017, Eduardo Hahn Paredes, Ecuador
 * @version     1.0.2
 * @repository  https://github.com/Cumbiambero/yakruna
 *
 * @section     LICENCE
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version. This program is distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details at https://www.gnu.org/copyleft/gpl.html
 *
 * @section     DESCRIPTION
 * Contains the setup and entry point of the program.
 */

#include "yakruna.h"
#include "waveforms.h"

/** LCD does only uses the first 8 of the 16 available columns but both rows */
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

/** Mode of the sequencer */
Direction direction{Direction::FORWARDS};

DAC<TimerOne> pwmCV;            /**> Fast PWM used for CV */
DAC<TimerFour> pwmOsc;          /**> Fast PWM used for oscillator */
NoteList notes;                 /**> Used for CV / Gate control */
Operation operation;            /**> Current operator for osc manipulation */
WaveForms shape;                /**> Actual waveform */
char line1[9]{"        "};      /**> First line of the LCD */
char line2[9]{"        "};      /**> Second line of the LCD */
uint8_t channel{1};             /**> MIDI channel for output */
uint8_t maxStep{15};            /**> Last step in sequence */
uint8_t midiThru{0};            /**> Active MIDI mode */
uint8_t minStep{0};             /**> First step in sequence */
uint8_t mod{0};                 /**> Modificator */
uint8_t note{0};                /**> Active note */
uint8_t octave{0};              /**> Active octave */
uint8_t step{0};                /**> Active step */
uint8_t osc{0};                 /**> Current position in the WAVES table */
uint32_t duration{0};           /**> Used for note length/pause in sequencer */
uint32_t currentMillis{0};      /**> Used for note length/pause in sequencer */
uint32_t previousMillis{0};     /**> Used for note length/pause in sequencer */
bool ledLeft{false};            /**> State of left LED */
bool ledRight{false};           /**> State of right LED */
bool running{false};            /**> State of sequencer */
bool standingStep{false};       /**> State for manual sequencing */
bool upwards{true};             /**> Direction state */
bool sample{false};             /**> MIDI channel changing for volca sample */

/** Potentiometers used by the sequencer */
Pot pots[16] {
    {2, 3, LOW,   LOW, A4,   1,   0, INH_1, 0}, //  1 X0
    {2, 3, HIGH,  LOW, A4,   2,   0, INH_1, 0}, //  2 X1
    {2, 3, LOW,  HIGH, A4,   4,   0, INH_1, 0}, //  3 X2
    {2, 3, HIGH, HIGH, A4,   8,   0, INH_1, 0}, //  4 X3
    {2, 3, LOW,   LOW, A5,  16,   0, INH_1, 0}, //  5 Y0
    {2, 3, HIGH,  LOW, A5,  32,   0, INH_1, 0}, //  6 Y1
    {2, 3, LOW,  HIGH, A5,  64,   0, INH_1, 0}, //  7 Y2
    {2, 3, HIGH, HIGH, A5, 128,   0, INH_1, 0}, //  8 Y3
    {4, 5, LOW,   LOW, A6,   0, 128, INH_2, 0}, //  9 X0
    {4, 5, HIGH,  LOW, A6,   0,  64, INH_2, 0}, // 10 X1
    {4, 5, LOW,  HIGH, A6,   0,  32, INH_2, 0}, // 11 X2
    {4, 5, HIGH, HIGH, A6,   0,  16, INH_2, 0}, // 12 X3
    {4, 5, LOW,   LOW, A7,   0,   8, INH_2, 0}, // 13 Y0
    {4, 5, HIGH,  LOW, A7,   0,   4, INH_2, 0}, // 14 Y1
    {4, 5, LOW,  HIGH, A7,   0,   2, INH_2, 0}, // 15 Y2
    {4, 5, HIGH, HIGH, A7,   0,   1, INH_2, 0}  // 16 Y3
};

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

/** Sends a noteOff to all MIDI notes that are playable by a keyboard */
void silence() {
    MIDI.send(midi::ControlChange, 123, 0, channel);
    notes.clear();
    digitalWrite(GATE, LOW);
    note = 0;
}

/** Removes everything from display */
void clearLCD() {
    strncpy(line1, "        ", 8);
    strncpy(line2, "        ", 8);
}

/** Refreshes the display */
void updateLCD() {
    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);
}

/** Prints the current note on the second row of the display
 * @param n MIDI-note (0 ... 127)
 */
void printNote(uint8_t n) {
    int octave{n / 12 - 1}, note{n % 12};
    strncpy(line2, "        ", 8);
    char sup = DE[(note << 1) + 1];
    line2[0] = DE[note << 1];
    if (sup == ' ') {
        itoa(octave, &line2[1], 10);
        line2[2] = sup;
    } else {
        line2[1] = sup;
        itoa(octave, &line2[2], 10);
    }
    line2[3] = ' ';
    strncpy(&line2[4], &ES[note << 2], 4);
}

/** Plays a note and updates the display
 * @param ch channel (0 ... 16)
 * @param n MIDI-note number (0 ... 127)
 * @param v velocity (0 ... 127)
 */
void noteOn(uint8_t ch, uint8_t n, uint8_t v) {
    if (sample && ch == 10) {
        MIDI.sendNoteOn(60, 127, n % 11);
        MIDI.sendNoteOff(60, 127, n % 11);
    } else if (ch == channel) {
        digitalWrite(GATE, HIGH);
        notes.put(n);
        printNote(n);
        pwmCV.write8(PWM_CV_VALUES[n]);
        OCR5A = OCR5A_VALUES[n];
        note = n;
    }
}

/** Silences a note and updates the display
 * @param ch channel (0 ... 16)
 * @param n MIDI-note number (0 ... 127)
 * @param v velocity (0 ... 127)
 */
void noteOff(uint8_t ch, uint8_t n, uint8_t v) {
    notes.drop(n);
    clearLCD();
    if (notes.empty()) {
        pwmCV.write8(LOW);
        pwmOsc.write8(LOW);
        digitalWrite(GATE, LOW);
        note = 0;
    } else {
        noteOn(ch, notes.getLast(), 127);
    }
}

/** Prepares the Arduino Mega 2560 */
void setup() {
    lcd.begin(8, 2);

    /* MIDI */
    Serial1.begin(31250);                              // MIDI baud rate
    MIDI.begin(MIDI_CHANNEL_OMNI);
    MIDI.setHandleNoteOn(noteOn);
    MIDI.setHandleNoteOff(noteOff);

    /* Ports used for LED rows */
    DDRA = 255;
    DDRC = 255;

    /* Timer2 for MIDI */
    cli();
    OCR2A = 255;                                       // 7.8 kHz
    TCCR2B = (1 << WGM21) | (1 << CS11);               // CTC, prescaler 8
    TIMSK2 = (1 << OCIE2A);                            // compare interrupt

    /* Timer3 for pauses and oscilltor manipulation */
    TCCR3B = (1 << WGM12) | (1 << CS12) | (1 << CS10); // CTC, prescaler 1024
    TIMSK3 = (1 << OCIE3A);                            // compare interrupt

    /* Timer5 for oscillator */
    TCCR5B = (1 << WGM53) | (1 << CS10);               // CTC, no prescaling
    TIMSK5 = (1 << OCIE5A);                            // compare interrupt
    sei();

    /* Set control pins for the 2 MCB14052BCP multiplexers */
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(INH_1, OUTPUT);
    pinMode(INH_2, OUTPUT);
    pinMode(PWM_OSC, OUTPUT);

    /* Set pullup input pins (connected to GND) */
    pinMode(B_1, INPUT_PULLUP);
    pinMode(B_2, INPUT_PULLUP);
    pinMode(B_3, INPUT_PULLUP);
    pinMode(SWI_EXT_CLK, INPUT_PULLUP);
    pinMode(SWI_INT_CLK, INPUT_PULLUP);
    pinMode(SWI_LARGE, INPUT_PULLUP);

    /* Set normal input pins */
    pinMode(SYNC_IN, INPUT);
    pinMode(SYNC_OUT, INPUT);

    /* Initalize CV pin for PWM with 8 bit resolution */
    pwmCV.init(PWM_CV, DAC<TimerOne>::Resolution::BITS_8);
    pwmOsc.init(PWM_OSC, DAC<TimerFour>::Resolution::BITS_8);

    /* Loading settings from internal EEPROM */
    uint8_t eeprom{eeprom_read_byte(static_cast<uint8_t*>(0))};
    uint8_t temp(eeprom << 4);
    channel = temp ? temp >> 4 : 1;
    sample = eeprom > 63;
    temp = eeprom << 2;
    midiThru = temp >> 6;
    
    if(midiThru == 0) {
        MIDI.turnThruOff();
    } else {
        MIDI.turnThruOn(midi::MidiFilterMode(midiThru));
    }
}

/** Reading a potentiometer (writing into val) form pots @param pot step */
void readPot(uint16_t pot) {
    Pot& p = pots[pot];
    digitalWrite(p.inhibit, LOW);
    digitalWrite(p.A, p.a);
    digitalWrite(p.B, p.b);
    PORTA = p.portA;
    PORTC = p.portC;
    p.val = readDAC(p.out);
    digitalWrite(p.inhibit, HIGH);
}

/**
 * Mutes a previous step in one of those 2 conditions:
 * - The note has reached is length and shall be muted before the step duration
 *   has completed.
 * - The actual step has finished and a new step shall be activated.
 */
void muteStep() {
    uint8_t prevNote{static_cast<uint8_t> (OCTAVES[octave] + note)};
    notes.clear();
    if (sample) {
        MIDI.sendNoteOff(60, 0, prevNote % 10 + 1);
    } else {
        MIDI.sendNoteOff(prevNote, 0, channel);
    }
    note = 0;
}

/** Generating audio and MIDI for a step in the sequence */
void playStep() {
    if (note) {
        muteStep();
    }
    octave = (readDAC(POT_OCTAVE) >> 8) + 1;
    readPot(step);
    if (pots[step].val != LOW) {
        note = (pots[step].val >> 5) + (readDAC(POT_NOTE) >> 6);
        uint8_t midiNote{static_cast<uint8_t>(OCTAVES[octave] + note)};
        printNote(midiNote);
        noteOn(channel, midiNote, 127);
        if (sample) {
            MIDI.sendNoteOff(60, 0, midiNote % 10 + 1);
        } else {
            MIDI.sendNoteOn(midiNote, 127, channel);
        }
    } else {
        note = 0;
        strncpy(line2, "--------", 8);
        digitalWrite(GATE, LOW);
    }
}

/** Processing the sequence */
void processStep() {
    minStep = readDAC(POT_MIN_STEP) >> 6;
    maxStep = readDAC(POT_MAX_STEP) >> 6;
    if (step <= maxStep && step >= minStep) {
        switch(direction) {
            case Direction::FORWARDS: {
                if (step == maxStep) {step = minStep;} else {++step;} break;};
            case Direction::BACKWARDS: {
                if (step == minStep) {step = maxStep;} else {--step;} break;};
            case Direction::PINGPONG: {
                if (step == minStep) {upwards = true;}
                else if (step == maxStep) {upwards = false;}
                if (upwards) {++step;} else {--step;} break;};
        }
    }
    playStep();
}

/** Timer2 for MIDI and audio out*/
ISR(TIMER2_COMPA_vect) {
    MIDI.read();
    if (note > 1) {        
        switch(operation) {
            case Operation::NONE : {
                pwmOsc.write8(WAVES[static_cast<uint8_t> (shape)][osc]);
            } break;
            case Operation::OR   : {
                pwmOsc.write8(WAVES[static_cast<uint8_t> (shape)][osc] | mod);
            } break;
            case Operation::SUB  : {
                pwmOsc.write8(WAVES[static_cast<uint8_t> (shape)][osc] - mod);
            } break;
            case Operation::XOR  : {
                pwmOsc.write8(WAVES[static_cast<uint8_t> (shape)][osc] ^ mod);
            } break;
            case Operation::ADD  : {
                pwmOsc.write8(WAVES[static_cast<uint8_t> (shape)][osc] + mod);
            } break;
            case Operation::NOT  : {
                pwmOsc.write8(WAVES[static_cast<uint8_t> (shape)][~osc]);
            } break;
            case Operation::MOD  : {
                pwmOsc.write8(WAVES[static_cast<uint8_t> (shape)][osc] % mod);
            } break;
            case Operation::AND  : {
                pwmOsc.write8(WAVES[static_cast<uint8_t> (shape)][osc] & mod);
            } break;
        }
    } else {
        pwmOsc.write8(LOW);
    }
}

/** Timer3 for pot readings and note lengths */
ISR(TIMER3_COMPA_vect) {
    shape = static_cast<WaveForms> (readDAC(POT_WAVEFORM) >> 7);
    uint16_t interval{static_cast<uint16_t>(duration - readDAC(POT_DURATION))};
    uint16_t potVal{static_cast<uint16_t> (readDAC(POT_WAVESUB))};
    operation = static_cast<Operation> (potVal >> 7);
    mod = potVal ? (WAVES[potVal >> 8][(potVal + osc) % 128 ]) : 0;
    currentMillis = millis();
    if (running && (currentMillis - previousMillis) > interval) {
        previousMillis = currentMillis;
        muteStep();
    }
}

/** Timer5 for oscillator */
ISR(TIMER5_COMPA_vect) {
    if (osc > 127) { osc = 0; }
    ++osc;
}

/** Main loop of the Arduino Mega 2560 */
void loop() {
    PORTA = 0;
    PORTC = 0;
    if (digitalRead(SWI_INT_CLK) == LOW) {
        running = true;
        duration = 1024 - readDAC(POT_SPEED);
        delay(duration);
    } else if (digitalRead(SWI_EXT_CLK) == LOW) {
        running = true;
        duration = pulseIn(SYNC_IN, HIGH) >> 10;
    } else {
        running = false;
        uint8_t mode{0};
        updateLCD();
        if (digitalRead(B_1) == LOW && digitalRead(B_2) == LOW) {
            standingStep = !standingStep;
            strncpy(line1, "pasitos ", 8);
            updateLCD();
            delay(500);
        }

        /** Entering settings mode */
        while (digitalRead(SWI_LARGE) != LOW) {
            silence();
            digitalWrite(LED_RIGHT, HIGH);
            if (digitalRead(B_1) == LOW) {
                if (mode == 4) {mode = 0;} else { ++mode; }
            } else {strncpy(line2, "        ", 8);}
            /** Saving settings to internal EEPROM */
            if (digitalRead(B_3) == LOW) {
                uint8_t eeprom{channel};
                if (sample) {
                    eeprom += 64;
                }
                eeprom += (midiThru << 4);
                if (eeprom_is_ready()) {
                    eeprom_write_byte(static_cast<uint8_t*> (0), eeprom);
                }
                bool leds{true};
                for (uint8_t b{0}; b < 6; ++b) {
                    digitalWrite(LED_LEFT, leds);
                    digitalWrite(LED_RIGHT, leds);
                    leds = !leds;
                    delay(250);
                }
            }
            switch (mode) {
                case 0: {
                    digitalWrite(LED_LEFT, LOW);
                } break;
                case 1:
                {
                    strncpy(line1, "sample  ", 8);
                    strncpy(line2, sample ? "activo  " : "inactivo", 8);
                    if (digitalRead(B_2) == LOW) {
                        if (sample) {
                            sample = false;
                            strncpy(line2, "inactivo", 8);
                        } else {                            
                            sample = true;
                            strncpy(line2, "activo  ", 8);
                        }
                    }
                }
                break;
                case 2: {
                    strncpy(line1, "MIDI    ", 8);
                    strncpy(line2, "canal   ", 8);
                    if (digitalRead(B_2) == LOW) {
                        if (++channel > 16) {channel = 1;}
                    }
                    char* str = &line2[7];
                    if (channel > 9) {str = &line2[6];}
                    itoa(channel, str, 10);
                } break;
                case 3: {
                    strncpy(line1, "MIDIThru", 8);
                    if (digitalRead(B_2) == LOW) {
                        if (midiThru == 4) {midiThru = 0;} else {++midiThru;}
                    }
                    switch(midiThru) {
                        case 3: {strncpy(line2, "distinto", 8);} break;
                        case 0: {strncpy(line2, "apagado ", 8);} break;
                        case 2: {strncpy(line2, "mismo   ", 8);} break;
                        case 1: {strncpy(line2, "todo    ", 8);} break;
                    }
                    MIDI.turnThruOn(midi::MidiFilterMode(midiThru));
                }
            }
            updateLCD();
            delay(250);
            clearLCD();
            digitalWrite(LED_RIGHT, LOW);
        }
    }

    if (running) {
        if (digitalRead(B_1) == LOW && digitalRead(B_2) == LOW) {
            direction = Direction::PINGPONG;
        } else if (digitalRead(B_1) == LOW) {
            direction = Direction::BACKWARDS;
        } else if (digitalRead(B_2) == LOW) {
            direction = Direction::FORWARDS;
        }
        if (digitalRead(B_3) == LOW) {
            step = (direction == Direction::BACKWARDS) ? 0 : 15;
        }

        ledLeft = !ledLeft;
        digitalWrite(SYNC_OUT, HIGH);
        digitalWrite(LED_LEFT, ledLeft);
        processStep();
        updateLCD();
        digitalWrite(SYNC_OUT, LOW);
    } else {
        if (standingStep) {
            if (digitalRead(B_1) == LOW) {
                step = (step == 15) ? 0 : step + 1;
            } else if (digitalRead(B_2) == LOW) {
                step = (step == 0) ? 15 : step - 1;
            }
            playStep();
            delay(1024 - readDAC(POT_SPEED));
        }
    }
    if (running || note) {
        switch (operation) {
            case Operation::NONE : {line1[0] = ' ';} break;
            case Operation::SUB  : {line1[0] = '-';} break;
            case Operation::OR   : {line1[0] = '|';} break;
            case Operation::XOR  : {line1[0] = '^';} break;
            case Operation::NOT  : {line1[0] = '!';} break;
            case Operation::ADD  : {line1[0] = '+';} break;
            case Operation::AND  : {line1[0] = '&';} break;
            case Operation::MOD  : {line1[0] = '%';} break;
        }

        if (!running) {
            line1[1] = ' ';
        } else {
            switch (direction) {
                case Direction::BACKWARDS: {line1[1] = '<';} break;
                case Direction::FORWARDS:  {line1[1] = '>';} break;
                case Direction::PINGPONG:  {line1[1] = 'X';} break;
            }
        }

        switch (shape) {
            case WaveForms::SINE:     {strncpy(&line1[2], "arollo", 6);} break;
            case WaveForms::RAMP:     {strncpy(&line1[2], "subida", 6);} break;
            case WaveForms::SAWTOOTH: {strncpy(&line1[2], "bajada", 6);} break;
            case WaveForms::SQUARE:   {strncpy(&line1[2], "cajita", 6);} break;
            case WaveForms::TRIANGLE: {strncpy(&line1[2], "volcan", 6);} break;
            case WaveForms::EXP:      {strncpy(&line1[2], "yaguar", 6);} break;
            case WaveForms::RANDOM:   {strncpy(&line1[2], "katari", 6);} break;
            case WaveForms::MIX:      {strncpy(&line1[2], "caucho", 6);} break;
        }
    }
}
