/**
 * @file        yakruna.h
 * @author      Eduardo Hahn Paredes <cumbiamberos@gmail.com>
 * @copyright   © 2016, Eduardo Hahn Paredes, Ecuador
 * @version     1.0
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
 * Contains constants, struct declarations and include definitions.
 *
 * Links:
 * - https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads (LCD)
 * - https://github.com/FortySevenEffects/arduino_midi_library (MIDI)
 * - https://github.com/PaulStoffregen/TimerOne (Timer)
 */

#ifndef YAKRUNA_H
#define YAKRUNA_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <MIDI.h>
#include <pins_arduino.h>
#include <TimerOne.h>
#include <TimerFour.h>
#include <Wire.h>
#include <avr/eeprom.h>

static const uint8_t B_1{10};             /**< Most left push button */
static const uint8_t B_2{11};             /**< Second left push button */
static const uint8_t B_3{9};              /**< Most right push button */
static const uint8_t INH_1{6};            /**< First MC14052BCP INH bit */
static const uint8_t INH_2{7};            /**< Second MC14052BCP INH bit */
static const uint8_t GATE{40};            /**< GATE (binary) */
static const uint8_t LED_RIGHT{52};       /**< Right LED */
static const uint8_t LED_LEFT{53};        /**< Left LED */
static const uint8_t POT_DURATION{A13};   /**< Note length for sequencer */
static const uint8_t POT_MAX_STEP{A10};   /**< Last step in sequence */
static const uint8_t POT_MIN_STEP{A9};    /**< First step in sequence */
static const uint8_t POT_NOTE{A12};       /**< Note for sequencer */
static const uint8_t POT_OCTAVE{A11};     /**< Octave for sequencer */
static const uint8_t POT_SPEED{A8};       /**< Speed for sequencer */
static const uint8_t POT_WAVEFORM{A14};   /**< Oscillator waveform */
static const uint8_t POT_WAVESUB{A15};    /**< Waveform manipulator */
static const uint8_t PWM_CV{13};          /**< CV (pulse width modulated) */
static const uint8_t PWM_OSC{8};          /**< Oscillator output (PWM) */
static const uint8_t SWI_EXT_CLK{44};     /**< External clock (by SYNC_IN) */
static const uint8_t SWI_INT_CLK{42};     /**< Internal clock (by POT_SPEED) */
static const uint8_t SWI_LARGE{47};       /**< Main switch */
static const uint8_t SYNC_IN{50};         /**< Sync-Out jack */
static const uint8_t SYNC_OUT{48};        /**< Sync-In jack */

/** String used by the printNote function */
static const char* DE{"C C#D D#E F F#G G#A A#H H#"};
static const char* ES{"do  do# re  re# mi  fa  fa# sol sol#la  la# si  si# "};

/** Note increments by octave */
static const uint8_t OCTAVES[10]{0, 12, 24, 32, 48, 60, 72, 84, 96, 108};

/** Movement directions for the sequencer between MIN_STEP and MAX_STEP */
enum class Direction : uint8_t {
    FORWARDS = 1,  /**< Movement from left to right */
    BACKWARDS = 2, /**< Movement from right to left */
    PINGPONG = 3   /**< Changing direction in every cilce */
};

/** Operations for the waveform combinations */
enum class Operation : uint8_t {
    NONE = 0, /**<   */
     SUB = 1, /**< - */
      OR = 2, /**< | */
     XOR = 3, /**< ^ */
     AND = 4, /**< & */
     NOT = 5, /**< ! */
     ADD = 6, /**< + */
     MOD = 7, /**< % */
};

/** Very simple collection to keep track of active notes */
struct NoteList {
public:
    /** Removes all notes from the list */
    void clear() {
        notePtr = 0;
    }

    /** Removes a note from the list @param note MIDI-note number */
    void drop(uint8_t note) {
        bool found{false};
        for (uint8_t i{0}; i < notePtr + 1; ++i) {
            if (notes[i] == note) {
                found = true;
            }
            if (found) {
                notes[i] = notes[i + 1];
            }
        }
        if (found) {
            notePtr--;
        }
    }

    /** Indicates emptyness @return true if there are no active notes */
    bool empty() {
        return notePtr < 1;
    }

    /** Gets the last note that has been inserted @return MIDI-note number */
    uint8_t getLast() {
        return notes[notePtr];
    }
    /** Adds a note to the list @param note MIDI-note number */
    void put(uint8_t note) {
        if (note != notes[notePtr] && notePtr < 10) {
            notes[++notePtr] = note;
        }
    }

    /** Number of notes that are currently in the list @return total */
    uint8_t size() {
        return notePtr;
    }

private:
    uint8_t notePtr{0};
    uint8_t notes[11]{0};
};

/** Sequence potentiometer */
struct Pot {
    const uint8_t A;       /**< Connection to pin A on MC14052BCP */
    const uint8_t B;       /**< Connection to pin B on MC14052BCP */
    const uint8_t a;       /**< Binary value for pin A on MC14052BCP */
    const uint8_t b;       /**< Binary value for pin B on MC14052BCP */
    const uint8_t out;     /**< Connection to X or Y on MC14052BCP */
    const uint8_t portA;   /**< Active LED on the upper row of LEDs */
    const uint8_t portC;   /**< Active LED on the lower row of LEDs */
    const uint8_t inhibit; /**< Inhibit pin on MC14052BCP */
    mutable uint16_t val;  /**< Value obtained form the potentiometer */
};


/**
 * This array contanis the precalculated values for the Timer5 compare register 
 * where the simple formula OCR5A = TCNT5 / (frequency * length of wavetable)
 * has been used. No prescaler is used so for the note C4 as example it is:
 * 16000000 / (261.63 * 128) = 477.77 -> 478
 */
static const uint16_t OCR5A_VALUES[128]{
    15289, /*C-1*/ 14431, /*C#-1*/ 13621, /*D-1*/  12856, /*D#-1*/     
    12135, /*E-1*/ 11454, /*F-1*/  10811, /*F#-1*/ 10204, /*G-1*/     
    9632,  /*G#-1*/ 9091, /*A-1*/   8580, /*A#-1*/  8099, /*B-1*/     
    7644,  /*C0*/   7215, /*C#0*/   6811, /*D0*/    6428, /*D#0*/     
    6068,  /*E0*/   5727, /*F0*/    5406, /*F#0*/   5102, /*G0*/     
    4816,  /*G#0*/  4545, /*A0*/    4290, /*A#0*/   4050, /*B0*/     
    3822,  /*C1*/   3608, /*C#1*/   3405, /*D1*/    3214, /*D#1*/     
    3034,  /*E1*/   2863, /*F1*/    2703, /*F#1*/   2551, /*G1*/     
    2408,  /*G#1*/  2273, /*A1*/    2145, /*A#1*/   2025, /*B1*/     
    1911,  /*C2*/   1804, /*C#2*/   1703, /*D2*/    1607, /*D#2*/     
    1517,  /*E2*/   1432, /*F2*/    1351, /*F#2*/   1276, /*G2*/     
    1204,  /*G#2*/  1136, /*A2*/    1073, /*A#2*/   1012, /*B2*/     
    956,   /*C3*/    902, /*C#3*/    851, /*D3*/     804, /*D#3*/     
    758,   /*E3*/    716, /*F3*/     676, /*F#3*/    638, /*G3*/     
    602,   /*G#3*/   568, /*A3*/     536, /*A#3*/    506, /*B3*/     
    478,   /*C4*/    451, /*C#4*/    426, /*D4*/     402, /*D#4*/     
    379,   /*E4*/    358, /*F4*/     338, /*F#4*/    319, /*G4*/     
    301,   /*G#4*/   284, /*A4*/     268, /*A#4*/    284, /*B4*/     
    239,   /*C5*/    225, /*C#5*/    213, /*D5*/     201, /*D#5*/     
    190,   /*E5*/    179, /*F5*/     169, /*F#5*/    159, /*G5*/     
    150,   /*G#5*/   142, /*A5*/     134, /*A#5*/    127, /*B5*/     
    119,   /*C6*/    113, /*C#6*/    106, /*D6*/     100, /*D#6*/     
    95,    /*E6*/     89, /*F6*/      84, /*F#6*/     80, /*G6*/     
    75,    /*G#6*/    71, /*A6*/      67, /*A#6*/     63, /*B6*/     
    60,    /*C7*/     56, /*C#7*/     53, /*D7*/      50, /*D#7*/     
    47,    /*E7*/     45, /*F7*/      42, /*F#7*/     40, /*G7*/     
    38,    /*G#7*/    36, /*A7*/      34, /*A#7*/     32, /*B7*/     
    30,    /*C8*/     28, /*C#8*/     27, /*D8*/      25, /*D#8*/     
    24,    /*E8*/     22, /*F8*/      21, /*F#8*/     20, /*G8*/     
    19,    /*G#8*/    18, /*A8*/      17, /*A#8*/     16, /*B8*/     
    15,    /*C9*/     14, /*C#9*/     13, /*D9*/      13, /*D#9*/     
    12,    /*E9*/     11, /*F9*/      11, /*F#9*/     10  /*G9*/ 
};

/** PWM for 5 octaves */
static const uint8_t PWM_CV_VALUES[128]{
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    4,   /*C2*/        8, /*C#2*/     12, /*D2*/      16, /*D#2*/
    20,  /*E2*/       24, /*F2*/      28, /*F#2*/     32, /*G2*/
    36,  /*G#2*/      40, /*A2*/      44, /*A#2*/     48, /*B2*/
    52,  /*C3*/       56, /*C#3*/     60, /*D3*/      64, /*D#3*/
    68,  /*E3*/       72, /*F3*/      76, /*F#3*/     80, /*G3*/
    84,  /*G#3*/      88, /*A3*/      92, /*A#3*/     96, /*B3*/
    100, /*C4*/      104, /*C#4*/    108, /*D4*/     116, /*D#4*/
    120, /*E4*/      124, /*F4*/     128, /*F#4*/    132, /*G4*/
    136, /*G#4*/     140, /*A4*/     144, /*A#4*/    148, /*B4*/
    152, /*C5*/      156, /*C#5*/    160, /*D5*/     164, /*D#5*/
    168, /*E5*/      172, /*F5*/     176, /*F#5*/    180, /*G5*/
    184, /*G#5*/     188, /*A5*/     192, /*A#5*/    196, /*B5*/
    200, /*C6*/      204, /*C#6*/    208, /*D6*/     216, /*D#6*/
    220, /*E6*/      224, /*F6*/     228, /*F#6*/    232, /*G6*/
    236, /*G#6*/     240, /*A6*/     244, /*A#6*/    248, /*B6*/
    252, /*C7*/      255, /*C#7*/
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

/**
 * Fast Pulse-with-modulation digital to analog convertion
 * This part is inspired py the FastPWMdac class of
 * Copyright by Albert van Dalen (Copyright 2015).
 * See http://www.avdweb.nl/arduino/libraries/fast-pwm-dac.html
 * and http://www.avdweb.nl/arduino/libraries/fast-10-bit-adc.html
 * for more information
 */
template <typename T>
struct DAC {
public:
    
    /** Resolution of the DAC */
    enum class Resolution : uint8_t {
        BITS_8 = 8,  /**< 8 bytes. Less precission but faster */
        BITS_10 = 10 /**< 10 bytes. More precission but slower */
    };

    /**
     * Has to be called before usage of any write function
     * @param pin Only 11, 12, 13 can be used for Timer1 8 and 9 for Timer4
     * @param resolution 8 or 10 bits
     * @param number timer can be 1 or 4
     */
    void init(uint8_t pin, Resolution resolution) {
        outPin = pin;
        if (resolution == Resolution::BITS_8) {
            timer.initialize(32);
        } else if (resolution == Resolution::BITS_10) {
            timer.initialize(128);
        }
        timer.pwm(pin, 0);
    }

    /** Converts 8 bits to voltage @param value 0 ... 255 */
    void write8(uint8_t value) {
        timer.setPwmDuty(outPin, value << 2);
    }

    /** Converts 10 bits to voltage @param value 0 ... 1023 */
    void write10(uint16_t value) {
        timer.setPwmDuty(outPin, value);
    }

private:
    T timer;
    uint8_t outPin;
};

/**
 * This static method can be called without initialization of the PWMDAC.
 * @param pin Analog input pin A0 to A15
 * @param prescalerBits 4 is default value
 * @return result form analogRead with a faster ADC clock
 */
static int readDAC(uint8_t pin, uint8_t prescalerBits = 4) {
    uint8_t originalClock{ADCSRA};
    ADCSRA = (ADCSRA & B11111000) | prescalerBits;
    int value = analogRead(pin);
    ADCSRA = originalClock;
    return value;
}

#endif
