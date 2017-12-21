# Yakruna #

Yakruna uses an Arduino MEGA 2560 as main component. It is a musical device 
that can act as a 5 octave MIDI to CV/Gate converter for KORG monotribe 
(or maybe other hardware synthesizers). It can also route MIDI-input to 
different channels for the usage with KORG volca sample. It is usable with 
the KORG sync-signal. Finally it also has a built-in 16 step sequencer and it 
can produce audio.

## Requirements ##
cmake arduino-core

## Installation ##
mkdir build  
cd build  
cmake ..  
make upload

## Changes ##
### 1.0.1 (2017/12/21) ###
- Load MIDI Thru correctly on start
### 1.0.1 (2017/06/17) ###
- Receiving Korg volca sample MIDI in on channel 10 (R channel of Roland MC 505)
- Only playing notes from the channel set (and channel 10 in sample mode)

### 1.0.0 (2016/06/19) ###
- Initial version

