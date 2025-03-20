
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <errno.h>
#include <ctype.h>

#include "rtttl.h"
#include "timers.h"
#include "pwm.h"
#include "IOconfig.h"

/*
 * I've used the following EBNF to program the parser.
 * Some songs online are not in a valid format as they
 * often specify the dot before the octave (e.g. a#.6)
 * instead of the correct format (e.g. a#6.).
 * 
 * <RTTTL> := <title> ":" [<control-section>] ":" <tone-commands>
 *
 * <title> := <string:10>
 *  
 * <control-section> := <def-tone-duration> "," <def-tone-scale> "," <def-beats>
 * <def-duration> := "d=" <duration>
 * <def-scale> := "o=" <scale>
 * <def-beats> := "b=" <beats-per-minute>
 * ; if not specified, defaults are 
 * ; duration=4, scale=6, beats-per-minute=63
 *  
 * <tone-commands> := <tone> ["," <tone-commands>]
 * <tone> := [<duration>] <tone> [<scale>] [<special-duration>]
 *  
 * <duration> := "1"|"2"|"4"|"8"|"16"|"32"
 * <scale> := "5"|"6"|"7"|"8"
 * <beats-per-minute> := "5"|"28"|"31"|"35"|"40"|"45"|"50"|"56"|"63"|
 *                       "70"|"80"|"90"|"100"|"112"|"125"|"140"|"160"|
 *                       "180"|"200"|"225"|"250"|"285"|"20"|"355"|"400"|
 *                       "450"|"500"|"565"|"635"|"715"|"800"|"900"
 * <tone> := "p"|"c"|"c#"|"d"|"d#"|"e"|"f"|"f#"|"g"|"g#"|"a"|"a#"|"b"|"h"
 * <special-duration> := "."
 */


// Create arrays that hold all valid values for the different
// numerable values in RTTTL.
const uint16_t validDurations[] = {1, 2, 4, 8, 16, 32};
const uint16_t validOctaves[] = {4, 5, 6, 7};
const uint16_t validBeats[] = {25, 28, 31, 35, 40, 45, 50, 56, 63, 70, 80, 90, 100, 112, 125, 140, 160, 180, 200, 225, 250, 285, 320, 355, 400, 450, 500, 565, 635, 715, 800, 900};

// This array holds all possible tones in the following array, where the tone is in one dimension
// and for each tone the the corresponding octave can be selected by the other dimension.
//      0  1  2  3  4  5  6  7  8  9 10 11 12
// 0(4) C C#  D D#  E  F F#  G G#  A A#  H  P
// 1(5) ...
// 2(6) ...
// 3(7) ...
const uint16_t notesByOctave[4][13] = {
  {261, 277, 293, 311, 329, 349, 369, 392, 415, 440, 466, 493, 0},
  {523, 554, 587, 622, 659, 698, 739, 783, 830, 880, 932, 987, 0},
  {1046, 1108, 1174, 1244, 1318, 1396, 1479, 1567, 1661, 1760, 1864, 1975, 0},
  {2093, 2217, 2349, 2489, 2637, 2793, 2959, 3135, 3322, 3520, 3729, 3951, 0},
  };


/**
 * Parsing - Helper Functions
 */

// Increase the pointer to the next character.
static void nextChar(RtttlReader *input)
{
  input->pos++;
}

// Expect that on the current position is a given
// character. In that case increase the pointer
// and return true. Otherwise, return false.
static bool expectChar(RtttlReader *input, char c)
{
  if (*(input->pos) == c)
  {
    nextChar(input);
    return true;
  }
  else
  {
    return false;
  }
}

// Validate a given integer if it is inside a given array.
static bool validateInt(const uint16_t* validNums, uint8_t len, uint16_t *num) {
  bool res = false;

  // Check each entry in the array against the number.
  // In case we find a match, early out.
  // Would be more efficient with a hash table, but this
  // will be sufficient in our case.
  for(int i = 0; i < len; i++) {
    if (validNums[i] == *num) {
      res = true;
      break;
    }
  }
  
  return res;
}

// Parse the dot.
static bool parseDot(RtttlReader *input) {
  // Return whether there is a dot or not.
  return expectChar(input, '.');
}

// Parse an integer value.
static bool parseInt(RtttlReader *input, uint16_t *num)
{
  // Check first whether the first char is not a number.
  if (isdigit(*(input->pos)) == 0) {
    return false;
  }
  
  // A pointer to the first char after the parsed number.
  char *newPos;
  // Parse the number out of the string.
  *num = strtol(input->pos, &newPos, 10);
  // Set the pointer to the new position.
  input->pos = newPos;

  // Return whether an error occurred or not.
  return num != 0 || errno != EINVAL;
}

// Parse the title.
static bool parseTitle(RtttlReader *input)
{
  // The title is limited to 10 characters.
  uint8_t titleLen = 0;
  
  // We are currently not interested in the
  // title so just skip it.
  while (*(input->pos) != ':' && titleLen < 10)
  {
    nextChar(input);
    titleLen++;
  }

  // We should now be exactly on the colon.
  return expectChar(input, ':');
}

// Parse a single default parameter.
static bool parseDefault(RtttlReader *input, RtttlDefaults *defaults, char c, bool last)
{
  bool noNum = true;
  bool res = false;

  // Check for c= in the string for a given char c.
  if (expectChar(input, c) && expectChar(input, '='))
  {
    uint16_t *defaultAddr;
    const uint16_t *validNums;
    uint8_t len;

    // Set the variables to the correct values for each
    // case.
    switch(c) {
      case 'd':
        defaultAddr = &defaults->d;
        validNums = validDurations;
        len = sizeof(validDurations) / sizeof(uint16_t);
        break;
      case 'o':
        defaultAddr = &defaults->o;        
        validNums = validOctaves;
        len = sizeof(validOctaves) / sizeof(uint16_t);
        break;
      case 'b':        
        defaultAddr = &defaults->b;
        validNums = validBeats;
        len = sizeof(validBeats) / sizeof(uint16_t);
        break;
    }

    // Track that this parameter is given in the string.
    noNum = false;

    // Parse the integer number and validate its value.
    bool res1 = parseInt(input, defaultAddr);
    bool res2 = validateInt(validNums, len, defaultAddr);
    
    res = res1 & res2;
  }

  // In case this is the last parameter, there should
  // be a colon after it.
  char delimiter;
  switch (last) {
    case false:
      delimiter = ',';
      break;
    case true:
      delimiter = ':';
      break;
  }

  // In case there was no parameter given, we only need to
  // check for the delimiter. Otherwise, we need to have
  // a correct result and then the delimiter.
  res = (noNum | res) & expectChar(input, delimiter);
  
  return res;
}

// Parse the default parameters.
static bool parseDefaults(RtttlReader *input, RtttlDefaults *defaults)
{
  // Set the defaults in case there are not defaults defined 
  // in the string.
  defaults->d = 4;
  defaults->o = 6;
  defaults->b = 63;

  // Parse the defaults.
  bool res1 = parseDefault(input, defaults, 'd', false);
  bool res2 = parseDefault(input, defaults, 'o', false);
  bool res3 = parseDefault(input, defaults, 'b', true);  

  // Pre-calculate the duration for a whole tone.
  defaults->bDuration =  ((uint32_t) 60 * 1000 * 4 / defaults->b);
  
  return res1 & res2 & res3;
}

// Parse the duration.
static bool parseDuration(RtttlReader *input, uint16_t *duration) {
  // Check that we are on a digit to exclude negative numbers.
  if(isdigit(*(input->pos)) == 0) {
    return true; 
  }

  // Parse and validate the integer value.
  return parseInt(input, duration) & validateInt(validDurations, sizeof(validDurations) / sizeof(uint16_t), duration);
}

// Parse the tone.
static bool parseNote(RtttlReader *input, uint8_t *note) {
  // Track if we need to check for a sharp later on.
  bool checkForSharp = true;

  switch(*(input->pos)) {
    case 'c':
      *note = C;
      break;
    case 'd':
      *note = D;
      break;
    case 'e':
      *note = E;
      checkForSharp = false;
      break;
    case 'f':
      *note = F;
      break;
    case 'g':
      *note = G;
      break;
    case 'a':
      *note = A;
      break;
    // Also allow b as this is often used instead of h.
    case 'b':
    case 'h':
      *note = H;
      checkForSharp = false;
      break;
    case 'p':
      *note = P;
      checkForSharp = false;
      break;
     default:
        return false;
        break;
  }

  nextChar(input);

  // In case we need to watch out for a sharp,
  // check if it is there and switch to this
  // tone in that case.
  if(checkForSharp && expectChar(input, '#')) {
      (*note)++;
  }
  
  return true;
}

// Parse the octave.
static bool parseOctave(RtttlReader *input, uint16_t *octave) {
  // Check that we are on a digit to exclude negative numbers.
  if(isdigit(*(input->pos)) == 0) {
    return true; 
  }
  
  // Parse and validate the integer value.
  return parseInt(input, octave) & validateInt(validOctaves, sizeof(validOctaves) / sizeof(uint16_t), octave);
}


// Parse a single tone.
static bool parseTone(RtttlReader *input, RtttlDefaults * defaults, RtttlNote* notes, uint16_t index) {
  // Parse the duration.
  uint16_t duration = 0;
  bool res1 = parseDuration(input, &duration);

  // Parse the tone.
  uint8_t note;
  bool res2 = parseNote(input, &note);

  // Parse the octave.
  uint16_t octave = 0;
  bool res3 = parseOctave(input, &octave);

  // Check if there is a dot next to it.
  bool dotFound = parseDot(input);

  // In case the duration or octave are not given, fallback
  // to the default values.
  duration = duration == 0 ? defaults->d : duration;
  octave = octave == 0 ? defaults->o : octave;

  // Select the correct frequency based on the tone and the octave out of the
  // mapping array.
  notes[index].frequency = notesByOctave[octave - 4][note];

  // Set the duration based on the duration and the possible dot after the tone.
  notes[index].duration = defaults->bDuration / duration + (defaults->bDuration * dotFound) / (duration * 2);
  
  return res1 & res2 & res3;
}

// Parse the list of tones in the third part of the string.
static bool parseTones(RtttlReader *input, RtttlDefaults *defaults, RtttlNotes *notes) {
  // Track the index of the tone in the array that we currently parse.
  uint16_t index = 0;

  while(true) {
    if (index >= notes->bufferLen) {
      // Boundary of tone array reached. Error.
      return false;
    }
    
    // Parse the tone.
    bool res = parseTone(input, defaults, notes->notes, index);

    // In case it failed, early out.
    if (!res) {
      return false;
    }

    index++;

    // In case we reached the end of the string, break.
    if(*(input->pos) == 0) {
      break;
    // Otherwise there should be a comma after the tone.
    } else if(!expectChar(input, ',')) {
      return false;
    }    
  }

  // Set the number of elements that have been loaded into the array.
  notes->notesLen = index;
  return true;
}

/**
 * Parsing - Main Function
 */

// Main entry point to parse the RTTTL string.
bool parseRTTTL(RtttlReader *input, RtttlNotes *output)
{
  // Parse the title first.
  bool res1 = parseTitle(input);

  // Then parse the defaults after the first colon.
  RtttlDefaults defaults;
  bool res2 = parseDefaults(input, &defaults);

  // Finally parse the tones after the second colon.
  bool res3 = parseTones(input, &defaults, output);
  
  // Accumulate the results.
  return res1 & res2 & res3;
}

/**
 * RTTTL API
 */
#define MAX_NOTES_PER_SONG 128

volatile bool songRepeat = false;
volatile bool songPlaying = false;
// Holds the song currently playing.
volatile RtttlNotes rtttlNotes;

static const char *const songRTTTL[SONG_COUNT] = {
    [SONG_MISSION_IMPOSSIBLE] = "MissionImp:d=16,o=6,b=100:32d,32d#,32d,32d#,32d,32d#,32d,32d#,32d,32d,32d#,32e,32f,32f#,32g,g,8p,g,8p,a#,p,c7,p,g,8p,g,8p,f,p,f#,p,g,8p,g,8p,a#,p,c7,p,g,8p,g,8p,f,p,f#,p,a#,g,2d,32p,a#,g,2c#,32p,a#,g,2c,a#5,8c,2p,32p,a#5,g5,2f#,32p,a#5,g5,2f,32p,a#5,g5,2e,d#,8d",
    [SONG_CANTINA] = "Cantina:d=8,o=5,b=250:a,p,d6,p,a,p,d6,p,a,d6,p,a,p,g#,4a,a,g#,a,4g,f#,g,f#,4f.,d.,16p,4p.,a,p,d6,p,a,p,d6,p,a,d6,p,a,p,g#,a,p,g,p,4g.,f#,g,p,c6,4a#,4a,4g",
    [SONG_SUPER_MARIO_UNDERGROUND] = "SMBundergr:d=16,o=6,b=100:c,c5,a5,a,a#5,a#,2p,8p,c,c5,a5,a,a#5,a#,2p,8p,f5,f,d5,d,d#5,d#,2p,8p,f5,f,d5,d,d#5,d#,2p,32d#,d,32c#,c,p,d#,p,d,p,g#5,p,g5,p,c#,p,32c,f#,32f,32e,a#,32a,g#,32p,d#,h5,32p,a#5,32p,a5,g#5",
    [SONG_MUPPETS] = "Muppets:d=4,o=5,b=250:c6,c6,a,h,8a,h,g,p,c6,c6,a,8h,8a,8p,g.,p,e,e,g,f,8e,f,8c6,8c,8d,e,8e,8e,8p,8e,g,2p,c6,c6,a,h,8a,h,g,p,c6,c6,a,8h,a,g.,p,e,e,g,f,8e,f,8c6,8c,8d,e,8e,d,8d,c",
    [SONG_GADGET] = "Gadget:d=16,o=5,b=50:32d#,32f,32f#,32g#,a#,f#,a,f,g#,f#,32d#,32f,32f#,32g#,a#,d#6,4d6,32d#,32f,32f#,32g#,a#,f#,a,f,g#,f#,8d#",
    [SONG_SMURFS] = "Smurfs:d=32,o=5,b=200:4c#6,16p,4f#6,p,16c#6,p,8d#6,p,8h,p,4g#,16p,4c#6,p,16a#,p,8f#,p,8a#,p,4g#,4p,g#,p,a#,p,h,p,c6,p,4c#6,16p,4f#6,p,16c#6,p,8d#6,p,8h,p,4g#,16p,4c#6,p,16a#,p,8h,p,8f,p,4f#",
};

static RtttlNote notesArray[SONG_COUNT][MAX_NOTES_PER_SONG];
static RtttlNotes preParsedSongs[SONG_COUNT];


bool parseAllSongs(void) {
    bool status = true;
    
    for (RtttlSong song = 0; song < SONG_COUNT; song++) {
        preParsedSongs[song].notes = notesArray[song];
        preParsedSongs[song].bufferLen = MAX_NOTES_PER_SONG;
        preParsedSongs[song].notesLen = 0;
        preParsedSongs[song].noteIndex = 0;

        RtttlReader reader = {.pos = songRTTTL[song]};
        
        status &= parseRTTTL(&reader, &preParsedSongs[song]);
        
        if (!status) {
            preParsedSongs[song].notesLen = 0;
        }
    }
    
    return status;
}

bool playSong(RtttlSong song, bool repeat) {
    if (song >= SONG_COUNT)
        return false;

    rtttlNotes = preParsedSongs[song];

    songRepeat = repeat;
    songPlaying = true;
    rtttlNotes.noteIndex = 0;

    if (rtttlNotes.notes[0].frequency > 0) {
        setPWMFrequency(BUZZ_PWM_MODULE, rtttlNotes.notes[0].frequency);
        setPWMState(BUZZ_PWM_MODULE, BUZZ_PWM_CHANNEL, true);
    } else {
        setPWMState(BUZZ_PWM_MODULE, BUZZ_PWM_CHANNEL, false);
    }

    setPWMDutyCycle(BUZZ_PWM_MODULE, BUZZ_PWM_CHANNEL, 0.5);
    registerTimerCallback(TIMER_3, songISR);

    return true;
}

void stopSong(void) {
    setPWMState(BUZZ_PWM_MODULE, BUZZ_PWM_CHANNEL, false);
    rtttlNotes.noteIndex = 0;
    songPlaying = false;
}


// Track the time passed within this timer callback.
static int16_t songISR(void) {
    static uint32_t rtttlTimeCount = 0;
    
    // 1) First, increment the time counter.
    rtttlTimeCount++;
    
    // 2) Check if we've reached the current note's duration.
    if (rtttlTimeCount >= rtttlNotes.notes[rtttlNotes.noteIndex].duration)
    {
        // Advance to next note
        rtttlNotes.noteIndex++;
      
        if (rtttlNotes.noteIndex >= rtttlNotes.notesLen)
        {
          if (songRepeat) {
              rtttlNotes.noteIndex = 0;
          } else {
              stopSong();
              return 0;
          }
        }
      
        // Reset and load the next note
        rtttlTimeCount = 0;

        // Turn of PWM in case frequency is zero, enable otherwise
        if (songPlaying && rtttlNotes.notes[rtttlNotes.noteIndex].frequency > 0) {
            setPWMFrequency(BUZZ_PWM_MODULE, rtttlNotes.notes[rtttlNotes.noteIndex].frequency);
            setPWMState(BUZZ_PWM_MODULE, BUZZ_PWM_CHANNEL, true);
        } else {
            setPWMState(BUZZ_PWM_MODULE, BUZZ_PWM_CHANNEL, false);
        }
    }
    
    return 1;
}