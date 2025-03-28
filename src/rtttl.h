#ifndef RTTTL_H
#define	RTTTL_H

#include <stdint.h>

#include "timers.h"

typedef enum {
    SONG_MISSION_IMPOSSIBLE,
    SONG_CANTINA,
    SONG_SUPER_MARIO_UNDERGROUND,
    SONG_MUPPETS,
    SONG_GADGET,
    SONG_SMURFS,
    SONG_COUNT // Must always be the last one in the enum.
} RtttlSong;

typedef struct
{
  uint16_t duration;
  uint16_t frequency;
}
RtttlNote;

typedef struct
{
    RtttlNote *notes;
    uint16_t bufferLen;
    uint16_t notesLen;
    uint16_t noteIndex;
} RtttlNotes;

typedef struct
{
  const char *pos;
}
RtttlReader;

typedef struct
{
  uint16_t d;
  uint16_t o;
  uint16_t b;
  // Duration for a whole note.
  // Determined by 60 * 1000 * 4 / b.
  uint16_t bDuration;
}
RtttlDefaults;

#define NOTE_C  0
#define NOTE_CS 1
#define NOTE_D  2
#define NOTE_DS 3
#define NOTE_E  4
#define NOTE_F  5
#define NOTE_FS 6
#define NOTE_G  7
#define NOTE_GS 8
#define NOTE_A  9
#define NOTE_AS 10
#define NOTE_H  11
#define NOTE_P  12

/**
 * @brief Parses an RTTTL string into structured note data. Requires an
 * RtttlReader with the RTTTL data and an RtttlNotes structure for storing
 * the parsed result. Returns true on success, false otherwise.
 */
bool parseRTTTL(RtttlReader *input, RtttlNotes *output);

/**
 * @brief Starts playing a specified song. Takes a RtttlSong enum value
 * to choose the song and a boolean indicating whether it should repeat.
 * Returns true if the song was successfully started, false otherwise.
 */
bool playSong(RtttlSong song, bool repeat, Timer_t timer);

/**
 * @brief Stops the currently playing song if any is active.
 */
void stopSong(void);

/**
 * @brief Parses all songs defined in the RtttlSong enum. Returns true
 * if parsing is successful for all, false otherwise.
 */
bool parseAllSongs(void);

/**
 * @brief Interrupt service routine for song playback. Called periodically to
 * advance the song's notes. Returns an 0 when the song has been played or stopped.
 * Returns 1 otherwise.
 */
static int16_t songISR(void);

#endif	/* RTTTL_H */