#ifndef RTTTL_H
#define	RTTTL_H

#include <stdint.h>

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

// Holds the data of the song currently played
extern volatile RtttlNotes rtttlNotes;
extern volatile bool songRepeat;
extern volatile bool songPlaying;

#define C  0
#define CS 1
#define D  2
#define DS 3
#define E  4
#define F  5
#define FS 6
#define G  7
#define GS 8
#define A  9
#define AS 10
#define H  11
#define P  12

bool parseRTTTL(RtttlReader *input, RtttlNotes *output);
bool playSong(RtttlSong song, bool repeat);
void stopSong(void);
bool parseAllSongs(void);
int8_t songISR(void);

#endif	/* RTTTL_H */

