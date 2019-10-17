/*==========================================================================
  r2d2-voice
  main.c
  Copyright (c)2016 Kevin Boone
  Distributed under the terms of the GNU Public Licence, V3.0
==========================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sched.h>
#include <errno.h>
#include <getopt.h>
#include <alsa/asoundlib.h>
#include <sys/time.h>
#include <math.h>

// Sample rate in Hz
#define RATE 48000

// Data format -- everybody should support signed 16-bit
#define FORMAT SND_PCM_FORMAT_S16

// Output buffer size in usec
#define BUFFER_TIME 2000000
// Length of a single buffer update in usec. To get smooth whistles, this
//  needs to be < 20msec or so
#define PERIOD_TIME 10000

#define DEFAULT_DEVICE "plughw:0,0"

// These buffer sizes are worked out during hw initialization
static snd_pcm_sframes_t buffer_size;
static snd_pcm_sframes_t period_size;

// Types of sound available
typedef enum {sound_type_random=0, sound_type_glissando, sound_type_silence,
  sound_type_noise, sound_type_buzz}
  SoundType;

/* ==========================================================================
  generate_sine
  fill the buffer with sinewave, paying attention to the starting point
  (phase), which will have been carried forward from the previous
  period to avoid discontinuity
==========================================================================*/
static void generate_sine (const snd_pcm_channel_area_t *areas,
		int count, double *_phase, int freq, int fade)
  {
  static double max_phase = 2. * M_PI;
  double phase = *_phase;
  double step = max_phase*freq/(double)RATE;
  unsigned char *samples[1];
  int steps [1];
  int format_bits = snd_pcm_format_width (FORMAT);
  unsigned int maxval = (1 << (format_bits - 1)) - 1;
  int bps = format_bits / 8; /* bytes per sample */
  int phys_bps = snd_pcm_format_physical_width (FORMAT) / 8;
  int big_endian = snd_pcm_format_big_endian (FORMAT) == 1;
  samples[0] = (((unsigned char *)areas[0].addr)
    + (areas[0].first / 8));
  steps[0] = areas[0].step / 8;
  int start_count = count;

  while (count-- > 0)
    {
    int res, i;

    res = sin(phase) * maxval;
    if (freq == 0) res = 5;
    if (big_endian)
      {
      if (fade)
        res = res * count / start_count;
      for (i = 0; i < bps; i++)
        *(samples [0] + phys_bps - 1 - i) = (res >> i * 8) & 0xff;
      }
    else
      {
      if (fade)
        res = res * count / start_count;
      for (i = 0; i < bps; i++)
        *(samples[0] + i) = (res >> i * 8) & 0xff;
      }
    samples[0] += steps[0];

    phase += step;
    if (phase >= max_phase)
      phase -= max_phase;
    }
  *_phase = phase;
  }

/* ==========================================================================
  generate_buzz
==========================================================================*/
static void generate_buzz (const snd_pcm_channel_area_t *areas,
		int count, int freq, int fade)
  {
  static double max_phase = 2. * M_PI;
  double phase = 0;
  double step = max_phase*freq/(double)RATE;
  unsigned char *samples[1];
  int steps [1];
  int format_bits = snd_pcm_format_width (FORMAT);
  unsigned int maxval = (1 << (format_bits - 1)) - 1;
  int bps = format_bits / 8; /* bytes per sample */
  int phys_bps = snd_pcm_format_physical_width (FORMAT) / 8;
  int big_endian = snd_pcm_format_big_endian (FORMAT) == 1;
  samples[0] = (((unsigned char *)areas[0].addr)
    + (areas[0].first / 8));
  steps[0] = areas[0].step / 8;
  int start_count = count;

  while (count-- > 0)
    {
    int res, i;

    if (phase > M_PI)
      res = maxval / 4;
    else
      res = -maxval / 4;

    if (big_endian)
      {
      if (fade)
        res = res * count / start_count;
      for (i = 0; i < bps; i++)
        *(samples [0] + phys_bps - 1 - i) = (res >> i * 8) & 0xff;
      }
    else
      {
      if (fade)
        res = res * count / start_count;
      for (i = 0; i < bps; i++)
        *(samples[0] + i) = (res >> i * 8) & 0xff;
      }
    samples[0] += steps[0];

    phase += step;
    if (phase >= max_phase)
      phase -= max_phase;
    }
  }





/* ==========================================================================
  generate_silence
  fill the buffer with silence
  Note that we must actively generate silence -- we can't just pause,
  because the playback buffer would underrun
==========================================================================*/
static void generate_silence (const snd_pcm_channel_area_t *areas,
    int count)
  {
  unsigned char *samples[1];
  int steps [1];
  int format_bits = snd_pcm_format_width (FORMAT);
  int bps = format_bits / 8; /* bytes per sample */
  int phys_bps = snd_pcm_format_physical_width (FORMAT) / 8;
  int big_endian = snd_pcm_format_big_endian (FORMAT) == 1;
  samples[0] = (((unsigned char *)areas[0].addr)
    + (areas[0].first / 8));
  steps[0] = areas[0].step / 8;
  samples[0] += 0;

  while (count-- > 0)
    {
    // We might think that zero would be a good sample value for silence but,
    //  in fact, any constant value is silent. However, setting zero in my
    //  tests actually generates a low hiss -- no idea why
    int res = 5, i;

    if (big_endian)
      {
      for (i = 0; i < bps; i++)
        *(samples [0] + phys_bps - 1 - i) = (res >> i * 8) & 0xff;
      }
    else
      {
      for (i = 0; i < bps; i++)
        *(samples[0] + i) = (res >> i * 8) & 0xff;
      }
    samples[0] += steps[0];

    }
  }


/* ==========================================================================
  generate_noise
  fill the buffer with white noise
==========================================================================*/
static void generate_noise (const snd_pcm_channel_area_t *areas,
    int count)
  {
  unsigned char *samples[1];
  int steps [1];
  int format_bits = snd_pcm_format_width (FORMAT);
  int bps = format_bits / 8; /* bytes per sample */
  int phys_bps = snd_pcm_format_physical_width (FORMAT) / 8;
  int big_endian = snd_pcm_format_big_endian (FORMAT) == 1;
  unsigned int maxval = (1 << (format_bits - 1)) - 1;
  samples[0] = (((unsigned char *)areas[0].addr)
    + (areas[0].first / 8));
  steps[0] = areas[0].step / 8;
  samples[0] += 0;

  while (count-- > 0)
    {
    int i;
    int res = (double) rand() / RAND_MAX * (double) maxval;
    if (big_endian)
      {
      for (i = 0; i < bps; i++)
        *(samples [0] + phys_bps - 1 - i) = (res >> i * 8) & 0xff;
      }
    else
      {
      for (i = 0; i < bps; i++)
        *(samples[0] + i) = (res >> i * 8) & 0xff;
      }
    samples[0] += steps[0];

    }
  }



/*=========================================================================
  set_hwparams
  Initialize hardware
=========================================================================*/
static int set_hwparams(snd_pcm_t *handle,
		snd_pcm_hw_params_t *params,
		const char *argv0)
  {
  unsigned int rrate;
  snd_pcm_uframes_t size;
  int err, dir;
  err = snd_pcm_hw_params_any(handle, params);
  if (err < 0)
    {
    fprintf(stderr, "%s: No configurations available: %s\n", argv0,
       snd_strerror(err));
    return err;
    }
  err = snd_pcm_hw_params_set_access (handle, params,
    SND_PCM_ACCESS_RW_INTERLEAVED);
  if (err < 0)
    {
    fprintf(stderr,"%s: Access type not available: %s\n",
      argv0, snd_strerror(err));
    return err;
    }
  err = snd_pcm_hw_params_set_format (handle, params, FORMAT);
  if (err < 0)
    {
    fprintf(stderr,"%s: Sample format not available: %s\n",
      argv0, snd_strerror(err));
    return err;
    }
  err = snd_pcm_hw_params_set_channels (handle, params, 1);
  if (err < 0)
    {
    fprintf(stderr,"%s: Can't set mono playback: %s\n", argv0,
      snd_strerror(err));
    return err;
    }
  rrate = RATE;
  err = snd_pcm_hw_params_set_rate_near (handle, params, &rrate, 0);
  if (err < 0)
    {
    fprintf (stderr,"%s: Rate %iHz not available: %s\n", argv0,
        RATE, snd_strerror(err));
    return err;
    }
  if (rrate != RATE)
    {
    fprintf (stderr,
      "%s: Warning: Rate not available (requested %iHz, get %iHz)\n",
       argv0, RATE, err);
    }
  unsigned int buffer_time = BUFFER_TIME;
  err = snd_pcm_hw_params_set_buffer_time_near (handle, params,
    &buffer_time, &dir);
  if (err < 0)
    {
    fprintf(stderr,"%s: Unable to set buffer time %i: %s\n",
      argv0, buffer_time, snd_strerror(err));
    return err;
    }
  err = snd_pcm_hw_params_get_buffer_size(params, &size);
  if (err < 0)
    {
    fprintf(stderr, "%s: Unable to get buffer size: %s\n",
      argv0, snd_strerror(err));
    return err;
    }
  buffer_size = size;
  unsigned int period_time = PERIOD_TIME;
  err = snd_pcm_hw_params_set_period_time_near (handle, params,
      &period_time, &dir);
  if (err < 0)
    {
    fprintf (stderr, "%s: Unable to set period time %i: %s\n",
      argv0, period_time, snd_strerror(err));
    return err;
    }
  err = snd_pcm_hw_params_get_period_size (params, &size, &dir);
  if (err < 0)
    {
    fprintf(stderr, "%s: Unable to get period size: %s\n", argv0,
      snd_strerror(err));
    return err;
    }
  period_size = size;
  err = snd_pcm_hw_params (handle, params);
  if (err < 0)
    {
    fprintf (stderr, "%s: Unable to set hwparams: %s\n", argv0, snd_strerror(err));
    return err;
    }
  return 0;
  }

/*=========================================================================
  set_swparams
  Initialize buffering
=========================================================================*/
static int set_swparams(snd_pcm_t *handle, snd_pcm_sw_params_t *swparams,
    const char *argv0)
  {
  int err;
  err = snd_pcm_sw_params_current(handle, swparams);
  if (err < 0)
    {
    fprintf(stderr, "%s: Unable to determine current swparams: %s\n",
      argv0, snd_strerror(err));
    return err;
    }
  err = snd_pcm_sw_params_set_start_threshold (handle, swparams,
    (buffer_size / period_size) * period_size);
  if (err < 0)
    {
    fprintf(stderr, "%s: Unable to set start threshold: %s\n", argv0,
      snd_strerror(err));
    return err;
    }
  err = snd_pcm_sw_params_set_avail_min (handle, swparams, period_size);
  if (err < 0)
    {
    fprintf(stderr, "%s: Unable to set avail min for playback: %s\n", argv0,
      snd_strerror(err));
    return err;
    }
  err = snd_pcm_sw_params(handle, swparams);
  if (err < 0)
    {
    fprintf(stderr, "%s: Unable to set sw params for playback: %s\n", argv0,
      snd_strerror(err));
    return err;
    }
  return 0;
  }


/*=========================================================================
  play_sound
=========================================================================*/
void play_sound (snd_pcm_t *handle, SoundType sound_type, const int duration,
    const int pitch_duration, const int f1,
    const int f2)
  {
  double phase = 0;
  signed short *ptr;
  int err, cptr;
  signed short *samples;
  snd_pcm_channel_area_t *areas;

  int freq = f1;
  int time_per_period = PERIOD_TIME / 1000;
  int loops = duration / time_per_period;
  int f_increment = (f2 - f1) / loops;
  int loops_per_pitch_duration = pitch_duration / time_per_period;

  samples = (signed short*) malloc ((period_size *
    snd_pcm_format_physical_width (FORMAT)) / 8);
  areas = (snd_pcm_channel_area_t *) malloc (sizeof (snd_pcm_channel_area_t));

  areas[0].addr = samples;
  areas[0].first = 0;
  areas[0].step = snd_pcm_format_physical_width (FORMAT);

  int loop;
  for (loop = 0; loop < loops; loop++)
    {
    if (sound_type == sound_type_buzz)
      {
      if ((loop % loops_per_pitch_duration) == 0)
        freq = f1 + (f2 - f1) * (double) rand() / RAND_MAX ;
      generate_buzz (areas, period_size, freq, loop == loops - 1);
      }
    else if (sound_type == sound_type_random)
      {
      if ((loop % loops_per_pitch_duration) == 0)
        freq = f1 + (f2 - f1) * (double) rand() / RAND_MAX ;
      generate_sine (areas, period_size, &phase, freq, loop == loops - 1);
      }
    else if (sound_type == sound_type_glissando)
      {
      freq += f_increment;
      generate_sine (areas, period_size, &phase, freq, loop == loops - 1);
      }
    else if (sound_type == sound_type_noise)
      generate_noise (areas, period_size);
    else
      generate_silence (areas, period_size);
    ptr = samples;
    cptr = period_size;
    while (cptr > 0)
      {
      err = snd_pcm_writei(handle, ptr, cptr);
      if (err == -EAGAIN)
        continue;
      if (err < 0)
        {
        // Underrun -- should never happen
        break;
        }
      ptr += err;
      cptr -= err;
      }
    }

  free(areas);
  free(samples);
  }


/*=========================================================================
  setup_sound
=========================================================================*/
int setup_sound (snd_pcm_t **handle, const char *device, const char *argv0)
  {
  snd_pcm_hw_params_t *hwparams;
  snd_pcm_sw_params_t *swparams;
  snd_pcm_hw_params_alloca (&hwparams);
  snd_pcm_sw_params_alloca (&swparams);
  int err;
  if ((err = snd_pcm_open (handle, device, SND_PCM_STREAM_PLAYBACK, 0)) < 0)
    {
    fprintf(stderr, "%s: can't open playback device: %s\n", argv0,
      snd_strerror(err));
    return err;
    }
  if ((err = set_hwparams (*handle, hwparams, argv0)) < 0)
    {
    fprintf(stderr, "%s: can't set hwparams: %s\n", argv0, snd_strerror(err));
    return err;
    }
  if ((err = set_swparams (*handle, swparams, argv0)) < 0)
    {
    fprintf(stderr, "Can't set swparams: %s\n", snd_strerror(err));
    return err;
    }
  return 0;
  }


/*=========================================================================
  show_help
=========================================================================*/
void show_help (const char *argv0)
  {
  printf
    ("Usage: %s [-hv] [--device {ALSA-device}] r2d2-voice {sound string}\n\n",
    argv0);
  printf ("The sound string consists of any number of groups of five\n");
  printf ("arguments, of the following form:\n\n");
  printf ("sound_type length step_length lower_frequency upper_frequency.\n\n");
  printf ("sound_type can be one of the following letters: (g)lissando,\n");
  printf ("(r)andom, or (s)silence. length is the overall duration of the\n");
  printf ("sound in msec; step-length is how long in msec the sound will\n");
  printf ("continue on the same pitch.\n\n");
  printf (
    "Example:\nr2d2-voice g 450 50 400 200 s 500 0 0 0 r 450 50 200 6000\n");
  printf ("Plays a 450msec glissando between 400 and 200Hz, then 500msec\n");
  printf
    ("of silence, then 450msec of random pitches between 200Hz and 600Hz.\n");
  }


/*==========================================================================
  main
==========================================================================*/
int main(int argc, char *argv[])
  {
  struct option long_option[] =
    {
    {"help", 1, NULL, 'h'},
    {"device", 1, NULL, 'D'},
    {NULL, 0, NULL, 0},
    };

  snd_pcm_t *handle;
  char *device = NULL;
  int err;
  while (1)
    {
    int c;
    if ((c = getopt_long (argc, argv, "D:h", long_option, NULL)) < 0)
      break;
    switch (c)
      {
      case 'h':
          show_help (argv[0]);
          return (0);
      case 'D':
        device = strdup (optarg);
          break;
      }
    }

  srand (time (NULL));
  if (!device)
    device = strdup (DEFAULT_DEVICE);

  int args_left = argc - optind;
  if (args_left % 5 != 0)
    {
    fprintf (stderr, "%s: takes a number of groups of five arguments.\n",
      argv[0]);
    fprintf (stderr, "'%s -- help' for usage.\n", argv[0]);
    return (-1);
    }

  err = setup_sound (&handle, device, argv[0]);
  if (err)
    return (-1); // Error message already displayed

  int i;
  for (i = 0; i < args_left / 5; i++)
    {
    const char *arg1 = argv[i * 5 + optind + 0];
    const char *arg2 = argv[i * 5 + optind + 1];
    const char *arg3 = argv[i * 5 + optind + 2];
    const char *arg4 = argv[i * 5 + optind + 3];
    const char *arg5 = argv[i * 5 + optind + 4];
    int length = atoi (arg2);
    int step_length = atoi (arg3);
    int f1 = atoi (arg4);
    int f2 = atoi (arg5);
    SoundType sound_type;
    switch (arg1[0])
      {
      case 'g': case 'G':
        sound_type = sound_type_glissando;
        play_sound (handle, sound_type, length, step_length, f1, f2);
        break;

      case 'r': case 'R':
        sound_type = sound_type_random;
        play_sound (handle, sound_type, length, step_length, f1, f2);
        break;

      case 'b': case 'B':
        sound_type = sound_type_buzz;
        play_sound (handle, sound_type, length, step_length, f1, f2);
        break;

      case 's': case 'S':
        sound_type = sound_type_silence;
        play_sound (handle, sound_type, length, step_length, f1, f2);
        break;

      case 'n': case 'N':
        sound_type = sound_type_noise;
        play_sound (handle, sound_type, length, step_length, f1, f2);
        break;

      default:
        fprintf (stderr, "%s: unknown sound type specifier '%c'.\n",
          argv[0], arg1[0]);
        fprintf (stderr, "'%s -- help' for usage.\n", argv[0]);
        return (-1);
      }
    }


  // Wait for buffer to be empty, or for five seconds at most
  snd_pcm_state_t state;
  int wait_count = 0;
  do
    {
    state = snd_pcm_state (handle);
    usleep (100000);
    wait_count++;
    } while (state != 4 && wait_count < 50);

  snd_pcm_close (handle);

  if (device) free(device);
  return 0;
  }

