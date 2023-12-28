/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Sample app to demonstrate PWM-based servomotor control
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/shell/shell.h>
#include "buzzer.h"

static const struct pwm_dt_spec sBuzzer = PWM_DT_SPEC_GET(DT_NODELABEL(buzzer));

#define sixteenth 38
#define eigth 75
#define quarter 150
#define half 300
#define whole 600

#define C4  262
#define Db4 277
#define D4  294
#define Eb4 311
#define E4  330
#define F4  349
#define Gb4 370
#define G4  392
#define Ab4 415
#define A4  440
#define Bb4 466
#define B4  494
#define C5  523
#define Db5 554
#define D5  587
#define Eb5 622
#define E5  659
#define F5  698
#define Gb5 740
#define G5  784
#define Ab5 831
#define A5  880
#define Bb5 932
#define B5  988
#define C6  1046
#define Db6 1109
#define D6  1175
#define Eb6 1245
#define E6  1319
#define F6  1397
#define Gb6 1480
#define G6  1568
#define Ab6 1661
#define A6  1760
#define Bb6 1865
#define B6  1976

#define REST 1
#define END 0

#define MAX_LEN 1000

struct note_duration
{
	int note;	  // hz
	int duration; // msec
};

static const struct note_duration mario_song[] = {
	{.note = E6, .duration = quarter},
	{.note = REST, .duration = eigth},
	{.note = E6, .duration = quarter},
	{.note = REST, .duration = quarter},
	{.note = E6, .duration = quarter},
	{.note = REST, .duration = quarter},
	{.note = C6, .duration = quarter},
	{.note = E6, .duration = half},
	{.note = G6, .duration = half},
	{.note = REST, .duration = quarter},
	{.note = G4, .duration = whole},
	{.note = REST, .duration = whole},
	// break in sound
	{.note = C6, .duration = half},
	{.note = REST, .duration = quarter},
	{.note = G5, .duration = half},
	{.note = REST, .duration = quarter},
	{.note = E5, .duration = half},
	{.note = REST, .duration = quarter},
	{.note = A5, .duration = quarter},
	{.note = REST, .duration = quarter},
	{.note = B5, .duration = quarter},
	{.note = REST, .duration = quarter},
	{.note = Bb5, .duration = quarter},
	{.note = A5, .duration = half},
	{.note = G5, .duration = quarter},
	{.note = E6, .duration = quarter},
	{.note = G6, .duration = quarter},
	{.note = A6, .duration = half},
	{.note = F6, .duration = quarter},
	{.note = G6, .duration = quarter},
	{.note = REST, .duration = quarter},
	{.note = E6, .duration = quarter},
	{.note = REST, .duration = quarter},
	{.note = C6, .duration = quarter},
	{.note = D6, .duration = quarter},
	{.note = B5, .duration = quarter},
	{.note = END}
};

static const struct note_duration funkytown_song[] = {
	{.note = C5, .duration = quarter},
	{.note = REST, .duration = eigth},
	{.note = C5, .duration = quarter},
	{.note = Bb4, .duration = quarter},
	{.note = C5, .duration = quarter},
	{.note = REST, .duration = quarter},
	{.note = G4, .duration = quarter},
	{.note = REST, .duration = quarter},
	{.note = G4, .duration = quarter},
	{.note = C5, .duration = quarter},
	{.note = F5, .duration = quarter},
	{.note = E5, .duration = quarter},
	{.note = C5, .duration = quarter},
	{.note = END}
};

static const struct note_duration *songs[] = {
	mario_song,
	funkytown_song,
};


static int buzzer_play_song(const struct note_duration *song, uint8_t volume) {
	pwm_set_pulse_dt(&sBuzzer, 0);
	for (int i = 0; i < MAX_LEN; i++)
	{
		if (song[i].note == END)
		{
			break;
		} else if (song[i].note < 10)
		{
			// Low frequency notes represent a 'pause'
			pwm_set_pulse_dt(&sBuzzer, 0);
			k_msleep(song[i].duration);
		}
		else
		{
			pwm_set_dt(&sBuzzer, PWM_HZ(song[i].note), PWM_HZ((song[i].note)) * volume / 200);
			k_msleep(song[i].duration);
		}
	}
	pwm_set_pulse_dt(&sBuzzer, 0);
	return 0;
}

int buzzer_play(enum buzzer_sound id, uint8_t volume) {
	if (id > BUZZER_LAST || volume > 100)
		return -EINVAL;
	return buzzer_play_song(songs[id], volume);
}

static int cmd_buzzer_play(const struct shell *sh, size_t argc,
			      char **argv)
{
	uint32_t id, vol;
	if (argc < 3 || sscanf(argv[1],"%u", &id) != 1 
		|| sscanf(argv[2],"%u", &vol) != 1) {
		shell_help(sh);
		return -EINVAL;
	}
	return buzzer_play(id, vol);
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_buzzer,
	SHELL_CMD(play, NULL, "id volume", cmd_buzzer_play),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(buzzer, &sub_buzzer, "rc commands", NULL);
