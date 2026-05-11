/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _buzzer_h_
#define _buzzer_h_

enum buzzer_sound {
	BUZZER_MARIO,
	BUZZER_FUNKYTOWN,
	BUZZER_TETRIS,
	BUZZER_LAST = BUZZER_TETRIS
};

int buzzer_init();
int buzzer_play(enum buzzer_sound id, uint8_t volume);
void buzzer_stop(void);

#endif
