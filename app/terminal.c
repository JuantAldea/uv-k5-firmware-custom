/* Copyright 2023 Dual Tachyon
 * https://github.com/DualTachyon
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 */

#include "terminal.h"
#include "audio.h"
#include "font.h"
#include "misc.h"
#include "ui/helper.h"
#include "driver/keyboard.h"
#include "driver/st7565.h"
#include "ui/ui.h"
#include <limits.h>
#include <string.h>
#include "../external/printf/printf.h"

static uint32_t cursor = 0;
static uint32_t line = 0;
static bool lock_adquired = false;

bool TERMINAL_Init()
{
	/*
    if (gScreenLock) {
        return false;
    }
    gScreenLock = true;
    */

	lock_adquired = true;

	cursor = 0;
	line = 0;

	UI_DisplayClear();
	TERMINAL_UpdateScreen();
	return true;
}

void TERMINAL_Exit()
{
	if (!lock_adquired) {
		return;
	}

	lock_adquired = false;
	//gScreenLock = false;
}

void TERMINAL_UpdateScreen()
{
	ST7565_BlitDisplay();
}

static uint8_t *TERMINAL_GetLineBuffer(uint8_t line)
{
	if (line == 0) {
		return gStatusLine;
	}

	return gFrameBuffer[line - 1];
}

static void TERMINAL_BlitCurrentLine()
{
	if (!lock_adquired) {
		return;
	}

	if (line == 0) {
		ST7565_BlitStatusLine();
		return;
	}

	ST7565_BlitLine(line - 1);
}

static void TERMINAL_ScrollScreen_UP()
{
	for (uint8_t line = 1; line < FRAME_LINES + 1; line++) {
		uint8_t *const dst_buffer = TERMINAL_GetLineBuffer(line - 1);
		memcpy(dst_buffer, gFrameBuffer[line - 1], sizeof(gFrameBuffer[0]));
	}

	memset(gFrameBuffer[FRAME_LINES - 1], 0, sizeof(gFrameBuffer[0]));
	ST7565_BlitDisplay();
}

static void TERMINAL_AdvanceLine()
{
	cursor = 0;
	if (++line >= FRAME_LINES + 1) {
		line = FRAME_LINES;
		TERMINAL_ScrollScreen_UP();
	}
}

static void TERMINAL_AdvanceCursor()
{
	uint8_t cursor_next = cursor + ARRAY_SIZE(gFontSmall[0]) + 1;
	if (cursor_next >= LCD_WIDTH) {
		TERMINAL_AdvanceLine();
		return;
	}

	cursor = cursor_next;
}

void _TERMINAL_putchar(char c, __attribute__((unused)) void *arg)
{
	if (!lock_adquired) {
		return;
	}

	if (c == '\n' || c == '\r') {
		TERMINAL_AdvanceLine();
		return;
	}

	char ptr[1] = { c };

	const uint8_t cursor_next = cursor + ARRAY_SIZE(gFontSmall[0]) + 1;

	if (cursor_next >= LCD_WIDTH) {
		TERMINAL_AdvanceLine();
	}

	UI_PrintStringBuffer(ptr, TERMINAL_GetLineBuffer(line) + cursor, ARRAY_SIZE(gFontSmall[0]),
			     (const uint8_t *)gFontSmall);
	TERMINAL_BlitCurrentLine();
	TERMINAL_AdvanceCursor();
}

void TERMINAL_putchar(char c)
{
	_TERMINAL_putchar(c, NULL);
}

static void TERMINAL_Key_EXIT(__attribute__((unused)) bool bKeyPressed, __attribute__((unused)) bool bKeyHeld)
{
	TERMINAL_Exit();
}

static void TERMINAL_Key_MENU(__attribute__((unused)) bool bKeyPressed, __attribute__((unused)) bool bKeyHeld)
{
	TERMINAL_Init();
}

void TERMINAL_ProcessKeys(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld)
{
	if (bKeyHeld || !bKeyPressed) {
		return;
	}

	switch (Key) {
	case KEY_MENU:
		TERMINAL_Key_MENU(bKeyPressed, bKeyHeld);
		break;
	case KEY_EXIT:
		TERMINAL_Key_EXIT(bKeyPressed, bKeyHeld);
		break;
	case KEY_0 ... KEY_9:
	default:
		break;
	}
}

void TERMINAL_Demo()
{
	TERMINAL_Init();

	fctprintf(_TERMINAL_putchar, NULL, "%s\n", "En un lugar de la Mancha,");
	fctprintf(_TERMINAL_putchar, NULL, "%s\n", "de cuyo nombre, no quiero acordarme");
	fctprintf(_TERMINAL_putchar, NULL, "%s\n",
		  "no ha mucho tiempo que vivia un hidalgo de los de lanza en astillero, ");
	fctprintf(_TERMINAL_putchar, NULL, "%s", "adarga antigua ");
	fctprintf(_TERMINAL_putchar, NULL, "%s", "y galgo corredor");

	TRAP();
}