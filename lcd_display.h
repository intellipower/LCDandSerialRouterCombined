#ifndef LCD_DISPLAY_H_
#define LCD_DISPLAY_H_

#include "main.h"

#define FB_CLEAR BIT7		// Send to fakeButton() to clear fakeButtonState

void lcdOperatingScreen(volatile struct upsDataStrucT *upsData);
void lcdOptionScreen(volatile struct upsDataStrucT *upsData);
void update_LCD_bar(float level, int row);
void update_LCD_bat_levels(float level);
void update_LCD_load_levels(float level);
int fakeButton(int whichButton);
void lcdManager(volatile struct upsDataStrucT *upsData);

#endif
