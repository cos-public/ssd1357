#ifndef INC_SSD1357_H_
#define INC_SSD1357_H_

#include <stdint.h>
#include <stdbool.h>

// Configuration
#define SSD1357_DC_GPIO_Port	GPIOA
#define SSD1357_DC_LL_Pin	LL_GPIO_PIN_6
#define SSD1357_NSS_GPIO_Port	GPIOA
#define SSD1357_NSS_LL_Pin	LL_GPIO_PIN_4
#define SSD1357_OLED_64x64

#ifdef SSD1357_OLED_64x64
#define SSD1357_START_ROW	0x00
#define SSD1357_START_COL	0x20
#define SSD1357_STOP_ROW 	0x3F //START_ROW + 64 - 1
#define SSD1357_STOP_COL	0x5F //START_COL + 64 - 1
#else // assume 128x128
// Not tested
#define SSD1357_START_ROW	0x00
#define SSD1357_START_COL	0x00
#define SSD1357_STOP_ROW 	0x7F
#define SSD1357_STOP_COL	0x7F
#endif

void SSD1357_SetCommandLock(bool locked);
void SSD1357_SetSleepMode(bool on);
void SSD1357_SetClockDivider(uint8_t divider_code);
void SSD1357_SetMuxRatio(uint8_t ratio);
void SSD1357_SetDisplayOffset(uint8_t offset);
void SSD1357_SetDisplayStartLine(uint8_t line);
#define SSD1357_COLOR_MODE_256 0x00
#define SSD1357_COLOR_MODE_65k 0x01
void SSD1357_SetRemapColorDepth(bool inc_Vh, bool rev_ColAddr, bool swap_ColOrder, bool rev_SCAN, bool en_SplitOddEven, uint8_t color_depth_code);
void SSD1357_SetContrastCurrentABC(uint8_t ccA, uint8_t ccB, uint8_t ccC);
void SSD1357_SetMasterContrastCurrent(uint8_t ccCode);
void SSD1357_SetResetPrechargePeriod(uint8_t reset_clocks, uint8_t precharge_clocks);
void SSD1357_SetPrechargeVoltage(uint8_t voltage_scale_code);
void SSD1357_SetMLUTGeryscale(uint8_t * pdata63B);
void SSD1357_UseDefaultLUT();
void SSD1357_setVCOMH(uint8_t voltage_scale_code);
void SSD1357_SetColumnAddress(uint8_t start, uint8_t stop);
void SSD1357_SetRowAddress(uint8_t start, uint8_t stop);
void SSD1357_SetDisplayMode_Reset();

void SSD1357_DefaultInit();

/// convert 0-255 RGB values to RGB565
#define RGB565(R,G,B) ((((B * 0x1F) / 0xFF) << 11) | (((G * 0x3F) / 0xFF) << 5) | (((R * 0x1F) / 0xFF) << 0))

void SSD1357_FillDisplay(uint16_t value);
void SSD1357_FillRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint16_t value);
void SSD1357_Image(uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t * pixels);

#endif /* INC_SSD1357_H_ */
