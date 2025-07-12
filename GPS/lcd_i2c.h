#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "stm32f1xx_hal.h"

// ??I2C?????? (0x27 ? 0x3F)
#define LCD_ADDR     (0x27 << 1)  // 7?????1?

// ???? (??????)
#define RS_PIN   0
#define EN_PIN   2
#define D4_PIN   4
#define D5_PIN   5
#define D6_PIN   6
#define D7_PIN   7
#define BL_PIN   3   // ????

void LCD_Init(I2C_HandleTypeDef *hi2c);
void LCD_SendCommand(uint8_t cmd);
void LCD_WriteChar(char ch);
void LCD_WriteString(char *str);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Clear(void);
void LCD_Backlight(uint8_t state);
void LCD_CreateCustomChar(uint8_t char_num, uint8_t *data);

#endif