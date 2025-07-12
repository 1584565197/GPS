#include "lcd_i2c.h"
#include <string.h>
#include "main.h"

static I2C_HandleTypeDef *hi2c_dev;
static uint8_t backlight_state = 1;  // ??????

static void LCD_SendNibble(uint8_t data, uint8_t rs) {
    uint8_t buf = 0;
    
    // ????? (?4?)
    if (data & 0x10) buf |= (1 << D4_PIN);
    if (data & 0x20) buf |= (1 << D5_PIN);
    if (data & 0x40) buf |= (1 << D6_PIN);
    if (data & 0x80) buf |= (1 << D7_PIN);
    
    // ??RS?
    if (rs) buf |= (1 << RS_PIN);
    
    // ????
    buf |= (backlight_state << BL_PIN);
    
    // ???? (EN?->?)
    buf |= (1 << EN_PIN);  // EN?
    HAL_I2C_Master_Transmit(hi2c_dev, LCD_ADDR, &buf, 1, 100);
    
    buf &= ~(1 << EN_PIN); // EN?
    HAL_I2C_Master_Transmit(hi2c_dev, LCD_ADDR, &buf, 1, 100);
    
    // ????
    for (volatile uint32_t i = 0; i < 100; i++);
}

void LCD_SendCommand(uint8_t cmd) {
    // ???4?
    LCD_SendNibble(cmd, 0);
    // ???4?
    LCD_SendNibble(cmd << 4, 0);
    
    // ?????????????
    if (cmd == 0x01 || cmd == 0x02) {
        HAL_Delay(2);
    } else {
        // ???????
        HAL_Delay(1);
    }
}

void LCD_Init(I2C_HandleTypeDef *hi2c) {
    hi2c_dev = hi2c;
    
    // LCD?????????
    HAL_Delay(50);
    
    // ???????
    for (uint8_t i = 0; i < 3; i++) {
        LCD_SendNibble(0x30, 0);  // 8???
        HAL_Delay(5);
    }
    
    // ???4???
    LCD_SendNibble(0x20, 0);
    HAL_Delay(1);
    
    // ???????
    LCD_SendCommand(0x28);  // 4???,2?,5x8??
    LCD_SendCommand(0x0C);  // ???,???
    LCD_SendCommand(0x06);  // ????,???
    LCD_Clear();            // ??
}

void LCD_WriteChar(char ch) {
    // ???4?
    LCD_SendNibble((uint8_t)ch, 1);
    // ???4?
    LCD_SendNibble((uint8_t)ch << 4, 1);
    HAL_Delay(1);  // ???????
}

void LCD_WriteString(char *str) {
    while (*str) {
        LCD_WriteChar(*str++);
    }
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};  // LCD2004???
    LCD_SendCommand(0x80 | (col + row_offsets[row]));
}

void LCD_Clear(void) {
    LCD_SendCommand(0x01);
    HAL_Delay(2);  // ????????
}

void LCD_Backlight(uint8_t state) {
    backlight_state = state;
    // ?????????????
    LCD_SendCommand(0x00);
}

void LCD_CreateCustomChar(uint8_t char_num, uint8_t *data) {
    LCD_SendCommand(0x40 | (char_num << 3));  // ??CGRAM??
    
    for (uint8_t i = 0; i < 8; i++) {
        LCD_WriteChar(data[i]);  // ??????
    }
}
