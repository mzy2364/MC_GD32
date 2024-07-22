#ifndef __LCD_INIT_H
#define __LCD_INIT_H

#include "gd32f30x.h"

#define USE_HORIZONTAL 2  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏


#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 135
#define LCD_H 240

#else
#define LCD_W 240
#define LCD_H 135
#endif



//-----------------LCD端口定义---------------- 

#define LCD_RES_Clr()  gpio_bit_write(GPIOC,GPIO_PIN_15,RESET)
#define LCD_RES_Set()  gpio_bit_write(GPIOC,GPIO_PIN_15,SET)

#define LCD_DC_Clr()   gpio_bit_write(GPIOC,GPIO_PIN_14,RESET)
#define LCD_DC_Set()   gpio_bit_write(GPIOC,GPIO_PIN_14,SET)
 		     
#define LCD_CS_Clr()   gpio_bit_write(GPIOC,GPIO_PIN_1,RESET)
#define LCD_CS_Set()   gpio_bit_write(GPIOC,GPIO_PIN_1,SET)

#define LCD_BLK_Clr()  gpio_bit_write(GPIOC,GPIO_PIN_13,RESET)
#define LCD_BLK_Set()  gpio_bit_write(GPIOC,GPIO_PIN_13,SET)



void LCD_GPIO_Init(void);//初始化GPIO
void LCD_Writ_Bus(uint8_t dat);//模拟SPI时序
void LCD_WR_DATA8(uint8_t dat);//写入一个字节
void LCD_WR_DATA(uint16_t dat);//写入两个字节
void LCD_WR_REG(uint8_t dat);//写入一个指令
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);//设置坐标函数
void LCD_Init(void);//LCD初始化
#endif




