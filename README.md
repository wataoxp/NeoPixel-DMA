# NeoPixel-DMA

## Overview
NeoPixel(WS2812B)をDMAで制御するプログラムです。

## Requirement
* STM32CubeIDE
* STM32G030F6P6&G031F6P6
* Clang

## Description
* SPI&DMAが基本となっています。
2025-04-08  
より柔軟なクロックを設定できるPWM出力の設定も追加しました。
PWM_DMAの各ヘッダ、ソースをそのままinc、srcに上書きしてください。  
※i2cおよびso1602ファイルはテスト用のLCD表示プログラムです。  
main.cの232行目あたりの「if(ReturnFlag()) LCD_IR(&MSB, I2C2);」
およびcontrol.cの160行目あたりの「case LCDView:」「case Submit:」の処理にのみ使用しています。  

Edited File
* main.c
* neopixel.h .c
* ll_dma.h .c
* IRremote.h .c
* control.h .c
* gpio.h .c
* i2c.h .c
* s93c46.h .c
* eeprom.h .c
* exti.h
* so1602.h .c
* delay.h .c
* stm32g0xx_it.c

## Licence
[MIT](https://github.com/wataoxp/NeoPixel-DMA/blob/main/LICENSE)
