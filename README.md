# F103_BME280
This repository contains an STM32CubeIDE project for a simple weather station using the STM32F103C8T6 development board, BME280 sensor, and an SSD1306 OLED display communicating over I2C.

In addition, a software PWM implementation is used to control the brightness of the LED on pin PC13. The PWM duty cycle is set based on the temperature from the BME280 sensor (temperature = 22 °C, duty cycle = 0/temperature = 25 °C, duty cycle = 100).
