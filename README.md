# STM32L4-basic-functionality

## In this repository I present assignments from the laboratory classes of the Microprocessor Technology course in the Automation and Robotics course, at the AGH University of Science and Technology. And a number of assignments from a public course on microcontrollers available at this [link](https://forbot.pl/blog/kurs-stm32-l4-wstep-spis-tresci-dla-kogo-jest-ten-kurs-id48575).

I have learned the basics of programming STM microcontrollers on two NUCLEO boards with processors having Cortex-M4 cores. 
One is NUCLEO-L476RG ("**L4_**..." files) with KA-Nucleo-Multisensor expander. And the other is NUCLEO-F411RE ("**F4_**..." files). In each folder are the corresponding code and ioc files. Use STM32CubeMX version 6.9.2 and ioc files to generate the configuration used in the given issue.  

**The individual tasks consisted of:**

---
#### **STM32F411RE**
- [F4_Led](https://github.com/patryklatka/STM32L4-basic-functionality/tree/main/F4_Led) Creating a "snake" from four leds placed on the expander. 

- [F4_EXTI](https://github.com/patryklatka/STM32L4-basic-functionality/tree/main/F4_EXTI) Using two buttons and external interrupts to increment and decrement the counter, the current value of which is displayed. 

- [F4_Display_7_seg](https://github.com/patryklatka/STM32L4-basic-functionality/tree/main/F4_Display_7_seg) Program the seven-segment display to display four different digits using interrupts.

- [F4_ADC](https://github.com/patryklatka/STM32L4-basic-functionality/tree/main/F4_ADC) Using a photoresistor and ADC to test the brightness of a room, and displaying the result on a display.
---
#### **STM32L476RG**
- [L4_UART](https://github.com/patryklatka/STM32L4-basic-functionality/tree/main/L4_UART) Learning about the UART communication protocol. Applying it to transfer data to a terminal on a PC. Any terminal can be used, but I used Tera Term. 

- [L4_Timers](https://github.com/patryklatka/STM32L4-basic-functionality/tree/main/L4_Timers) Flashing the RGB LED using counters to generate a PWM signal, and increasing or decreasing the number displayed on the terminal depending on the direction of rotation of the encoder.

- [L4_SPI](https://github.com/patryklatka/STM32L4-basic-functionality/tree/main/L4_SPI) Using SPI to communicate with a port expander (MCP23S08), as well as with a TFT display equipped with an ST7735S controller. In the task with the TFT, for the purpose of checking the correctness of the SPI configuration, I used an older version of the ready-made - [library](https://github.com/tuupola/hagl?tab=MIT-1-ov-file), which I included in the files. 



- [L4_Pressure_sensor_I2C](https://github.com/patryklatka/STM32L4-basic-functionality/tree/main/L4_Pressure_sensor_I2C) Relative altitude was measured using the LPS25HB pressure and altitude sensor, with which communication was via I2C.

- [L4_Interrupts](https://github.com/patryklatka/STM32L4-basic-functionality/tree/main/L4_Interrupts) Simple application of interrupts.

- [L4_Distnace_sensor](https://github.com/patryklatka/STM32L4-basic-functionality/tree/main/L4_Distance_sensor) Programming a simple rangefinder using the HC-SR04 ultrasonic sensor, with air temperature affecting the speed of sound using the LM35 analog temperature sensor. 

- [L4_ADC](https://github.com/patryklatka/STM32L4-basic-functionality/tree/main/L4_ADC) Basics of ADC operation - voltage measurement from a single input.

- [L4_1Wire_UART](https://github.com/patryklatka/STM32L4-basic-functionality/tree/main/L4_1Wire_UART) Using UART to communicate 1Wire with DS18B20 temperature sensor .

