# NRF52-WS2812B-LEDController-EVB
EVB code for LED controller based on NRF52 and WS2812B led strip

## Prerequisites
* Nordic Semiconductor [nRF5 SDK v11.0.0](http://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v11.x.x/)
* Nordic Semiconductor [nRF5x Command Line Tools](http://www.nordicsemi.com/eng/Products/Bluetooth-Smart-Bluetooth-low-energy/nRF52-DK#Downloads)

## Features
*LED_EVB is an integrated Nordic nRF52832 32-bit ARM® Cortex™-M4F CPU with 512kB + 64kB RAM development board.

*The embedded 2.4GHz transceiver supports Bluetooth low energy and proprietary 2.4 GHz protocol stack.

*Segger J-Link OB Program/Debug supported

*Breakout UART interface

*The EVB can operate on a power supply between 4.7v and 5.2v.

## Specification
Size: 3.8 (Length) x 2.1 (width) x 1 (height) cm
Weight: g

## System Diagram

<img alt="Crescendo sequence" src="/pic/System Diagram.jpg" width="1000"/>

## Schematic

<img alt="Crescendo sequence" src="/pic/Schematic.jpg" width="1000"/>

## Pin Define

PIN Name  Pin Function  Description
P0.02     NRF_TX        Universal Asynchronous Receiver Transmitter (UART) _ TX Pin
P0.03     NRF_RX        Universal Asynchronous Receiver Transmitter (UART) _ RX Pin
P0.11     WS1           WS2812B LED Control data signal output
P0.25     LED           Onboard LED signal 

## PCB_Layout

<img alt="Crescendo sequence" src="/pic/PCB_Layout.jpg" width="1000"/>

## Equipment

<img alt="Crescendo sequence" src="/pic/Equipment.jpg" width="1000"/>


## Part List
Part List: LEDC_EVB x1

## Demo video

[![LED](http://img.youtube.com/vi/OZ7Efc-QP9c/0.jpg)](https://www.youtube.com/watch?v=OZ7Efc-QP9c "LED")

[![LED](http://img.youtube.com/vi/pRjLmnnzIZ0/0.jpg)](https://www.youtube.com/watch?v=pRjLmnnzIZ0 "LED")

## SPI part from 
[https://github.com/takafuminaka/nRF52832]

## Android Sample code
[https://github.com/MaBaoGW/LED-Android-Sample]


