# 3-DOF Robotic Manipulator

Bare-metal firmware for a 3-DOF robotic manipulator built on a STM32F446 microcontroller running FreeRTOS. The system features joint-space control of the end-effector and implements real-time inverse kinematics, trapezoidal velocity motor profiles, and multi-joint stepper motor synchronization. 

## Hardware

| Component | Details |
|---|---|
| MCU | STM32F446RE (ARM Cortex-M4F, 180 MHz, hardware FPU) |
| Motor Drivers | TMC2209 (x3) driving stepper motors at 3200 microsteps/rev |
| Communication | USART2 @ 115200 baud for status monitoring |
| Safety | On-board button (PC13) for emergency stop via external interrupt |
| Programmer | ST-Link via OpenOCD |

## Project Structure

```
Core/
  Inc/           Headers and type definitions
  Src/           Application logic (main, robot, motion, stepper)
Drivers/
  Peripherals/   Low-level peripheral drivers (GPIO, UART, Timer, EXTI)
FreeRTOS/        FreeRTOS kernel, Cortex-M4F port, and heap allocator
CMSIS/           ARM CMSIS headers for STM32F446xx
Startup/         Startup assembly and linker script
```