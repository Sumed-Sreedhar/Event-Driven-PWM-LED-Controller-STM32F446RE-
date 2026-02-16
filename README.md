# Multi-Mode PWM LED Controller (STM32F446RE)

**Status:** Completed and hardware-tested  
**Platform:** STM32F446RE (Nucleo)  
**Architecture:** Interrupt-driven + Time-based state machine  

---

## What It Does

A multi-mode LED brightness control system using:

- **Hardware PWM (TIM3)** for deterministic brightness control
- **Timer-based system tick (TIM2)** for time-driven logic
- **EXTI interrupts** for mode transitions and fault control
- **UART interrupt-driven CLI**
- **4x3 matrix keypad input**
- **Hierarchical state machine architecture**

The system manages LED brightness across multiple operational modes while enforcing structured state transitions and fault handling.

---

## System Architecture

### Core Peripherals
- **TIM3 (PWM Mode 1)** → LED brightness control (CCR-based duty cycle)
- **TIM2 (Base Timer Interrupt)** → 1ms system tick
- **USART2 (Interrupt mode)** → CLI input parsing
- **GPIO EXTI** → Mode button, Fault trigger, Fault clear
- **GPIO Matrix Scanning** → 4x3 keypad

---

## State Machine Design

### Primary System States

```
INIT → IDLE → MANUAL → LOCK → AUTO_RAMP → FAULT
```

### State Behavior Overview

| State       | Behavior |
|------------|----------|
| INIT       | Startup indicator phase |
| IDLE       | LED OFF, status LED slow blink |
| MANUAL     | Brightness controlled via UART or keypad |
| LOCK       | Brightness locked, status double-blink |
| AUTO_RAMP  | Automatic smooth ramp up/down (PWM duty cycle sweep) |
| FAULT      | Forced shutdown, fault LED active |

---

## Mode Control Logic

### Event-Driven Transitions
- **MODE button (EXTI)** cycles through operational modes
- **Force Fault button (EXTI)** enters FAULT state
- **Clear Fault button (EXTI)** exits FAULT (if valid)

All EXTI logic is debounced using timestamp comparison against `system_tick`.

---

## PWM Implementation

- TIM3 configured in **PWM Mode 1**
- `ARR = 999` → 1000-step resolution
- Brightness mapped directly to `CCR1`
- Duty cycle updated via:

```
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, brightness);
```

### AUTO_RAMP Mode
- Time-driven ramp logic (10 ms step interval)
- Smooth increase/decrease between 0–999
- No blocking delays
- Fully timer-driven

---

## UART CLI (Interrupt Driven)

- Byte-by-byte reception using `HAL_UART_Receive_IT`
- Non-blocking echo handling
- Backspace support
- Command buffer with termination detection (`\n` / `\r`)
- CLI command converts numeric string to brightness value (0–999)

This runs without polling and without blocking the main loop.

---

## Keypad Interface

- 4x3 matrix scanning
- Row-driven output, column read with pull-ups
- 20ms scan interval
- Numeric input accumulates brightness value
- `#` commits value
- `*` clears input

---

## Status LED Patterns

| State      | Pattern |
|-----------|----------|
| IDLE      | 500ms toggle |
| MANUAL    | Solid ON |
| LOCK      | Double blink |
| AUTO_RAMP | Fast blink (200ms) |
| FAULT     | Solid OFF |

Implemented using time comparisons against `system_tick`.

---

## Fault Handling Layer

- FAULT overrides all modes
- PWM duty cycle forced to 0
- Fault LED activated
- System remains in FAULT until cleared
- All transitions gated through state logic

---

## Design Principles Applied

- Strict separation of:
  - Event detection (EXTI)
  - Time base (TIM2)
  - Hardware control (PWM via TIM3)
- No blocking delays
- Deterministic timer-based logic
- Debounced interrupt inputs
- Peripheral ownership discipline
- State entry timestamps for clean transitions

---

## What This Project Demonstrates

- Hardware PWM configuration and runtime control
- Multi-peripheral coordination
- Interrupt-driven CLI handling
- Matrix keypad scanning logic
- Time-based ramp algorithms
- Hierarchical state machine structuring
- Fault-safe system design
- Clean ISR responsibility boundaries

---

## Tools & Environment

- STM32CubeIDE
- STM32 HAL
- Embedded C
- Linux development environment
- Git & GitHub

---

## Repository Structure

```
Multi_Mode_PWM_Controller/
├── Core/
│   ├── Src/
│   │   └── main.c
│   └── Inc/
├── Drivers/
├── Multi_Mode_PWM_Controller.ioc
├── STM32F446RETX_FLASH.ld
├── STM32F446RETX_RAM.ld
└── README.md
```

---

## Status

Fully functional on hardware.  
Validated across all operational modes, including fault transitions and ramp behavior.
