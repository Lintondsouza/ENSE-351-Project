# STM32F103 Air‑Quality & Environment Monitor

A compact, low‑cost indoor‑air‑quality station that combines:

* **DHT22** – temperature / humidity
* **MQ135** – mixed‑gas VOC sensor (NH₃, NOₓ, smoke, alcohols, benzene …)
* **Piezo buzzer** – audible alarm when gas level exceeds a threshold
* **USART1 @ 9600 bps** – serial output for easy logging / debugging

Everything runs on a bare‑metal **STM32F103C8T6** (“Blue Pill”) at 72 MHz with no RTOS.

---

## Table of Contents
1. [Features](#features)  
2. [Hardware](#hardware)  
3. [Pinout](#pinout)  
4. [Schematic Notes](#schematic-notes)  

---

## Features
| Function | Interval | Output Example |
|----------|----------|----------------|
| DHT22 read | 2 s | `Temp: 24.3 °C, Hum: 38.7 %` |
| MQ135 read | 0.5 s | `MQ135 ADC: 68` |
| Buzzer control | immediate | `Buzzer ON` / `OFF` |

* Threshold is set by `MQ135_THRESHOLD` (default =`70`, tweak to taste).
* Entire code fits comfortably in < 10 kB Flash and < 2 kB RAM.

---

## Hardware
| Qty | Part | Notes |
|----:|------|-------|
| 1 | STM32F103C8 “Blue Pill” | Any 3.3 V STM32F1 will work. |
| 1 | DHT22 / AM2302 | Powered at 3.3 V (keeps logic levels safe). |
| 1 | MQ135 module | *If* you power it from 5 V, add a voltage divider on AO → PA0 so it never exceeds 3.3 V. |
| 1 | Piezo buzzer | Driven from PA6 (TIM3 PWM). Use NPN transistor or MOSFET if > 10 mA. |
| 1 | 10 kΩ resistor | Pull‑up on DHT22 data line |
| many | 0.1 µF caps | One per VDD/VSS pin on the MCU |

You’ll also need an **ST‑Link** or other SWD programmer.

---

## Pinout
| MCU Pin | Direction | Connected To | Purpose |
|---------|-----------|--------------|---------|
| PA0 | A‑IN | MQ135 AO | ADC1_IN0 reading |
| PA4 | I/O | DHT22 DATA | Bidirectional 1‑wire style |
| PA6 | PWM‑OUT | Piezo buzzer | Alarm |
| PA9 | USART1_TX | USB‑TTL / FTDI | Serial 9600 bps |
| PA10| USART1_RX | (optional) | Reserved |
| VDD / VSS | — | 3.3 V / GND | Power with 0.1 µF decoupling |
| BOOT0 | — | Pull‑down 10 kΩ | Boot from Flash |

---

## Schematic Notes
1. **Decoupling** – 0.1 µF close to every VDD/VSS pair.  
2. **NRST** – 10 kΩ to 3.3 V plus 100 nF to GND for clean resets.  
3. **Clock** – Internal HSI works, but add an 8 MHz crystal + 22 pF caps if you need USB later.  
4. **MQ135 level** – Keep AO ≤ 3.3 V. Most breakout boards are 0‑3 V by default when powered at 3.3 V; verify!  
5. **Buzzer driver** – For > 10 mA, use a small NPN + flyback diode (if it’s a coil).

---


