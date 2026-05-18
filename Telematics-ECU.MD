# Design Document — Telematics ECU

**Platform:** STM32F407VG Discovery Board · ESP32 · ThingsBoard Cloud · CAN Bus · GSM

---

## Table of Contents

- [1. Project Overview](#1-project-overview)
- [2. Hardware Platform — STM32F407VG Discovery Board](#2-hardware-platform--stm32f407vg-discovery-board)
- [3. Module and Component Selection](#3-module-and-component-selection)
- [4. CAN Bus Communication Design](#4-can-bus-communication-design)
- [5. ThingsBoard Cloud Platform Integration](#5-thingsboard-cloud-platform-integration)
- [6. Software Architecture (FreeRTOS)](#6-software-architecture-freertos)
- [7. Complete Data Flow — Scenarios](#7-complete-data-flow--scenarios)
- [8. Multi-ECU Integration Notes](#8-multi-ecu-integration-notes)

---

## 1. Project Overview

### 1.1 Project Summary

This project implements a simulated multi-ECU vehicle system on a network of STM32 microcontrollers. Eight ECUs are integrated into a shared CAN bus communication architecture to replicate the behaviour of a real vehicle's electronic backbone. Each team member is responsible for one ECU sub-system. This document covers the **Telematics ECU** implemented on the STM32F407VG Discovery board paired with an ESP32-WROOM-32 Wi-Fi module.

The Telematics ECU is the connectivity hub of the vehicle. It reads data from all other ECUs over the CAN bus, processes that data, and sends it to the cloud (ThingsBoard) via the ESP32. It also receives remote commands from the cloud and pushes them back into the CAN bus — enabling features like remote immobilization, over-the-air diagnostics, and real-time crash alerts.

### 1.2 System-Level ECU Map

The following eight ECUs form the complete vehicle simulation network. All communicate over a shared CAN bus at 500 kbps.

| # | ECU Name | Responsible For | Platform | CAN Node ID |
|---|----------|-----------------|----------|-------------|
| 1 | Engine Management ECU | Engine RPM simulation, temperature | STM32F407VG | `0x103` |
| 2 | Transmission ECU | Gear selection, torque converter, shift logic | STM32F407VG | `0x104` |
| 3 | ABS ECU | Wheel speed, slip ratio, brake modulation, distance, temperature | STM32F407VG | `0x102` |
| 4 | Body Control Module (BCM) | Lights, wipers, doors, central locking | STM32F407VG | `0x106` |
| 5 | Infotainment ECU | Display of speed, RPM, ABS status, temperature | STM32F407VG | `0x108` |
| 6 | Battery Management System (BMS) | Battery voltage, SoC, charge control | STM32F407VG | `0x105` |
| 7 | **Telematics ECU (This Document)** | GPS, over-the-air data, remote monitoring, immobilizer, crash alerts | STM32F407VG + ESP32 | `0x107` |
| 8 | Airbag ECU | Crash detection, airbag deployment trigger, seatbelt pretensioner | STM32F407VG | `0x100` |

#### Figure 1 — System-Level ECU Network Block Diagram

```
                        CAN BUS — 500 kbps (CANH / CANL differential pair)
  ┌────────┐  ┌──────────────┐  ┌────────┐   ┌────────┐   ┌────────┐   ┌────────┐   ┌────────┐   ┌────────┐
  │ Engine │  │Transmission  │  │  ABS   │   │  BCM   │   │Infotain│   │  BMS   │   │TELEMA- │   │ Airbag │
  │  ECU   │  │    ECU       │  │  ECU   │   │  ECU   │   │  ECU   │   │  ECU   │   │ TICS   │   │  ECU   │
  │ 0x103  │  │   0x104      │  │ 0x102  │   │ 0x106  │   │ 0x108  │   │ 0x105  │   │ 0x107  │   │ 0x100  │
  └───┬────┘  └──────┬───────┘  └───┬────┘   └───┬────┘   └───┬────┘   └───┬────┘   └───┬────┘   └───┬────┘
      │              │              │            │            │            │            │            │
══════╪══════════════╪══════════════╪════════════╪════════════╪════════════╪════════════╪════════════╪══════
[120Ω]                                    CAN Bus Backbone                                          [120Ω]

  Telematics ECU (0x700) is the gateway node → forwards all data to ThingsBoard Cloud via ESP32
```

### 1.3 Telematics ECU Scope

The Telematics ECU is responsible for the following within the integrated system:

- Reading CAN frames from all other ECUs (Engine, ABS, Airbag, BMS, Infotainment) at 500 kbps
- Forwarding vehicle telemetry to ThingsBoard Cloud via ESP32 (MQTT on port 1883 / TLS 8883)
- Detecting crash events from Airbag ECU (CAN ID `0x100`) and triggering GSM eCall + cloud alert
- Executing remote immobilization commands received from the cloud via ESP32 UART
- Sending immobilization CAN frame (`0x105`) to Engine ECU with rolling authentication token
- Logging all Diagnostic Trouble Codes (DTCs) received from any ECU to W25Q64 SPI flash memory
- Monitoring battery voltage from BMS ECU (`0x105`) and reporting to cloud dashboard
- Transmitting JSON-formatted telemetry packets to ThingsBoard Cloud every 5 seconds
- Managing an Independent Watchdog (IWDG) for system reliability — reset if firmware hangs
- Updating ThingsBoard Shared Device Attributes to maintain last-known vehicle state even when offline

---

## 2. Hardware Platform — STM32F407VG Discovery Board

#### Figure 2 — Telematics ECU Hardware Block Diagram

```
                    ┌─────────────────────────────────────────────┐
                    │         STM32F407VG Discovery Board         │
                    │     ARM Cortex-M4 @ 168 MHz | 1 MB Flash    │
                    │                                             │
  ┌─────────────┐   │  ┌─────────────┐    ┌──────────────────┐    │   ┌──────────────────┐
  │ SN65HVD230  │   │  │ bxCAN1      │    │ USART1 PA9/PA10  │    │   │  ESP32-WROOM-32  │
  │CAN Transceiv│◄──┤  │ PD0/PD1     │    │ 115200 baud      ├─── ┼──►│  Wi-Fi 802.11    │
  │ CANH / CANL │   │  └─────────────┘    └──────────────────┘    │   │  MQTT Client     │
  └──────┬──────┘   │                                             │   │  TLS 1.2         │
         │          │  ┌─────────────┐    ┌──────────────────┐    │   └────────┬─────────┘
  ═══════╪═══════   │  │ USART2      │    │ SPI1 PA5/PA6/PA7 │    │            │ MQTT
  CAN Bus│500kbps   │  │ PA2/PA3     │    │ W25Q64 Flash     │    │            ▼
         │          │  │ 9600 baud   │    │ DTC + Crash Logs │    │   ┌──────────────────┐
                    │  └──────┬──────┘    └──────────────────┘    │   │  ThingsBoard ☁   │
                    │         │                                   │   │  Cloud Platform  │
                    │  ┌──────┴──────────────────────────────┐    │   └──────────────────┘
                    │  │  FreeRTOS Tasks                     │    │
                    │  │  Watchdog(P5) | CAN_Rx(P4)          │    │   ┌──────────────────┐
                    │  │  CrashMgr(P3) | GSM(P3)             │    │   │  SIM800C GSM     │
                    │  │  FaultMgr(P2) | CloudTx(P1)         │    │   │  Emergency eCall │
                    │  └─────────────────────────────────────┘    │   │  3.7–4.2V supply │
                    │                                             │   └──────────────────┘
                    │  LEDs: PD12 CAN-RX | PD13 Cloud-TX          │
                    │        PD14 Crash  | PD15 Fault             │
                    └─────────────────────────────────────────────┘
```

### 2.1 Microcontroller Specifications

| Parameter | Specification |
|-----------|--------------|
| MCU Part Number | STM32F407VGT6 |
| Architecture | ARM Cortex-M4 with FPU (Floating Point Unit) |
| Clock Speed | Up to 168 MHz |
| Flash Memory | 1 MB |
| SRAM | 192 KB (128 KB main + 64 KB CCM) |
| Operating Voltage | 1.8 V – 3.6 V |
| GPIO Pins | 82 I/O pins |
| Timers | 14 timers (TIM1–TIM14): 2× advanced, 10× general, 2× basic |
| ADC | 3× 12-bit ADC; 24 channels; up to 2.4 MSPS |
| CAN Controllers | 2× bxCAN (CAN 2.0B); up to 1 Mbps |
| USART/UART | 6× USART; supports up to 10.5 Mbps |
| SPI | 3× SPI; up to 42 Mbps |
| I2C | 3× I2C |
| USB | 1× USB OTG FS/HS |
| DMA | 2× DMA controllers; 8 streams each |
| Interrupts | 82 maskable interrupt channels; 16 priority levels |
| On-board Debugger | ST-LINK/V2 (SWD interface) |
| On-board LEDs | 4× user LEDs: LD3 (orange PD13), LD4 (green PD12), LD5 (red PD14), LD6 (blue PD15) |
| On-board Sensors | LIS302DL or LIS3DSH MEMS accelerometer (board-specific) |

### 2.2 On-Board LED Assignment for Telematics Project

| LED | Pin | Colour | Telematics Function | Behaviour |
|-----|-----|--------|---------------------|-----------|
| LD3 | PD13 | Orange | System READY / Power ON | ON solid when system initialised; OFF on fault |
| LD4 | PD12 | Green | CAN RX Active Indicator | Blinks on every incoming CAN frame from vehicle bus |
| LD5 | PD14 | Red | Crash / Emergency Indicator | ON solid when crash event detected from Airbag ECU |
| LD6 | PD15 | Blue | Cloud TX / Immobilizer Active | Blinks on every MQTT publish; ON solid when immobilized |

### 2.3 GPIO and Peripheral Pin Assignment

#### CAN Bus Pins

| Signal | STM32 Pin | Direction | Peripheral Used | Notes |
|--------|-----------|-----------|-----------------|-------|
| CAN1 RX | PD0 | INPUT | bxCAN1 | CAN bus receive line; AF9 |
| CAN1 TX | PD1 | OUTPUT | bxCAN1 | CAN bus transmit line; AF9 |
| CAN2 RX | PB5 | INPUT | bxCAN2 | Second CAN bus RX (optional dual-CAN node); AF9 |
| CAN2 TX | PB6 | OUTPUT | bxCAN2 | Second CAN bus TX (optional); AF9 |

#### UART Pins — STM32 ↔ ESP32 (Cloud Communication)

| Signal | STM32 Pin | Direction | Peripheral Used | Notes |
|--------|-----------|-----------|-----------------|-------|
| USART1 TX | PA9 | OUTPUT | USART1 | STM32 → ESP32 GPIO16 (RX2); JSON telemetry + commands |
| USART1 RX | PA10 | INPUT | USART1 | ESP32 GPIO17 (TX2) → STM32; cloud commands in |
| GND | GND | — | — | Common ground with ESP32 |

#### UART Pins — STM32 ↔ GSM Module (SIM800C Emergency Calls)

| Signal | STM32 Pin | Direction | Peripheral Used | Notes |
|--------|-----------|-----------|-----------------|-------|
| USART2 TX | PA2 | OUTPUT | USART2 | STM32 → SIM800C RXD; AT commands for SMS/call |
| USART2 RX | PA3 | INPUT | USART2 | SIM800C TXD → STM32; GSM response parsing |

#### GPIO Pins — Control and Status

| STM32 Pin | Mode | Function | Active Level |
|-----------|------|----------|--------------|
| PC0 | Output Push-Pull | GSM Module Power Enable | HIGH = GSM ON |
| PC1 | Input + EXTI | External Crash Trigger (test) | HIGH = Crash event |
| PC2 | Output Push-Pull | Immobilization Status LED | HIGH = Vehicle immobilised |
| PC3 | Output Push-Pull | Independent Watchdog Test Pin | Toggle every 500 ms |
| PD12 | Output Push-Pull | Discovery LED — CAN RX indicator | HIGH = Frame received |
| PD13 | Output Push-Pull | Discovery LED — Cloud TX indicator | HIGH = MQTT published |
| PD14 | Output Push-Pull | Discovery LED — Crash indicator | HIGH = Crash detected |
| PD15 | Output Push-Pull | Discovery LED — Fault indicator | HIGH = Fault present |

#### SPI Pins — STM32 ↔ W25Q64 External Flash Memory

| Signal | STM32 Pin | Direction | Peripheral Used | Notes |
|--------|-----------|-----------|-----------------|-------|
| SPI1 SCK | PA5 | OUTPUT | SPI1 | W25Q64 CLK; AF5 |
| SPI1 MISO | PA6 | INPUT | SPI1 | W25Q64 DO (data out); AF5 |
| SPI1 MOSI | PA7 | OUTPUT | SPI1 | W25Q64 DI (data in); AF5 |
| CS | PE3 | OUTPUT | GPIO (manual) | W25Q64 Chip Select; active LOW |

---

## 3. Module and Component Selection

This section justifies every hardware component chosen for the Telematics ECU node and provides complete wiring connection tables for each peripheral.

### 3.1 ESP32 — Wi-Fi and Cloud Module

The ESP32-WROOM-32 is chosen as the cloud communication co-processor because the STM32F407VG has no built-in Wi-Fi. The ESP32 handles all TCP/IP, TLS, and MQTT operations — offloading this from the real-time STM32 core entirely.

- Dual-core Xtensa LX6 @ 240 MHz — handles TLS 1.2 handshake without impacting STM32 real-time tasks
- Built-in Wi-Fi 802.11 b/g/n and Bluetooth — no external RF components needed
- 3.3V logic — directly compatible with STM32F407VG UART pins; no level shifter required
- Has ThingsBoard MQTT SDK support (arduino-esp32 + PubSubClient library)
- UART2 (GPIO16/17) is used for STM32 ↔ ESP32 communication at 115200 baud
- 4 MB on-chip flash — enough to store ThingsBoard access token and Wi-Fi credentials in firmware

#### ESP32 Pin Connection Table

| ESP32 Pin | Function | Connected To | Notes |
|-----------|----------|--------------|-------|
| GPIO17 (TX2) | UART2 TX | STM32 PA10 (USART1_RX) | ESP32 transmits → STM32 receives; cloud commands |
| GPIO16 (RX2) | UART2 RX | STM32 PA9 (USART1_TX) | STM32 transmits → ESP32 receives; telemetry JSON |
| GND | Ground | STM32 GND (common) | Shared ground — mandatory for UART to work |
| GPIO2 | Status LED | 220Ω → LED → GND | On-board blue LED; blinks on MQTT publish |
| 3.3V | Power | 3.3V rail (AMS1117) | ⚠️ 3.3V ONLY — do not power from 5V rail |

> **Note:** Both STM32F407VG and ESP32 operate at 3.3V logic levels. No level shifter is required on the UART lines.

### 3.2 SIM800C GSM Module — Emergency Communication

The SIM800C provides GSM/GPRS cellular communication for emergency eCall and SMS when a crash is detected. It is independent of Wi-Fi, ensuring it works even in areas without internet connectivity.

> ⚠️ **Warning:** SIM800C requires 3.7–4.2V supply and can draw up to 2A peak current during GSM transmission. Never power it from the STM32 3.3V rail. Use a dedicated LiPo cell or high-current 4V regulator.

| SIM800C Pin | Connected To | Notes |
|-------------|--------------|-------|
| VCC | 3.7V–4.2V dedicated supply | Separate high-current regulator — NOT the STM32 3.3V rail |
| GND | Common GND | Shared ground with STM32 |
| TXD | STM32 PA3 (USART2_RX) | SIM800C TX → STM32 RX; AT command responses |
| RXD | STM32 PA2 (USART2_TX) | STM32 TX → SIM800C RX; AT commands sent |
| RST | STM32 PC0 (GPIO output) | Pull LOW to hardware-reset the GSM module |

### 3.3 CAN Transceiver (SN65HVD230) Pin Connection

The SN65HVD230 is a 3.3V-compatible CAN transceiver designed for direct connection with the STM32F407VG without any level shifting. It converts the STM32 digital signals on PD0/PD1 into ISO 11898 differential CANH/CANL bus signals.

```
STM32F407VG          SN65HVD230            CAN Bus
──────────────       ──────────────        ──────────
PD1 (CAN1_TX) ──────► D (TXD)             CANH ──────► CANH wire
PD0 (CAN1_RX) ◄────── R (RXD)             CANL ──────► CANL wire
3.3V ───────────────── VCC                  │
GND  ───────────────── GND                ─┴─  120Ω termination
                       Rs ────── GND  (high-speed mode)
```

**Dominant / Recessive bit logic:**

- STM32 bxCAN pulls PD1 (TX) LOW → dominant bit → CANH=3.5V, CANL=1.5V (differential=2V)
- STM32 bxCAN releases PD1 HIGH → recessive bit → CANH=CANL=2.5V (differential≈0V)
- SN65HVD230 Rs pin tied to GND → high-speed mode, no slope limiting — correct for 500 kbps

### 3.4 Complete Bill of Materials (BOM)

| Item | Component | Part Number | Qty | Purpose |
|------|-----------|-------------|-----|---------|
| 1 | Main MCU | STM32F407VGT6 (Discovery Board) | 1 | Real-time CAN processing, UART, SPI, GPIO |
| 2 | Wi-Fi / Cloud Module | ESP32-WROOM-32 | 1 | MQTT / ThingsBoard Cloud / telemetry communication |
| 3 | GSM Module | SIM800C | 1 | Emergency eCall and SMS crash alerts |
| 4 | CAN Transceiver | SN65HVD230 (3.3V) | 1–2 | CAN bus physical layer (one per CAN channel) |
| 5 | SPI Flash Memory | W25Q64 (8 MB) | 1 | Crash logs, DTC logs, event timestamps |
| 6 | Termination Resistor | 120Ω | 2 | CAN bus termination at each bus end |
| 7 | Crystal Oscillator | 8 MHz | 1 | STM32 external clock source |
| 8 | Voltage Regulator 5V | LM7805 | 1 | 12V car battery → 5V conversion |
| 9 | Voltage Regulator 3.3V | AMS1117-3.3 | 1 | 5V → 3.3V for STM32, ESP32, SN65HVD230 |
| 10 | USB Mini-B Cable | Standard USB Mini-B | 1 | Power + ST-LINK debug interface |
| 11 | Breadboard + Wires | Standard | 1 set | Prototyping connections |
| 12 | IDE | STM32CubeIDE (free) | 1 | Firmware development with STM32CubeMX |
| 13 | CAN Analyser (optional) | PEAK PCAN-USB or equivalent | 1 | Verify CAN frames and bus arbitration |

---

## 4. CAN Bus Communication Design

### 4.1 CAN Bus Configuration

| Parameter | Value |
|-----------|-------|
| CAN Standard | CAN 2.0B (Standard 11-bit and Extended 29-bit frames) |
| Bus Speed | 500 kbps |
| STM32 CAN Peripheral | bxCAN1 (primary) + bxCAN2 (optional dual-CAN node) |
| CAN TX Pin | PD1 (AF9 — CAN1_TX) |
| CAN RX Pin | PD0 (AF9 — CAN1_RX) |
| CAN Transceiver | SN65HVD230 (3.3V compatible — direct connection, no level shifter) |
| Termination | 120Ω at each physical end of the bus (two resistors total) |
| Frame Type | Standard 11-bit CAN ID |
| TX Mode | Automatic retransmission; no loopback in production |
| RX Filters | 28 hardware acceptance filter banks; configured to pass only known ECU IDs |
| CAN Timing (168 MHz) | Prescaler=12; BS1=8; BS2=3 → 500 kbps |
| Frame bit count | ~60 bits minimum for 2-byte data frame (before bit stuffing) |
| Frame time @ 500kbps | ~120 µs per frame |

### 4.2 CAN Message Map — Telematics ECU Receives

| CAN ID | Source ECU | Content | Telematics Action |
|--------|------------|---------|-------------------|
| `0x103` | Engine ECU | Engine RPM, coolant temp, throttle | Package into telemetry JSON; monitor for over-RPM |
| `0x102` | ABS ECU | Wheel speed, slip ratio, brake temp | Include in telemetry; correlate with crash events |
| `0x100` | Airbag ECU | Crash flag, severity (Major/Minor) | Trigger GSM eCall + ThingsBoard crash alert immediately |
| `0x106` | BCM ECU | Door/lock state, lights status | Include in vehicle state snapshot |
| `0x108` | Infotainment ECU | Driver speed setting, ambient temp | Log for fleet analytics |
| `0x107` | BMS ECU | Battery voltage, SoC | Report battery health to cloud dashboard |
| `0x103` | Engine ECU | DTC fault code, fault source byte | Store to W25Q64 flash; forward to ThingsBoard fault telemetry |

### 4.3 CAN Message Map — Telematics ECU Transmits

| CAN ID | Frame Name | Byte Layout | Frequency | Recipients |
|--------|------------|-------------|-----------|------------|
| `0x105` | IMMOB_COMMAND | B0: `0xAA`=immobilize / `0x55`=clear; B1: auth token; B2-7: zeros | On command only | BMS ECU |
| `0x10X` | TELEM_HEARTBEAT | B0: system state; B1: cloud connection status (0=OK, 1=lost) | 1 second | All ECUs |
| `0x108` | TELEM_STATUS | B0: crash processed flag; B1: immob state; B2: fault count | 500 ms | Infotainment, BMS |

### 4.4 CAN Frame Structure and Timing

Standard 11-bit CAN data frame structure (used at 500 kbps):

| Field | Bits | Telematics Note |
|-------|------|-----------------|
| SOF (Start of Frame) | 1 | Dominant bit — marks start of frame |
| Arbitration ID | 11 | CAN ID `0x700` (Telematics ECU node) |
| RTR bit | 1 | 0 = data frame; 1 = remote frame (not used here) |
| IDE bit | 1 | 0 = standard frame (11-bit ID) |
| r0 (reserved) | 1 | Must be dominant (0) |
| DLC (Data Length Code) | 4 | Number of data bytes — 0 to 8 |
| Data bytes | 0–64 | Telemetry payload; typically 4–8 bytes per frame |
| CRC sequence | 15 | CRC-15 for error detection |
| CRC delimiter | 1 | Recessive bit |
| ACK slot | 1 | Any receiver pulls dominant to acknowledge |
| ACK delimiter | 1 | Recessive bit |
| EOF (End of Frame) | 7 | Seven recessive bits — marks frame end |
| **TOTAL (2-byte payload)** | **60** | ~120 µs at 500 kbps (before bit stuffing) |

---

## 5. Render Cloud Platform Integration

### 5.1 Architecture Overview

The ESP32 acts as an MQTT client connecting to ThingsBoard Cloud over MQTT on port 1883 (or TLS 8883). ThingsBoard receives telemetry, processes it through the Rule Chain Engine, raises alarms, and manages real-time dashboards.

#### Figure 5 — Render Cloud Integration Architecture

```
  ┌────────────────────────────────────┐        ┌────────────────────────────────────────┐
  │        Vehicle Hardware            │        │    Render Cloud (thingsboard.cloud)    │
  │                                    │        │                                        │
  │  ┌──────────────┐                  │        │  ┌────────────┐   ┌────────────────┐   │
  │  │  CAN Bus ECUs│                  │        │  │MQTT Broker │──►│  Rule Chain    │   │
  │  │  0x100–0x108 │                  │        │  │(Auth Token)│   │  Engine        │   │
  │  └──────┬───────┘                  │        │  └────────────┘   └───┬────────────┘   │
  │         │ CAN 500kbps              │        │                        │               │
  │  ┌──────▼───────┐  UART 115200     │        │          ┌─────────────┼────────────┐  │
  │  │  STM32F407VG │─────────────────►│──MQTT─►│          ▼             ▼            ▼  │
  │  │  FreeRTOS    │◄─────────────────│◄─RPC──-│  ┌──────────┐ ┌────────────┐ ┌───────┐ │
  │  └──────────────┘  RPC Commands    │        │  │ Alarm    │ │ Dashboard  │ │  RPC  │ │
  │         │ UART2                    │        │  │ Manager  │ │ Real-time  │ │Service│ │
  │  ┌──────▼───────┐                  │        │  │ CRITICAL │ │ Widgets    │ │Immob. │ │
  │  │  ESP32-WROOM │  MQTT :1883/8883 │        │  └──────────┘ └────────────┘ └───────┘ │
  │  │  Wi-Fi TLS   ├──────────────────┼────────►                                        │
  │  └──────────────┘                  │        │  ┌─────────────────────────────────┐   │
  │         │ UART2                    │        │  │  Shared Attributes               │  │
  │  ┌──────▼───────┐                  │        │  │  immob_status, crash_flag        │  │
  │  │  SIM800C GSM │ Emergency eCall  │        │  │  (persists when device offline)  │  │
  │  │  SMS Alerts  │                  │        │  └─────────────────────────────────┘   │
  │  └──────────────┘                  │        └────────────────────────────────────────┘
  └────────────────────────────────────┘

```text
ESP32 (MQTT Client)
     │
     │ MQTT — Port 1883 (plaintext) / 8883 (TLS)
     ▼
Render Cloud Platform Server (render.com)
     │
     ├──► Backend Event Processing Service Engine
     │         (process telemetry; raise alarms)
     │
     │         ├──► Render-hosted Web Dashboard
     │         │         (real-time vehicle monitoring widgets)
     │         │
     │         ├──► Alarm Manager
     │         │         (crash alerts; low battery; fault notifications)
     │         │
     │         └──► RPC Service
     │                   (remote immobilization commands to ESP32)
     │
     └──► Cloud Database State Storage
               (last known vehicle state; persists when device offline)
```

---

## 5.2 MQTT Topics

| Topic | Direction | Purpose |
|------|------|------|
| `v1/devices/me/telemetry` | ESP32 → Render | Vehicle telemetry (speed, RPM, battery, faults) every 5 seconds |
| `v1/devices/me/attributes` | ESP32 → Render | Client attribute update (`immob_status`, `crash_flag`, `fault_count`) |
| `v1/devices/me/rpc/request/+` | Render → ESP32 | REST API command handling commands (`immobilize`, `clear`, `OTA trigger`) |
| `v1/devices/me/rpc/response/{requestId}` | ESP32 → Render | RPC response / acknowledgment sent back to Render |
| `v1/devices/me/attributes/response/+` | Render → ESP32 | Shared attribute delta notifications from Render server |

---

## 5.3 Render Setup — Step by Step

### Step 1 — Create a Device

In Render → Entities → Devices → Add Device

- **Device Name:** `Telematics_ECU_VH001`
- **Device Profile:** `Vehicle_Telematics`
- **Label:** `VH001`

---

### Step 2 — Get Access Token (MQTT Authentication)

In Render → Device → Manage Credentials → Copy Access Token  
(example: `A1B2C3D4E5F6G7H8`)

No TLS certificates are needed — embed the access token as the MQTT username in ESP32 firmware. Password field is left blank.

---

### Step 3 — Configure Backend Event Processing Service for Crash Alert

In Render → Backend Event Processing Services → Root Backend Event Processing Service

Add Rule Node:

- `"Message Type Filter"`  
  (`type: POST_TELEMETRY_REQUEST`)

Link to `"Script"` node:

```js
check crash_status == 1
```

On match:

- Trigger `"Create Alarm"` node
  - **Severity:** `CRITICAL`
  - **Type:** `VEHICLE_CRASH`

Connect Alarm node to:

- `"Send Email"` or
- `"REST API Call"` node

for SMS notification.

---

### Step 4 — Create Render-hosted Web Dashboard

In Render → Render-hosted Web Dashboards → Add Render-hosted Web Dashboard

Add the following widgets:

- Time-series line chart (`speed`, `RPM`)
- Radial Gauge (`battery SoC`)
- Boolean indicator (`immob_status`)
- Alarm Widget (`VEHICLE_CRASH` events)
- Value Card (`fault_count`)

Bind all widgets to device `Telematics_ECU_VH001` telemetry keys.

---

## 5.4 Render RPC — Remote Immobilization Flow

Render uses REST API command handling (Remote Procedure Calls) to send commands to the ESP32. Unlike AWS Device Shadow, RPC is a direct request-response mechanism — Render delivers the command payload and waits for an acknowledgment from the device before updating the dashboard state.

### RPC Payload from Render → ESP32

```json
{
  "method": "immobilize",
  "params": {
    "action": "enable",
    "token": "0xAB"
  }
}
```

---

### Flow

1. Fleet manager clicks **Immobilize** button on Render-hosted Web Dashboard

2. Render sends RPC request to ESP32 via:

```text
v1/devices/me/rpc/request/{id}
```

3. ESP32 checks vehicle speed `< 10 km/h` (safety gate)

4. ESP32 sends UART command to STM32:

```text
CMD:IMMOB:AB\n
```

5. STM32 validates token → sends CAN ID=`0x450` to Engine ECU

6. Engine ECU disables ignition → ACK on CAN ID=`0x100`

7. STM32 confirms → ESP32 publishes RPC response to:

```text
v1/devices/me/rpc/response/{id}
```

8. ESP32 updates client attribute:

```text
immob_status=true
```

Dashboard Status:

```text
VH001 — IMMOBILIZED ✓
```
## 6. Software Architecture (FreeRTOS)

### 6.1 FreeRTOS Task Architecture

#### Figure 6 — FreeRTOS Task Architecture & Inter-Task Communication

```
Priority │  Task                  │ Period       │ Stack  │ Responsibility
─────────┼────────────────────────┼──────────────┼────────┼──────────────────────────────────
P5 ████  │  Watchdog_Task         │ 500 ms       │ 256 B  │ Kick IWDG; monitor task health
P4 ███   │  CAN_RxTask            │ Event-driven │ 512 B  │ Process CAN frames from RX FIFO
P3 ██    │  CrashManager_Task     │ Event-driven │ 512 B  │ Crash event → GSM + cloud alert
P3 ██    │  GSM_Task              │ Event-driven │ 512 B  │ SIM800C AT command state machine
P2 █     │  FaultManager_Task     │ 100 ms       │ 512 B  │ DTC poll → flash write → cloud
P1 ░     │  CloudTx_Task          │ 5 seconds    │ 1024 B │ JSON build → UART1 → ESP32

Shared Resources (FreeRTOS Queues & Mutexes):
  ┌─────────────────┐  ┌─────────────────┐  ┌──────────────────┐  ┌──────────────────┐
  │ CAN RX Ring Buf │  │Crash Event Queue│  │  Fault Tbl Mutex │  │  UART TX Mutex   │
  └─────────────────┘  └─────────────────┘  └──────────────────┘  └──────────────────┘

Interrupts (ISR → queue wakeup):
  bxCAN1 RX FIFO0 (Priority 0) ──► wakes CAN_RxTask
  USART1 RX       (Priority 2) ──► cloud command received
  USART2 RX       (Priority 3) ──► GSM AT response
  SPI1 DMA        (Priority 4) ──► flash write complete
```

| Task Name | Priority | Stack | Period | Job |
|-----------|----------|-------|--------|-----|
| Watchdog_Task | 5 (Highest) | 256 B | 500 ms | Kick IWDG; monitor task health; reset if hang |
| CAN_RxTask | 4 | 512 B | Event-driven | Process incoming CAN frames from RX FIFO |
| CrashManager_Task | 3 | 512 B | Event-driven | Handle crash events; trigger GSM + cloud alert |
| GSM_Task | 3 | 512 B | Event-driven | Manage SIM800C AT command state machine |
| FaultManager_Task | 2 | 512 B | 100 ms | Poll fault queue; write DTC to flash; forward to cloud |
| CloudTx_Task | 1 (Lowest) | 1024 B | 5 seconds | Build JSON telemetry; send to ESP32 via UART1 |

### 6.2 Interrupt Priority Assignment

| Interrupt | Priority | Reason |
|-----------|----------|--------|
| bxCAN1 RX FIFO0 | 0 (Highest) | CAN frames must be captured immediately — immobilize command latency |
| TIM6 — System tick | 1 | FreeRTOS tick timer; must not be preempted |
| USART1 — ESP32 RX | 2 | Cloud command from ESP32 must be processed quickly |
| USART2 — GSM RX | 3 | GSM AT response parsing; tolerates brief delay |
| SPI1 — Flash DMA | 4 | Flash write completes asynchronously; low urgency |
| USART2 — GSM TX | 5 (Lowest) | Debug/TX complete; never delays real-time tasks |

### 6.3 Key Configuration Constants (`config.h`)

| `#define` Constant | Value | Meaning |
|--------------------|-------|---------|
| `CAN_SPEED_KBPS` | 500 | CAN bus speed in kbps |
| `CAN_ID_TELEMATICS` | 0x107 | This ECU's CAN heartbeat node ID |
| `CAN_ID_IMMOB` | 0x103 | Immobilization command sent to BMS ECU |
| `CAN_ID_CRASH` | 0x100 | Crash frame received from Airbag ECU |
| `CAN_ID_ENGINE` | 0x103 | Engine ECU base CAN ID |
| `IMMOB_SPEED_LIMIT` | 10 | Maximum vehicle speed (km/h) to allow immobilization |
| `CRASH_DEBOUNCE_FRAMES` | 3 | Crash flag must persist N frames before acting |
| `CLOUD_TX_PERIOD_MS` | 5000 | Telemetry publish period (5 seconds) |
| `FAULT_POLL_MS` | 100 | Fault table poll interval |
| `WATCHDOG_KICK_MS` | 500 | IWDG kick period (must be < watchdog timeout) |
| `UART_BAUD_ESP32` | 115200 | UART1 baud rate to ESP32 |
| `UART_BAUD_GSM` | 9600 | UART2 baud rate to SIM800C |
| `FLASH_CRASH_ADDR` | 0x00001000 | W25Q64 start address for crash log region |
| `FLASH_FAULT_ADDR` | 0x00010000 | W25Q64 start address for DTC fault log region |
| `MAX_FAULT_ENTRIES` | 5000 | Maximum fault records stored in external flash |
| `TEMP_AMBIENT_DEFAULT` | 30 | Default ambient temperature (°C) for GSM thermal model |

---

## 7. Complete Data Flow — Scenarios

### 7.1 Scenario 1 — Crash Detection → eCall + Cloud Alert

#### Figure 3 — Crash Detection Flow

```
  Airbag ECU                STM32F407VG                  ESP32           ThingsBoard
      │                          │                          │                  │
      │  CAN 0x100               │                          │                  │
      │  [0x02, 0x01] MAJOR      │                          │                  │
      ├─────────────────────────►│                          │                  │
      │                          │ bxCAN ISR fires          │                  │
      │                          │ → RX ring buffer         │                  │
      │                          │                          │                  │
      │                          │ CAN_RxTask processes     │                  │
      │                          │ ID=0x321 matched         │                  │
      │                          │                          │                  │
      │                  ┌───────┴──────┐                   │                  │
      │                  │ Debounce ≥ 3 │                   │                  │
      │                  │ frames?      │                   │                  │
      │                  └───────┬──────┘                   │                  │
      │                    YES   │                          │                  │
      │            ┌─────────────┴─────────────┐            │                  │
      │            ▼                           ▼            │                  │
      │    GSM_Task                    Build crash JSON     │                  │
      │    AT+CMGS="+911"              → UART1              │                  │
      │    "Crash: VH001, MAJOR"                            │                  │
      │    📞 Emergency eCall          {"crash":1,"sev":2} ├─────────────────►│
      │                                                     │  MQTT publish    │
      │                                                     │  v1/.../telemetry│ CRITICAL Alarm
      │                                                     │                  ├──► Dashboard
      │                                                     │                  ├──► Email/SMS
      │                          │                          │                  │
      │                 PD14 (Red LED) ON solid             │                  │
      │                 W25Q64 crash log written            │                  │
```

End-to-end trace of a crash event:

| Step | Action | Detail |
|------|--------|--------|
| 1 | Airbag ECU sends CAN frame | CAN ID=`0x100`, Data=`[0x02, 0x01]` (severity=Major, flag=Confirmed) |
| 2 | STM32 bxCAN ISR fires | `HAL_CAN_RxFifo0MsgPendingCallback()`; frame stored in CAN RX ring buffer |
| 3 | CAN_RxTask processes frame | Identifies ID=`0x100`; calls `CrashManager_HandleEvent()` |
| 4 | CrashManager logs to RAM | `CrashEvent.severity=2`, `.timestamp=92450ms`, `.crash_flag=1` |
| 5 | GSM_Task sends AT commands | UART2: `AT+CMGS="+911"` → `"Crash: VH001, MAJOR, 92450ms"` |
| 6 | STM32 builds JSON → UART1 → ESP32 | `{"vehicle_id":"VH001","crash":1,"severity":2,"ts":92450}` |
| 7 | ESP32 publishes to ThingsBoard | Topic: `v1/devices/me/telemetry` (MQTT port 1883) |
| 8 | ThingsBoard Rule Chain triggers alarm | Raises CRITICAL alarm; Alarm Widget updates; email/SMS notification sent |

### 7.2 Scenario 2 — Remote Immobilization from Cloud

#### Figure 4 — Remote Immobilization Sequence

```
ThingsBoard          ESP32               STM32F407VG        CAN Bus        Engine ECU
     │                   │                      │                │                │
     │ RPC request       │                      │                │                │
     │ method:immobilize │                      │                │                │
     ├─────────────────► │                      │                │                │
     │                   │ Request speed        │                │                │
     │                   ├────────────────────► │                │                │
     │                   │ {"speed": 8}         │                │                │
     │                   │◄──────────────────── ┤                │                │
     │                   │                      │                │                │
     │                   ├── speed < 10 km/h ── ┤ Safety Gate ✓  │                │
     │                   │                      │                │                │
     │                   │ "CMD:IMMOB:AB\n"     │                │                │
     │                   ├────────────────────► │                │                │
     │                   │                      │ Validate 0xAB  │                │
     │                   │                      │ CAN 0x450      │                │
     │                   │                      ├───────────────►│                │
     │                   │                      │                │ 0x450 frame    │
     │                   │                      │                ├───────────────►│
     │                   │                      │                │                │ Ignition OFF
     │                   │                      │                │ ACK 0x100      │
     │                   │                      │                │◄───────────────┤
     │                   │ {"immob":"confirmed"}│                │                │
     │                   │◄──────────────────── ┤                │                │
     │ RPC response      │                      │                │                │
     │ immob_status=true │                      │                │                │
     │◄───────────────── ┤                      │                │                │
     │                   │                      │                │                │
  Dashboard: VH001 — IMMOBILIZED ✓
```

| Step | Action | Detail |
|------|--------|--------|
| 1 | Fleet manager clicks Immobilize | ThingsBoard sends RPC: `method=immobilize, params={action:enable, token:0xAB}` |
| 2 | ThingsBoard delivers RPC to ESP32 | Topic: `v1/devices/me/rpc/request/{id}` |
| 3 | ESP32 checks vehicle speed | Requests speed from STM32 via UART; STM32 responds: `{"speed":8}` (safe) |
| 4 | ESP32 sends UART to STM32 | `"CMD:IMMOB:AB\n"` |
| 5 | STM32 validates token | Security Manager verifies `0xAB`; sends CAN ID=`0x103`, Data=`[0xAA,0xAB,...]` |
| 6 | Engine ECU receives `0x103` | Disables ignition relay; ACK on CAN ID=`0x100`, Byte0=`0xFF` (engine_off) |
| 7 | STM32 confirms → ESP32 | `{"immob_status":"confirmed","token":"0xAB"}` |
| 8 | ESP32 updates client attribute | `immob_status=true`; Dashboard: **VH001 — IMMOBILIZED ✓** |

### 7.3 Scenario 3 — Fault Monitoring → Cloud Diagnostics

| Step | Action | Detail |
|------|--------|--------|
| 1 | Engine ECU detects throttle fault | CAN ID=`0x510`, Data=`[0x01, 0x23]` → DTC=`0x0123` (P0123) |
| 2 | STM32 Fault Manager parses DTC | `fault_table[0]={dtc=0x0123, ecu_id=0x01, fault_type=Error}` |
| 3 | Write to W25Q64 flash | SPI write at address `0x00010000` (fault log region) |
| 4 | Build JSON → UART1 → ESP32 | `{"vehicle_id":"VH001","fault":"P0123","ecu":"Engine"}` |
| 5 | ESP32 publishes to ThingsBoard | Topic: `telematics/VH001/fault` |
| 6 | Rule Chain alarm | ThingsBoard creates alarm if same fault repeats ≥ 3 times |

---

## 8. Multi-ECU Integration Notes

### 8.1 What the Telematics ECU Provides to Each Other ECU

| Receiving ECU | Data Provided by Telematics | CAN Frame |
|---------------|----------------------------|-----------|
| Engine ECU | Immobilization/mobilization command with auth token | `0x103` |
| All ECUs | Telematics heartbeat (system alive, cloud status) | `0x10X` |
| Infotainment ECU | Cloud connectivity status; remote command acknowledgment | `0x108` |
| BMS ECU | Remote diagnostic request acknowledgment | `0x105` |

### 8.2 What the Telematics ECU Receives from Each Other ECU

| Sending ECU | Data Received | How Telematics Uses It |
|-------------|---------------|------------------------|
| Engine ECU | RPM, coolant temp, throttle, DTC faults | Package in telemetry JSON; log faults to flash |
| Transmission ECU | Gear, torque converter state | Include in vehicle state snapshot |
| ABS ECU | Wheel speed, slip ratio, brake temperature | Telemetry to cloud; correlate with crash events |
| BCM ECU | Door/lock states, lights | Vehicle state snapshot for Device Shadow |
| Infotainment ECU | Driver speed preference, ambient temperature | Log for fleet analytics; forward to cloud |
| BMS ECU | Battery voltage, State of Charge | Report battery health; alert on low voltage |
| Airbag ECU | Crash flag, severity (`0x321` frame) | Trigger eCall, cloud crash alert, DTC freeze-frame |

---

*Design Document — Telematics ECU | STM32F407VG + ESP32 + ThingsBoard | Document 17*
