# E-Bike Integration Plan

## Overview
Integration of e-bike management software (display, traction, torque sensor) into the LilyGO T-SIM7000G template.

## Current Template Status
- **Board:** LilyGO T-SIM7000G (ESP32 + SIM7000G modem)
- **Partition:** min_spiffs.csv (1920 KB per app, OTA-ready)
- **Flash usage:** 47.3% (929 KB / 1920 KB) - ~1 MB available
- **RAM usage:** 14.2% (46 KB / 327 KB)

### Already Implemented
- Cellular connectivity (TinyGSM + SIM7000G)
- MQTT client with subscriptions
- GPS background task (thread-safe with mutex)
- OTA firmware update via HTTPS (chunked download)
- Deep sleep management with RTC wake
- Temperature sensor (DS18B20)

## Integration Phases

### Phase 1: VESC UART Communication
- **Library:** VescUart (already in lib/)
- **UART2:** TX=18, RX=19 @ 9600 baud
- **Functions:** Motor current control, telemetry readback
- **Data:** RPM, voltage, current, duty cycle, Ah consumed

### Phase 2: Sensor ADC Inputs
- **Torque sensor:** Pin 33 (ADC, range 1700-4095)
- **PAS sensor:** Pin 32 (digital, 18 pulses/revolution)
- **Hall sensor 2:** Pin 34
- **Battery voltage:** Pin 35 (ADC, 30-54V range)
- **Potentiometer:** Pin 12 (ADC, throttle)

### Phase 3: OLED Display (SH1106)
- **Library:** Adafruit_SH1106 + Adafruit_GFX (already in lib/)
- **I2C:** SDA=21, SCL=22, Address=0x3C
- **Resolution:** 128x64
- **Pages:** Multi-page UI with gauges

### Phase 4: Traction Control Algorithm
- **Assist levels:** 6 levels [0.0, 0.3, 0.8, 1.2, 1.8, 2.5] A multipliers
- **Ramping:** 15A/second acceleration
- **Hysteresis:** Smooth pedal response
- **Cadence threshold:** 20 RPM minimum

### Phase 5: Button Interface
- **UP button:** Pin 36 (assist+, page navigation)
- **DOWN button:** Pin 39 (assist-)
- **Bypass PAS:** Pin 21
- **Debounce:** 50ms
- **Long press:** 400ms threshold

### Phase 6: Accelerometer (ADXL345)
- **I2C:** Address 0x53 (shared bus with display)
- **Function:** Pitch angle for hill detection
- **Calibration:** Initial offset calculation

## Hardware Pinout Summary

| Pin | Function | Type | Notes |
|-----|----------|------|-------|
| 4 | Modem PWR | Output | SIM7000G power control |
| 12 | Potentiometer/Throttle | ADC | Range 180-4095 |
| 18 | VESC TX | UART2 | 9600 baud |
| 19 | VESC RX | UART2 | 9600 baud |
| 21 | I2C SDA | I2C | Display + Accelerometer |
| 22 | I2C SCL | I2C | Display + Accelerometer |
| 25 | Modem DTR | Output | SIM7000G |
| 26 | Modem RX | UART1 | 115200 baud |
| 27 | Modem TX | UART1 | 115200 baud |
| 32 | PAS Sensor | Digital | 18 pulses/rev |
| 33 | Torque Sensor | ADC | Range 1700-4095 |
| 34 | Hall Sensor 2 | Digital | Secondary |
| 35 | Battery Voltage | ADC | 30-54V via divider |
| 36 | Button UP | Input | Pull-up |
| 39 | Button DOWN | Input | Pull-up |

## MQTT Topics (E-Bike Telemetry)

| Topic | Description |
|-------|-------------|
| eMTB01/AmpMotore | Motor current (A) |
| eMTB01/Watt | Power output (W) |
| eMTB01/BatteryVoltage | Battery voltage (V) |
| eMTB01/lat | GPS latitude |
| eMTB01/lon | GPS longitude |
| eMTB01/speed | GPS speed (km/h) |
| eMTB01/Status | System status |
| eMTB01/Stop | Remote STOP/START command |

## Traccar GPS Integration
- **Server:** gps.tecnocons.com:5055
- **Device ID:** 20240816EMTB
- **Protocol:** HTTP POST
- **Interval:** 60 seconds

## Key Parameters

### Battery
- **Cutoff voltage:** 30V (flagStop)
- **Max voltage:** 54V
- **Energy curve:** 13-point Wh mapping

### Motor Control
- **Max current:** 35A
- **Voltage range:** 30-54V
- **Ramp rate:** 15A/second
- **Base cycle:** 100ms

### Torque Sensor
- **ADC range:** 1700-4095 → 0-100%
- **Filter:** 200-sample moving average
- **Zero-speed threshold:** 10

### Cadence
- **Magnets:** 6 (corona wheel)
- **PAS pulses:** 18 per revolution
- **Min threshold:** 20 RPM

## FreeRTOS Task Architecture

| Task | Core | Priority | Function |
|------|------|----------|----------|
| GPS Task | 0 | 1 | Background GPS polling |
| Display/VESC | 0 | 1 | UI + motor telemetry |
| Traction | 1 | 1 | Motor control loop |
| MQTT/Main | 1 | 1 | Connectivity |

## Notes
- Modem mutex required for thread-safe GPS/MQTT access
- VESC communication on separate UART (no conflict)
- I2C bus shared between display and accelerometer
- Deep sleep must gracefully shutdown all tasks
