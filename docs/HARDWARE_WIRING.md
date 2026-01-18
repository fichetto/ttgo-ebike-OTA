# Hardware Wiring Guide

## LilyGO T-SIM7000G Connections

### Power Supply
- Main input: USB-C or battery (3.7V LiPo)
- Logic level: 3.3V

---

## VESC Motor Controller

Connect via UART2:

| LilyGO Pin | VESC Pin | Wire Color (typical) |
|------------|----------|----------------------|
| GPIO 18 | RX | Yellow |
| GPIO 19 | TX | Green |
| GND | GND | Black |

**Settings:**
- Baud rate: 9600
- No flow control

---

## OLED Display (SH1106 128x64)

Connect via I2C:

| LilyGO Pin | Display Pin |
|------------|-------------|
| GPIO 21 | SDA |
| GPIO 22 | SCL |
| 3.3V | VCC |
| GND | GND |

**I2C Address:** 0x3C

---

## Accelerometer (ADXL345)

Connect via I2C (same bus as display):

| LilyGO Pin | ADXL345 Pin |
|------------|-------------|
| GPIO 21 | SDA |
| GPIO 22 | SCL |
| 3.3V | VCC |
| GND | GND |

**I2C Address:** 0x53

---

## Torque Sensor

| LilyGO Pin | Sensor |
|------------|--------|
| GPIO 33 | Signal (analog out) |
| 3.3V | VCC (check sensor voltage!) |
| GND | GND |

**Note:** Verify sensor output voltage range. May need voltage divider if > 3.3V.

---

## PAS Sensor (Pedal Assist)

| LilyGO Pin | Sensor |
|------------|--------|
| GPIO 32 | Signal (digital pulses) |
| 3.3V or 5V | VCC |
| GND | GND |

**Pulses:** 18 per pedal revolution

---

## Hall Sensor (Secondary)

| LilyGO Pin | Sensor |
|------------|--------|
| GPIO 34 | Signal |
| 3.3V | VCC |
| GND | GND |

---

## Battery Voltage Monitoring

| LilyGO Pin | Connection |
|------------|------------|
| GPIO 35 | Voltage divider output |

**Voltage Divider Required:**
- Input: 30-54V battery
- Output: 0-3.3V to ESP32 ADC
- Suggested: 100K + 10K resistors (11:1 ratio)

```
Battery+ ----[100K]----+----[10K]---- GND
                       |
                    GPIO 35
```

**Calculation:**
- Vout = Vin × (10K / 110K)
- 54V → 4.9V (needs adjustment, use 100K + 8.2K for ~3.0V max)

---

## Potentiometer / Throttle

| LilyGO Pin | Connection |
|------------|------------|
| GPIO 12 | Wiper (middle pin) |
| 3.3V | VCC |
| GND | GND |

---

## Buttons

### UP Button
| LilyGO Pin | Connection |
|------------|------------|
| GPIO 36 | Button to GND |

### DOWN Button
| LilyGO Pin | Connection |
|------------|------------|
| GPIO 39 | Button to GND |

**Note:** Use internal pull-up resistors (INPUT_PULLUP) or external 10K to 3.3V.

---

## Wiring Diagram (ASCII)

```
                    +------------------+
                    |  LilyGO T-SIM7000G |
                    |                  |
    VESC RX <-------|  GPIO 18 (TX)    |
    VESC TX ------->|  GPIO 19 (RX)    |
                    |                  |
    I2C SDA <------>|  GPIO 21         |-------> OLED SDA
                    |                  |-------> ADXL345 SDA
    I2C SCL <------>|  GPIO 22         |-------> OLED SCL
                    |                  |-------> ADXL345 SCL
                    |                  |
    Torque -------->|  GPIO 33 (ADC)   |
    PAS ----------->|  GPIO 32         |
    Hall2 --------->|  GPIO 34         |
    Batt V -------->|  GPIO 35 (ADC)   |
                    |                  |
    Throttle ------>|  GPIO 12 (ADC)   |
                    |                  |
    BTN UP -------->|  GPIO 36         |
    BTN DOWN ------>|  GPIO 39         |
                    |                  |
                    |  GPIO 4  (PWR)   |-----> Modem Power
                    |  GPIO 25 (DTR)   |-----> Modem DTR
                    |  GPIO 26 (RX)    |<----- Modem TX
                    |  GPIO 27 (TX)    |-----> Modem RX
                    +------------------+
```

---

## Checklist Before Power On

- [ ] Verify all connections with multimeter (continuity)
- [ ] Check voltage levels (no 5V signals to ESP32 ADC!)
- [ ] Install voltage divider for battery monitoring
- [ ] Insert SIM card (data plan active)
- [ ] Connect antenna (cellular + GPS)
- [ ] Secure all connections (vibration resistant)

---

## Safety Notes

1. **Never connect battery directly to ADC** - always use voltage divider
2. **Check sensor voltage levels** - ESP32 ADC max is 3.3V
3. **Use proper wire gauges** for motor power connections
4. **Isolate high-voltage lines** from signal wires
5. **Add fuse protection** on battery positive line
