# LilyGO T-SIM7000G Project Template

Template per progetti IoT basati su **LilyGO T-SIM7000G** (ESP32 + SIM7000G LTE-M/NB-IoT/GPRS + GPS).

## Caratteristiche

- Connessione cellulare LTE-M / NB-IoT / GPRS
- GPS/GNSS integrato con task asincrono (FreeRTOS)
- MQTT over TCP (no SSL sul modem, usa SSLClient per HTTPS)
- OTA (Over-The-Air updates) via HTTPS con chunked download
- Deep sleep intelligente con gestione batteria
- Sensore temperatura DS18B20 (opzionale)

## Quick Start

### 1. Clona o usa come template

```bash
# Se hai clonato
git clone https://github.com/YOUR_USER/YOUR_PROJECT.git

# Oppure usa "Use this template" su GitHub
```

### 2. Configura il dispositivo

Modifica `src/main.cpp`:

```cpp
// ===== CONFIGURAZIONE DISPOSITIVO =====
const char* DEVICE_ID = "MyDevice01";    // ID univoco del dispositivo
const char* APN = "internet";            // APN del tuo operatore
const char* FIRMWARE_VERSION = "1.0.0";  // Versione firmware
```

### 3. Configura MQTT Broker

Cerca e modifica nel file `src/main.cpp`:

```cpp
const char* broker = "your-mqtt-broker.com";
```

E le credenziali in `mqttConnect()`:

```cpp
mqtt.connect(DEVICE_ID, "username", "password")
```

### 4. Configura porta upload

Modifica `platformio.ini`:

```ini
upload_port = COM8  ; La tua porta seriale
```

### 5. Compila e carica

```bash
pio run -t upload
pio device monitor
```

## Struttura progetto

```
├── src/
│   └── main.cpp          # Firmware principale
├── tools/
│   └── mqtt_cli.py       # CLI per controllo MQTT
├── platformio.ini        # Configurazione PlatformIO
├── reset_delay.py        # Script post-upload
└── .github/
    └── workflows/
        └── build.yml     # GitHub Actions per OTA releases
```

## Pin Configuration (LilyGO T-SIM7000G)

| Pin | Funzione |
|-----|----------|
| 4 | PWR_PIN (modem power) |
| 25 | DTR |
| 26 | RX (modem) |
| 27 | TX (modem) |
| 12 | LED |
| 35 | Battery ADC |
| 18 | OneWire (DS18B20) |

## Topic MQTT

| Topic | Dir | Descrizione |
|-------|-----|-------------|
| `{DEVICE_ID}/Temperature` | OUT | Temperatura sensore |
| `{DEVICE_ID}/BatteryVoltage` | OUT | Tensione batteria |
| `{DEVICE_ID}/lat` | OUT | Latitudine GPS |
| `{DEVICE_ID}/lon` | OUT | Longitudine GPS |
| `{DEVICE_ID}/SignalLevel` | OUT | Qualita segnale |
| `{DEVICE_ID}/init` | OUT | Messaggio avvio |
| `{DEVICE_ID}/cmd/ota` | IN | Comando OTA |
| `{DEVICE_ID}/cmd/reboot` | IN | Comando reboot |
| `{DEVICE_ID}/cmd/diagnostics` | IN | Richiesta diagnostica |
| `{DEVICE_ID}/ota/status` | OUT | Stato OTA |
| `{DEVICE_ID}/diagnostics` | OUT | JSON diagnostica |

## OTA Updates

### Setup GitHub Actions

1. Copia `.github/workflows/build.yml` nel tuo repo
2. Crea un tag per la release:

```bash
git tag v1.0.0
git push origin v1.0.0
```

3. GitHub Actions compila e crea la release automaticamente

### Invia comando OTA

```bash
mosquitto_pub -h your-broker.com -u user -P pass \
  -t "MyDevice01/cmd/ota" \
  -m '{"url":"https://github.com/user/repo/releases/download/v1.0.0/firmware.bin","checksum":"sha256:..."}'
```

## Deep Sleep Logic

| Condizione | Sleep Time |
|------------|------------|
| Giorno (6-18) | 10 minuti |
| Notte (18-6) | 30 minuti |
| Batteria bassa | 1 ora |
| USB alimentato | Mai (skip) |

## APN comuni Italia

| Operatore | APN |
|-----------|-----|
| TIM | `ibox.tim.it` o `shared.tids.tim.it` |
| Vodafone | `mobile.vodafone.it` |
| WindTre | `internet.wind` |
| Iliad | `iliad` |
| ho. | `web.ho-mobile.it` |
| Things Mobile | `TM` |

## License

MIT License - Usa liberamente per i tuoi progetti.

## Credits

Template basato su progetto RailTemp by Tecnocons.
