# ARCs Team 9 : ICARIAN APOGEE


## Introduction of our group : 
Our team decide to participate in ssi-society to develop a rocket that can reach the stratosphere and beyond. We are a group of passionate engineers student that are dedicated our hobby time to develop technologies and contribute to how to design rockeet in real world situation. We are not aim to make it fastest or most efficient rocket but we are aim to make it more reliable and safe rocket. However, the main motivation that always drive us to aganist the competition is to learn and having time during the competition as much as possible.

## Roadmap & Workflow

<p align="center">
  <img src="data/document_image/flowchart_servo_fins.png" width="850">
</p>


## Pin GPIO :

![Pico structure](https://www.raspberrypi.com/documentation/microcontrollers/images/pico2w-pinout.svg)

# SSI Rocketry — Pico W Flight Computer

A Raspberry Pi Pico W based rocket flight computer with IMU, barometric altimeter, GPS, LoRa telemetry, and SD card data logging.

---

## Hardware Overview

### Microcontroller
- **Raspberry Pi Pico W**

### Sensors & Modules

| Module | Function |
|---|---|
| MPU-6050 | IMU — accelerometer + gyroscope |
| MS5611-01BA | Barometric altimeter — altitude + apogee detection |
| NEO-6M-0-001 | GPS — position + speed |
| SX1278 Ra-02 | LoRa radio — telemetry + apogee alert |
| SD Module | SD card data logger |

### Power System

| Component | Details |
|---|---|
| Battery | 2S LiPo (7.4V) |
| BMS | HX-2S-D20 — 20A, over-discharge/charge/short circuit + cell balancing |
| Buck Converter | MP1584EN — 7.4V → 3.3V, 3A, ~2cm × 1.5cm |

---

## Pin Wiring

### MPU-6050 — IMU (I2C1)
> **Note:** MPU-6050 and MS5611 share the same I2C bus (same SDA/SCL pins).  
> If boards are split and wiring from the same pin is difficult, move the IMU to GPIO 11/12.

| MPU-6050 | Pico W | Pin # |
|---|---|---|
| SDA | GP2 *(or GP11 if split)* | Pin 4 *(or Pin 15)* |
| SCL | GP3 *(or GP12 if split)* | Pin 5 *(or Pin 16)* |
| VCC | 3.3V | Pin 36 |
| GND | GND | Pin 38 |

### MS5611-01BA — Barometer (I2C1)

| MS5611 | Pico W | Pin # |
|---|---|---|
| SDA | GP2 | Pin 4 |
| SCL | GP3 | Pin 5 |
| VCC | 3.3V | Pin 36 |
| GND | GND | Pin 38 |

### NEO-6M-0-001 — GPS (UART0)

| NEO-6M | Pico W | Pin # |
|---|---|---|
| TX | GP0 | Pin 1 |
| RX | GP1 | Pin 2 |
| VCC | 3.3V | Pin 36 |
| GND | GND | Pin 38 |

### SX1278 Ra-02 — LoRa Radio (SPI1)

| SX1278 | Pico W | Pin # |
|---|---|---|
| SCK | GP18 | Pin 24 |
| MISO | GP16 | Pin 21 |
| MOSI | GP19 | Pin 25 |
| NSS (CS) | GP17 | Pin 22 |
| RST | GP22 | Pin 29 |
| DIO0 | GP20 | Pin 26 |
| VCC | 3.3V | Pin 36 |
| GND | GND | Pin 38 |

### SD Card Module — Logger (SPI0)

| SD Module | Pico W | Pin # |
|---|---|---|
| MISO | GP8 | Pin 11 |
| MOSI | GP7 | Pin 10 |
| SCK | GP6 | Pin 9 |
| CS | GP9 | Pin 12 |
| VCC | 3.3V | Pin 36 |
| GND | GND | Pin 38 |

---

## Full Pin Map Summary

```
UART0  – GPS          →  GP0  (TX),  GP1  (RX)
I2C1   – IMU + Baro   →  GP2  (SDA), GP3  (SCL)
         IMU alt.     →  GP11 (SDA), GP12 (SCL)   ← if boards are split
SPI0   – SD Card      →  GP6  (SCK), GP7  (MOSI), GP8 (MISO), GP9 (CS)
SPI1   – LoRa SX1278  →  GP16 (MISO),GP18 (SCK),  GP19(MOSI), GP17(NSS)
         LoRa extras  →  GP20 (DIO0),GP22 (RST)
```

---

## Power Chain

```
2S LiPo (7.4V)
      ↓
HX-2S-D20 BMS
  (over-discharge / over-charge / short circuit / cell balancing)
      ↓
MP1584EN Buck Converter  (7.4V → 3.3V, 3A)
      ↓
Pico W VSYS (Pin 39)
      ↓
Pico onboard regulator → 3.3V rail → all sensors
```

---

## Software

### Commands (USB Serial)

| Command | Action |
|---|---|
| `i` | IMU mode — live accel + gyro |
| `b` | Baro mode — pressure, temperature, altitude |
| `g` | GPS mode — lat, lon, speed |
| `a` | All sensors |
| `t<sec>` | CSV log for N seconds (e.g. `t30`) |
| `n` | Stop current mode |

### Apogee Alert
- Monitors vertical speed from MS5611
- Arms above **50m** altitude
- Sends LoRa alert every second when vertical speed drops below **5 m/s** (approaching apogee)
- Sends final `APOGEE:CONFIRMED` message when descent begins

### CSV Log Format (all sensors)
```
CSV_HEADER,sample,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,pressure_pa,temperature_c,altitude_baro_m,lat,lon,speed_kts
CSV_DATA,0,0.0012,0.0008,1.0021,0.12,0.08,0.04,101325.00,21.50,0.00,53.381290,-1.465775,0.1200
```

---

## File Structure

```
SSI_rocketry/
├── CMakeLists.txt
├── pico_sdk_import.cmake
├── src/
│   ├── main.c              # Main firmware — IMU + Baro + GPS + LoRa + SD
│   ├── neo6m_gps.h         # GPS driver header
│   ├── neo6m_gps.c         # GPS driver — lat, lon, speed
│   ├── lora_sx1278.h       # LoRa driver header
│   ├── lora_sx1278.c       # LoRa SX1278 SPI driver
│   ├── apogee_alert.h      # Apogee alert header
│   └── apogee_alert.c      # Apogee detection + LoRa alert
├── ground_rx/
│   └── ground_rx.c         # Ground station receiver firmware
└── libs/
    └── FatFs_SPI/          # SD card FatFs library
```

---

## Build Instructions

```bash
mkdir build && cd build
cmake ..
make -j4
```
Flash `SSI_rocketry.uf2` to the Pico W by holding BOOTSEL and copying the file.

---

## Notes

- **SX1278 is 3.3V only** — never connect to 5V
- **Always attach antenna** to SX1278 before powering on
- **GPS cold start** takes 1–3 minutes outdoors to acquire fix
- **Set MP1584EN output to exactly 3.3V** with a multimeter before connecting Pico
- SD card must be formatted as **FAT32**
- BMS rated at 20A gives ~60x headroom over peak system draw (~325mA)


### **Avionic team member :**

   1. Ninrapat (Overall-code-management)
   2. Rohan    (Avionic lead)
   3. Ahmed    (Power and Design) 
   4. Saleh    (Simulation post flight)
   
   