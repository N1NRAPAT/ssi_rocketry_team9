# ARCs Team 9 : Team Sunfall



## Pin GPIO :

![Pico structure](https://www.raspberrypi.com/documentation/microcontrollers/images/pico2w-pinout.svg)

### Pin register by :

**I2C0 — For IMU + Barometer**
* SDA → GPIO 4
* SCL → GPIO 5
**UART1 — For GPS**
* GPS TX → GPIO 9
* GPS RX → GPIO 8
**SPI0 — For LoRa SX1278**
* SCK → GPIO 18
* MOSI → GPIO 19
* MISO → GPIO 16
* CS → GPIO 17
* DIO0 → GPIO 20
**SPI1 — For SD Card**
* SCK → GPIO 10
* MOSI → GPIO 11
* MISO → GPIO 12
* CS → GPIO 13
**Power**
* All sensors (except GPS) → 3.3V
* GPS → 5V
* All grounds → GND

## Progress : 
### From 28 Nov 2025 until now 

Our Avionic team have been discuss to split rough task into smaller task by setting goal and achivement 
during procession of working on code and stuff which our team more focus on senser, control and stablization
of rocket. However, by two to three week, avionic team start on build fundamental resources of core control
system in plane so contain with IMU:MPU6050, BAROMETER:M5611, and MicroSD-Card all these coming together in 
previous code.




## Run : Command
### How to use command prompt in ssi_rocketry :
Our team created particular rule to make thing much more easier by drag files by hand

'''bash
   cd SSI_ROCKETRY
   ./run.sh
'''

After run this command promt line into your terminal it will showing up Question (1-7).
So you aim to answer which one do you prefer to use started with Configure , Build , Run 

### Avionic team member :
1.Ninrapat 
2.Rohan 
3.Ahmed
4.Kene
5.Saleh 