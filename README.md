This is ARCs team 9 : 

Workflow Statement :

1. Everytime you code on this git repository. Make sure that you alway 
   check git status by write the terminal 
   
   terminal : git status 
              git pull 

2. After you finish coding stuff and if you create folder onto this workspace 
   you may have to press Crt + shift + p then type => Cmake:Configulation

   2.1 Cmake:configure => Cmake:Build 

   Inside CmakeList.txt find => set (SOURCES
                                     src/main
                                     src/imu)
   
   Add your file.c or file.h to this SOURCES                             

3. Finally, 

   termianl: git add. 
             git commit -m "your update"
             git push 


Avionic team member :
  Ninrapat 
  Rohan 
  Ahmed
  Kene 

  

Pin GPIO :

I2C0 — For IMU + Barometer
* SDA → GPIO 4
* SCL → GPIO 5
UART1 — For GPS
* GPS TX → GPIO 9
* GPS RX → GPIO 8
SPI0 — For LoRa SX1278
* SCK → GPIO 18
* MOSI → GPIO 19
* MISO → GPIO 16
* CS → GPIO 17
* DIO0 → GPIO 20
SPI1 — For SD Card
* SCK → GPIO 10
* MOSI → GPIO 11
* MISO → GPIO 12
* CS → GPIO 13
Power
* All sensors (except GPS) → 3.3V
* GPS → 5V
* All grounds → GND

