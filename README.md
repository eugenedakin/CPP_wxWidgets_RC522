# CPP_wxWidgets_RC522
Read Radio Frequency ID tag with Raspberry Pi, C++, and use wxWidgets GUI.

This C++ program for the Raspberry Pi uses the RC522 RFID module to read RFID tags to allow access into rooms or provide access. This was created in April 2026. 

Ensure the SPI communication protocol is enabled on the Raspberry Pi. The below video shows: 1) running, and 2) waving two separate RFID tags over a sensor to show the ID numbers.


![](https://github.com/eugenedakin/CPP_wxWidgets_RC522/blob/main/RC522Demo.gif)

Installation Instructions:
1. Install Raspberry Pi OS (64-bit)
2. Open a terminal and type the following command:
3. sudo apt install libwxgtk3.2-dev build-essential
4. sudo apt install gpiod libgpiod-dev
5. Create the example program in the terminal with:``g++ -std=c++17 -O2 RC522Rev1.cpp -o RC522Rev1 `wx-config --cxxflags --libs` $(pkg-config --cflags --libs libgpiod)``
6. Run the program with ./RC522Rev1

Below is the breadboard layout with the Raspberry Pi 4 connecting with SPI to the RC522.
![](https://github.com/eugenedakin/CPP_wxWidgets_RC522/blob/main/Breadboard.png)

Connections:
VCC = 3.3V
Rst = Pin22 (GPIO 25)
Gnd = ground
IRQ = not connected
MISO = Pin 21 (SPI MISO) GPIO9
MOSI = Pin 19 (SPIO MOSI) GPIO10
SCK = Pin 23 (SPI SCLK) GPIO11
SDA = Pin 24 (SPI CE0) GPIO8
