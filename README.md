
## TTN Floaty McFloatFace
# 23-02-2020

Arduino/ESP32/TheThingsNetwork based, GPS tracked mobile buoy for sea/air temperature and estury current tracking.

The code isnt pretty but this was my first attempt with Arudino - be gentle!

Designed to be low power, with a large battery (4x18650's) lasts over a week.
Wakes up every 60 seconds and sends a LoRa Packet with:
  Water temperature
  Air Temperature
  Internal temperature and humidity
  GPS (Lat, Long, HDOP)
  Battery Voltage
  Solar Voltage

#The process is simple;

1. Wakes up / Power on.
2. Turn on GPS power ('gpsen' pin is connected to the 2n2222 transistor to provide power to the GPS module).
3. Waits up to 60 seconds to get a GPS lock and sleeps if none.
4. Reads all sensors and makes packet.
5. Sends (ABP method) to The Things Network.

#TODO:
Change from ABP to OTAA. ABP was quick, and im not yet familiar enough to get OTAA working when the ESP sleeps
- need to store the join info to flash
Also for the same reason, frame counter should be continuous (right now its always doing it for the first time ;))
Tidy up!
