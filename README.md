# Digital Instrumentation 2022
### Project title:
Boat logging system with GPS and WiFi

### Description:
A logging system to be used on boats with a multitude of sensors and GPS. The system continuously reads from the sensors and shows relevant info on the local display in real time while logging all raw data to an SD card. At a specified time the system uploads filtered data to a cloud server via a WiFi module. For potential hazardous incidents the system will automatically notify the user over the internet. If there is detected water inside the boat the system will activate a water pump and log the water flow. The GPS module and 9DOF sensor keeps track of the boat's position and heading, and will notify the user if the boat moves outside a specified range if this feature is enabled. The system can be locked/unlocked using an RFID reader.


Sensors:
- GPS
- Temperatur / humidity (BME280)
- Water level sensor (HOYA)
- Battery voltage sensor (voltage divider and STM32 ADC)
- IR movement (HC SR501)
- 9DOF (heading/gyroscope)