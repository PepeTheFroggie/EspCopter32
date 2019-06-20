# EspCopter

Project using the ESP8266 and MPU6050 to control a quadcopter. Done with arduino for esp8266 ide. RC uses the ack-less ESPNOW protocol. See "espnow_RC_TX" for transmitter.

Serial Konze 0xF5 protocol:
The output to esc's is wired to the serial output pin #2 of the esp. Name is D4, GPIO02, TXD1. Baudrate is 128kb.

Normal PWM output: (#define PWMOUT)
The output to esc's is wired to the pins 14, 12, 13, 15. Refresh rate is about 6ms, pulses from 1ms to 2ms.

RC pulse input sequence is adaptable: 

* define ROL 0
* define PIT 1
* define THR 2
* define RUD 3
* define AU1 4

The copter will only arm after 1 second zero throttle. The copter will shut down motors RC data fails for more than 100ms.

The PID for level (acc) and gyro can be adjusted individually.  

To calibrate the ACC enter 'A' in the serial console.

6050 wiring: 

* SCL to D1 
* SDA to D2 
* VCC to 3.3V 
* GND to GND.
 
![wiring.png](Wiring.png "Wiring")

[Testflight](https://youtu.be/OhVVPzNwx6M)   
[Telemetry](https://youtu.be/0AWHVxgIqno)   

