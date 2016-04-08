/* 
	LTR303-ALS library example sketch
	Lovelesh, thingTronics

This sketch shows how to use the LTR303 library to read the Lite-On LTR303-ALS light sensor.

Hardware connections:

3V3 to 3.3V
GND to GND

(WARNING: do not connect 3V3 to 5V or the sensor will be damaged!)

You will also need to connect the I2C pins (SCL and SDA) to your Arduino.
The pins are different on different Arduinos:

                    SDA    SCL
Any Arduino        "SDA"  "SCL"
Uno, Pro            A4     A5
Mega2560, Due       20     21
Leonardo            2      3
ESP8266				5      4

You can connect the INT (interrupt) pin  but it is not required for basic operation.

*/

// Your sketch must #include this library, and the Wire library
// (Wire is a standard library included with Arduino):

#include <LTR303.h>
#include <Wire.h>

// Create an LTR303 object, here called "light":

LTR303 light;

// Global variables:

unsigned char gain;     // Gain setting, values = 0-7 
unsigned char integrationTime;  // Integration ("shutter") time in milliseconds
unsigned char measurementRate;  // Interval between DATA_REGISTERS update

void setup() {
  // Initialize the Serial port:
  
  Serial.begin(9600);
  Serial.println("LTR303-ALS example sketch");

  // Initialize the LTR303 library
  // 100ms 	initial startup time required
  delay(100);

  // You can pass nothing to light.begin() for the default I2C address (0x29)
  light.begin();

  // Get factory ID from sensor:
  // (Just for fun, you don't need to do this to operate the sensor)

  unsigned char ID;
  
  if (light.getPartID(ID)) {
    Serial.print("Got Sensor Part ID: 0X");
    Serial.print(ID,HEX);
  }
  // Most library commands will return true if communications was successful,
  // and false if there was a problem. You can ignore this returned value,
  // or check whether a command worked correctly and retrieve an error code:
  else {
    byte error = light.getError();
    printError(error);
  }

  // The light sensor has a default integration time of 100ms,
  // and a default gain of low (1X).
  
  // If you would like to change either of these, you can
  // do so using the setControl() and setMeasurementRate() command.
  
  // Gain can take any value from 0-7, except 4 & 5
  // If gain = 0, device is set to 1X gain (default)
  // If gain = 1, device is set to 2X gain
  // If gain = 2, device is set to 4X gain
  // If gain = 3, device is set to 8X gain
  // If gain = 4, invalid
  // If gain = 5, invalid
  // If gain = 6, device is set to 48X gain
  // If gain = 7, device is set to 96X gain
  gain = 0;
  Serial.println("Setting Gain...");
  light.setControl(gain, false, false);

  // If integrationTime = 0, integrationTime will be 100ms (default)
  // If integrationTime = 1, integrationTime will be 50ms
  // If integrationTime = 2, integrationTime will be 200ms
  // If integrationTime = 3, integrationTime will be 400ms
  // If integrationTime = 4, integrationTime will be 150ms
  // If integrationTime = 5, integrationTime will be 250ms
  // If integrationTime = 6, integrationTime will be 300ms
  // If integrationTime = 7, integrationTime will be 350ms

  unsigned char time = 1;

  // If integrationTime = 0, integrationTime will be 100ms (default)
  // If integrationTime = 1, integrationTime will be 50ms
  // If integrationTime = 2, integrationTime will be 200ms
  // If integrationTime = 3, integrationTime will be 400ms
  // If integrationTime = 4, integrationTime will be 150ms
  // If integrationTime = 5, integrationTime will be 250ms
  // If integrationTime = 6, integrationTime will be 300ms
  // If integrationTime = 7, integrationTime will be 350ms
  
  Serial.println("Set timing...");
  light.setMeasurementRate(time,3);

  // To start taking measurements, power up the sensor:
  
  Serial.println("Powerup...");
  light.setPowerUp();
  
  // The sensor will now gather light during the integration time.
  // After the specified time, you can retrieve the result from the sensor.
  // Once a measurement occurs, another integration period will start.
}

void loop() {
  // Wait between measurements before retrieving the result
  // You can also configure the sensor to issue an interrupt 
  // when measurements are complete)
  
  // This sketch uses the LTR303's built-in integration timer.
  
  int ms = 1000;
  
  delay(ms);
  
  
  // Once integration is complete, we'll retrieve the data.
  
  // There are two light sensors on the device, one for visible light
  // and one for infrared. Both sensors are needed for lux calculations.
  
  // Retrieve the data from the device:

  unsigned int data0, data1;
  
  if (light.getData(data0,data1)) {
    // getData() returned true, communication was successful
    
    Serial.print("data0: ");
    Serial.println(data0);
    Serial.print("data1: ");
    Serial.println(data1);
  
    // To calculate lux, pass all your settings and readings
    // to the getLux() function.
    
    // The getLux() function will return 1 if the calculation
    // was successful, or 0 if one or both of the sensors was
    // saturated (too much light). If this happens, you can
    // reduce the integration time and/or gain.
  
    double lux;    // Resulting lux value
    boolean good;  // True if neither sensor is saturated
    
    // Perform lux calculation:

    good = light.getLux(gain,integrationTime,data0,data1,lux);
    
    // Print out the results:
	
    Serial.print(" lux: ");
    Serial.println(lux);
    if (good) Serial.println(" (good)"); else Serial.println(" (BAD)");
  }
  else {
    // getData() returned false because of an I2C error, inform the user.

    byte error = light.getError();
    printError(error);
  }
}

void printError(byte error) {
  // If there's an I2C error, this function will
  // print out an explanation.

  Serial.print("I2C error: ");
  Serial.print(error,DEC);
  Serial.print(", ");
  
  switch(error) {
    case 0:
      Serial.println("success");
      break;
    case 1:
      Serial.println("data too long for transmit buffer");
      break;
    case 2:
      Serial.println("received NACK on address (disconnected?)");
      break;
    case 3:
      Serial.println("received NACK on data");
      break;
    case 4:
      Serial.println("other error");
      break;
    default:
      Serial.println("unknown error");
  }
}