/*
	LTR303 illumination sensor library for Arduino
	Lovelesh, thingTronics
	
The MIT License (MIT)

Copyright (c) 2015 thingTronics Limited

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

version 0.1
*/

#include <LTR303.h>
#include <Wire.h>


LTR303::LTR303(void) {
	// LTR303 object
}

boolean LTR303::begin(void) {
	// Initialize LTR303 library with default address (0x39)
	// Always returns true

	return(begin(LTR303_ADDR));
}

boolean LTR303::setPowerUp(void) {
	// Turn on LTR303, begin integrations
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)

	// Write 0x03 (reset = 1 & mode = 1) to command byte (power on)
	return(writeByte(LTR303_CONTR,0x03));
}


boolean LTR303::setPowerDown(void) {
	// Turn off LTR303
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)

	// Clear command byte (reset = 0 & mode = 0)(power off)
	return(writeByte(LTR303_CONTR,0x00));
}


boolean LTR303::setControl(byte gain, boolean reset, boolean mode) {
	// Sets the gain, SW reset and mode of LTR303
	// Default value is 0x00
	// If gain = 0, device is set to 1X gain (default)
	// If gain = 1, device is set to 2X gain
	// If gain = 2, device is set to 4X gain
	// If gain = 3, device is set to 8X gain
	// If gain = 4, invalid
	// If gain = 5, invalid
	// If gain = 6, device is set to 48X gain
	// If gain = 7, device is set to 96X gain
	//----------------------------------------
	// If reset = false(0), initial start-up procedure not started (default)
	// If reset = true(1), initial start-up procedure started
	//----------------------------------------
	// If mode = false(0), stand-by mode (default)
	// If mode = true(1), active mode
	
	byte control = 0x00;
	
	// sanity check for gain
	if (gain > 3 && gain < 6) {
		gain = 0x00;
	}
	else if(gain >= 7) {
		gain = 0x00;
	}
	
	// control byte logic
	control |= gain << 2;
	if(reset) {
		control |= 0x02;
	}
	
	if(mode) {
		control |= 0x01;
	}
	
	return(writeByte(LTR303_CONTR,control));
}			
			
boolean LTR303::getControl(byte &gain, boolean reset, boolean mode) {
	// Gets the control register values
	// Default value is 0x00
	// If gain = 0, device is set to 1X gain (default)
	// If gain = 1, device is set to 2X gain
	// If gain = 2, device is set to 4X gain
	// If gain = 3, device is set to 8X gain
	// If gain = 4, invalid
	// If gain = 5, invalid
	// If gain = 6, device is set to 48X gain
	// If gain = 7, device is set to 96X gain
	//----------------------------------------
	// If reset = false(0), initial start-up procedure not started (default)
	// If reset = true(1), initial start-up procedure started
	//----------------------------------------
	// If mode = false(0), stand-by mode (default)
	// If mode = true(1), active mode
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)			
	
	byte control;
	
	// Reading the control byte
	if(readByte(LTR303_CONTR, control)) {
		// Extract gain
		gain = (control & 0x1C) >> 2;
		
		// Extract reset
		reset = ((control >> 1) & 0x02) ? true : false; 
		
		// Extract mode
		mode = (control & 0x01) ? true : false;
		
		// return if successful
		return(true);
	}
	return(false);
}

boolean LTR303::setMeasurementRate(byte integrationTime, byte measurementRate) {
	// Sets the integration time and measurement rate of the sensor
	// integrationTime is the measurement time for each ALs cycle
	// measurementRate is the interval between DATA_REGISTERS update
	// measurementRate must be set to be equal or greater than integrationTime
	// Default value is 0x03
	// If integrationTime = 0, integrationTime will be 100ms (default)
	// If integrationTime = 1, integrationTime will be 50ms
	// If integrationTime = 2, integrationTime will be 200ms
	// If integrationTime = 3, integrationTime will be 400ms
	// If integrationTime = 4, integrationTime will be 150ms
	// If integrationTime = 5, integrationTime will be 250ms
	// If integrationTime = 6, integrationTime will be 300ms
	// If integrationTime = 7, integrationTime will be 350ms
	//------------------------------------------------------
	// If measurementRate = 0, measurementRate will be 50ms
	// If measurementRate = 1, measurementRate will be 100ms
	// If measurementRate = 2, measurementRate will be 200ms
	// If measurementRate = 3, measurementRate will be 500ms (default)
	// If measurementRate = 4, measurementRate will be 1000ms
	// If measurementRate = 5, measurementRate will be 2000ms
	// If measurementRate = 6, measurementRate will be 2000ms
	// If measurementRate = 7, measurementRate will be 2000ms
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
	
	byte measurement = 0x00;
	
	// Perform sanity checks
	if(integrationTime => 7) {
		integrationTime = 0x00;
	}
	
	if(measurementRate => 7) {
		measurementRate = 0x00;
	}
	
	measurement |= integrationTime << 3;
	measurement |= measurementRate;
	
	return(writeByte(LTR303_MEAS_RATE, measurement));
}

boolean LTR303::getMeasurementRate(byte &integrationTime, byte &measurementRate) {
	// Gets the value of Measurement Rate register
	// Default value is 0x03
	// If integrationTime = 0, integrationTime will be 100ms (default)
	// If integrationTime = 1, integrationTime will be 50ms
	// If integrationTime = 2, integrationTime will be 200ms
	// If integrationTime = 3, integrationTime will be 400ms
	// If integrationTime = 4, integrationTime will be 150ms
	// If integrationTime = 5, integrationTime will be 250ms
	// If integrationTime = 6, integrationTime will be 300ms
	// If integrationTime = 7, integrationTime will be 350ms
	//------------------------------------------------------
	// If measurementRate = 0, measurementRate will be 50ms
	// If measurementRate = 1, measurementRate will be 100ms
	// If measurementRate = 2, measurementRate will be 200ms
	// If measurementRate = 3, measurementRate will be 500ms (default)
	// If measurementRate = 4, measurementRate will be 1000ms
	// If measurementRate = 5, measurementRate will be 2000ms
	// If measurementRate = 6, measurementRate will be 2000ms
	// If measurementRate = 7, measurementRate will be 2000ms
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
			
	byte measurement = 0x00;
	
	// Reading the measurement byte
	if(readByte(LTR303_MEAS_RATE, measurement)) {
		// Extract integration Time
		integrationTime = (measurement & 0x38) >> 3;
		
		// Extract measurement Rate
		measurementRate = measurement & 0x07; 
		
		// return if successful
		return(true);
	}
	return(false);		
}

boolean LTR303::getPartID(byte &partID) {
	// Gets the part number ID and revision ID of the chip
	// Default value is 0x0A
	// part number ID = 0x0A (default)
	// Revision ID = 0x00
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
			
	return(readByte(LTR303_PART_ID, partID));
}

boolean LTR303::getManufacID(byte &manufacID) {
	// Gets the Manufacturers ID
	// Default value is 0x05
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
	
	return(readByte(LTR303_MANUFAC_ID, manufacID));
}

boolean LTR303::getData(unsigned int &CH0, unsigned int &CH1) {
	// Gets the 16-bit channel 0 and channel 1 data
	// Default value of both channels is 0x00
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
	
	return(readUInt(LTR303_DATA_CH0_0,CH0) && readUInt(LTR303_DATA_CH1_0,CH1));
}

boolean LTR303::getStatus(boolean valid, byte &gain, boolean intrStatus, boolean dataStatus) {
	// Gets the status information of LTR303
	// Default value is 0x00
	// If valid = false(0), Sensor data is valid (default)
	// If valid = true(1), Sensor data is invalid
	//--------------------------------------------
	// If gain = 0, device is set to 1X gain (default)
	// If gain = 1, device is set to 2X gain
	// If gain = 2, device is set to 4X gain
	// If gain = 3, device is set to 8X gain
	// If gain = 4, invalid
	// If gain = 5, invalid
	// If gain = 6, device is set to 48X gain
	// If gain = 7, device is set to 96X gain
	//---------------------------------------------
	// If intrStatus = false(0), INTR in inactive (default)
	// If intrStatus = true(1), INTR in active
	//---------------------------------------------
	// If dataStatus = false(0), OLD data (already read) (default)
	// If dataStatus = true(1), NEW data
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
	
	byte status = 0x00;
	
	// Reading the status byte
	if(readByte(LTR303_STATUS, status)) {
		// Extract validity
		valid = (status & 0x80) ? true : false;
	
		// Extract gain
		gain = (status & 0x70) >> 4;
	
		// Extract interrupt status
		intrStatus = (status & 0x08) ? true : false;
	
		// Extract data status
		dataStatus = (status & 0x04) ? true : false;
		
		// return if successful
		return(true);
	}
	return(false);
}

boolean setInterruptControl(boolean polarity, boolean intrMode) {
	// Sets up interrupt operations
	// Default value is 0x08
	// If polarity = false(0), INT pin is active at logic 0 (default)
	// If polarity = true(1), INT pin is active at logic 1
	//------------------------------------------------------
	// If intrMode = false(0), INT pin is inactive (default)
	// If intrMode = true(1), INT pin is active
	//------------------------------------------------------
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
	
	byte intrControl = 0x00;
	
	intrControl |= polarity << 2;
	intrControl |= intrMode << 1;
	
	return(writeByte(LTR303_INTERRUPT, intrControl));
}

boolean getInterruptControl(boolean polarity, boolean intrMode) {
	// Sets up interrupt operations
	// Default value is 0x08
	// If polarity = false(0), INT pin is active at logic 0 (default)
	// If polarity = true(1), INT pin is active at logic 1
	//------------------------------------------------------
	// If intrMode = false(0), INT pin is inactive (default)
	// If intrMode = true(1), INT pin is active
	//------------------------------------------------------
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
	
	byte intrControl = 0x00;
	
	// Reading the interrupt byte
	if(readByte(LTR303_INTERRUPT, intrControl)) {
		// Extract polarity
		polarity = (intrControl & 0x04) ? true : false;
	
		// Extract mode
		intrMode = (intrControl & 0x02) ? true : false;
	
		// return if successful
		return(true);
	}
	return(false);
}

boolean setThreshold(unsigned int upperLimit, unsigned int lowerLimit) {
	// Sets the upper limit and lower limit of the threshold
	// Default value of upper threshold is 0xFF and lower threshold is 0x00
	// Both the threshold are 16-bit integer values
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
	
	byte threshold = 0x00;
	
	return(writeUInt(LTR303_THRES_UP_0,UpperLimit) && writeUInt(LTR303_THRES_LOW_0,lowerLimit));
}

boolean getThreshold(unsigned int &upperLimit, unsigned int &lowerLimit) {
	// Gets the upper limit and lower limit of the threshold
	// Default value of upper threshold is 0xFF and lower threshold is 0x00
	// Both the threshold are 16-bit integer values
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
			
	return(readUInt(LTR303_THRES_UP_0,upperLimit) && readUInt(LTR303_THRES_LOW_0,lowerLimit));		
}

boolean setIntrPersist(byte persist) {
	// Sets the interrupt persistance i.e. controls the N number of times the 
	// measurement data is outside the range defined by upper and lower threshold
	// Default value is 0x00
	// If persist = 0, every sensor value out of threshold range (default)
	// If persist = 1, every 2 consecutive value out of threshold range
	// If persist = 2, every 3 consecutive value out of threshold range
	// If persist = 3, every 4 consecutive value out of threshold range
	// If persist = 4, every 5 consecutive value out of threshold range
	// If persist = 5, every 6 consecutive value out of threshold range
	// If persist = 6, every 7 consecutive value out of threshold range
	// If persist = 7, every 8 consecutive value out of threshold range
	// If persist = 8, every 9 consecutive value out of threshold range
	// If persist = 9, every 10 consecutive value out of threshold range
	// If persist = 10, every 11 consecutive value out of threshold range
	// If persist = 11, every 12 consecutive value out of threshold range
	// If persist = 12, every 13 consecutive value out of threshold range
	// If persist = 13, every 14 consecutive value out of threshold range
	// If persist = 14, every 15 consecutive value out of threshold range
	// If persist = 15, every 16 consecutive value out of threshold range
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
	
	// sanity check
	if(persist => 15)
		persist = 0x00;
	
	return(writeByte(LTR303_INTR_PERS,persist));
}




boolean LTR303::setTiming(boolean gain, unsigned char time, unsigned int &ms)
	// If gain = false (0), device is set to low gain (1X)
	// If gain = high (1), device is set to high gain (16X)
	// If time = 0, integration will be 13.7ms
	// If time = 1, integration will be 101ms
	// If time = 2, integration will be 402ms
	// If time = 3, use manual start / stop (ms = 0)
	// ms will be set to integration time
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
{
	// Calculate ms for user
	switch (time)
	{
		case 0: ms = 14; break;
		case 1: ms = 101; break;
		case 2: ms = 402; break;
		default: ms = 0;
	}
	// Set integration using base function
	return(setTiming(gain,time));
}


boolean LTR303::manualStart(void)
	// Starts a manual integration period
	// After running this command, you must manually stop integration with manualStop()
	// Internally sets integration time to 3 for manual integration (gain is unchanged)
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
{
	unsigned char timing;
	
	// Get timing byte
	if (readByte(LTR303_REG_TIMING,timing))
	{
		// Set integration time to 3 (manual integration)
		timing |= 0x03;

		if (writeByte(LTR303_REG_TIMING,timing))
		{
			// Begin manual integration
			timing |= 0x08;

			// Write modified timing byte back to device
			if (writeByte(LTR303_REG_TIMING,timing))
				return(true);
		}
	}
	return(false);
}


boolean LTR303::manualStop(void)
	// Stops a manual integration period
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
{
	unsigned char timing;
	
	// Get timing byte
	if (readByte(LTR303_REG_TIMING,timing))
	{
		// Stop manual integration
		timing &= ~0x08;

		// Write modified timing byte back to device
		if (writeByte(LTR303_REG_TIMING,timing))
			return(true);
	}
	return(false);
}


boolean LTR303::getData(unsigned int &data0, unsigned int &data1)
	// Retrieve raw integration results
	// data0 and data1 will be set to integration results
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
{
	// Get data0 and data1 out of result registers
	if (readUInt(LTR303_REG_DATA_0,data0) && readUInt(LTR303_REG_DATA_1,data1)) 
		return(true);

	return(false);
}


boolean LTR303::getLux(unsigned char gain, unsigned int ms, unsigned int CH0, unsigned int CH1, double &lux)
	// Convert raw data to lux
	// gain: 0 (1X) or 1 (16X), see setTiming()
	// ms: integration time in ms, from setTiming() or from manual integration
	// CH0, CH1: results from getData()
	// lux will be set to resulting lux calculation
	// returns true (1) if calculation was successful
	// RETURNS false (0) AND lux = 0.0 IF EITHER SENSOR WAS SATURATED (0XFFFF)
{
	double ratio, d0, d1;

	// Determine if either sensor saturated (0xFFFF)
	// If so, abandon ship (calculation will not be accurate)
	if ((CH0 == 0xFFFF) || (CH1 == 0xFFFF))
	{
		lux = 0.0;
		return(false);
	}

	// Convert from unsigned integer to floating point
	d0 = CH0; d1 = CH1;

	// We will need the ratio for subsequent calculations
	ratio = d1 / d0;

	// Normalize for integration time
	d0 *= (402.0/ms);
	d1 *= (402.0/ms);

	// Normalize for gain
	if (!gain)
	{
		d0 *= 16;
		d1 *= 16;
	}

	// Determine lux per datasheet equations:
	
	if (ratio < 0.5)
	{
		lux = 0.0304 * d0 - 0.062 * d0 * pow(ratio,1.4);
		return(true);
	}

	if (ratio < 0.61)
	{
		lux = 0.0224 * d0 - 0.031 * d1;
		return(true);
	}

	if (ratio < 0.80)
	{
		lux = 0.0128 * d0 - 0.0153 * d1;
		return(true);
	}

	if (ratio < 1.30)
	{
		lux = 0.00146 * d0 - 0.00112 * d1;
		return(true);
	}

	// if (ratio > 1.30)
	lux = 0.0;
	return(true);
}


boolean LTR303::setInterruptControl(unsigned char control, unsigned char persist)
	// Sets up interrupt operations
	// If control = 0, interrupt output disabled
	// If control = 1, use level interrupt, see setInterruptThreshold()
	// If persist = 0, every integration cycle generates an interrupt
	// If persist = 1, any value outside of threshold generates an interrupt
	// If persist = 2 to 15, value must be outside of threshold for 2 to 15 integration cycles
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
{
	// Place control and persist bits into proper location in interrupt control register
	if (writeByte(LTR303_REG_INTCTL,((control | 0B00000011) << 4) & (persist | 0B00001111)))
		return(true);
		
	return(false);
}


boolean LTR303::setInterruptThreshold(unsigned int low, unsigned int high)
	// Set interrupt thresholds (channel 0 only)
	// low, high: 16-bit threshold values
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
{
	// Write low and high threshold values
	if (writeUInt(LTR303_REG_THRESH_L,low) && writeUInt(LTR303_REG_THRESH_H,high))
		return(true);
		
	return(false);
}


boolean LTR303::clearInterrupt(void)
	// Clears an active interrupt
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
{
	// Set up command byte for interrupt clear
	Wire.beginTransmission(_i2c_address);
	Wire.write(LTR303_CMD_CLEAR);
	_error = Wire.endTransmission();
	if (_error == 0)
		return(true);

	return(false);
}


boolean LTR303::getID(unsigned char &ID)
	// Retrieves part and revision code from LTR303
	// Sets ID to part ID (see datasheet)
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
{
	// Get ID byte from ID register
	if (readByte(LTR303_REG_ID,ID))
		return(true);

	return(false);
}


byte LTR303::getError(void)
	// If any library command fails, you can retrieve an extended
	// error code using this command. Errors are from the wire library: 
	// 0 = Success
	// 1 = Data too long to fit in transmit buffer
	// 2 = Received NACK on transmit of address
	// 3 = Received NACK on transmit of data
	// 4 = Other error
{
	return(_error);
}

// Private functions:

boolean LTR303::readByte(unsigned char address, unsigned char &value)
	// Reads a byte from a LTR303 address
	// Address: LTR303 address (0 to 15)
	// Value will be set to stored byte
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() above)
{
	// Set up command byte for read
	Wire.beginTransmission(_i2c_address);
	Wire.write((address & 0x0F) | LTR303_CMD);
	_error = Wire.endTransmission();

	// Read requested byte
	if (_error == 0)
	{
		Wire.requestFrom(_i2c_address,1);
		if (Wire.available() == 1)
		{
			value = Wire.read();
			return(true);
		}
	}
	return(false);
}


boolean LTR303::writeByte(unsigned char address, unsigned char value)
	// Write a byte to a LTR303 address
	// Address: LTR303 address (0 to 15)
	// Value: byte to write to address
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() above)
{
	// Set up command byte for write
	Wire.beginTransmission(_i2c_address);
	Wire.write((address & 0x0F) | LTR303_CMD);
	// Write byte
	Wire.write(value);
	_error = Wire.endTransmission();
	if (_error == 0)
		return(true);

	return(false);
}


boolean LTR303::readUInt(unsigned char address, unsigned int &value)
	// Reads an unsigned integer (16 bits) from a LTR303 address (low byte first)
	// Address: LTR303 address (0 to 15), low byte first
	// Value will be set to stored unsigned integer
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() above)
{
	char high, low;
	
	// Set up command byte for read
	Wire.beginTransmission(_i2c_address);
	Wire.write((address & 0x0F) | LTR303_CMD);
	_error = Wire.endTransmission();

	// Read two bytes (low and high)
	if (_error == 0)
	{
		Wire.requestFrom(_i2c_address,2);
		if (Wire.available() == 2)
		{
			low = Wire.read();
			high = Wire.read();
			// Combine bytes into unsigned int
			value = word(high,low);
			return(true);
		}
	}	
	return(false);
}


boolean LTR303::writeUInt(unsigned char address, unsigned int value)
	// Write an unsigned integer (16 bits) to a LTR303 address (low byte first)
	// Address: LTR303 address (0 to 15), low byte first
	// Value: unsigned int to write to address
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() above)
{
	// Split int into lower and upper bytes, write each byte
	if (writeByte(address,lowByte(value)) 
		&& writeByte(address + 1,highByte(value)))
		return(true);

	return(false);
}
