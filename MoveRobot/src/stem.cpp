/*
  stem.cpp - Source file of the library to control the USAL robot.
  Created by Bilal A. Komati, December 15, 2020.
  Released for Al Mabarrat Schools.
*/

#include "stem.h"

// Outside of class
Robot *pointerToClass; // declare a pointer to Robot class

static void outsideInterruptHandlerEncoderA1(void) { // define global handler
  pointerToClass->classInterruptHandlerEncoderA1(); // call class member handler
}

static void outsideInterruptHandlerEncoderA2(void) { // define global handler
  pointerToClass->classInterruptHandlerEncoderA2(); // call class member handler
}

static void outsideInterruptHandlerEncoderB1(void) { // define global handler
  pointerToClass->classInterruptHandlerEncoderB1(); // call class member handler
}

static void outsideInterruptHandlerEncoderB2(void) { // define global handler
  pointerToClass->classInterruptHandlerEncoderB2(); // call class member handler
}

const unsigned int LED_BUILTIN_1 = 4;
const unsigned int LED_BUILTIN_2 = 45;
const unsigned int LED_BUILTIN_3 = 44;
const unsigned int LED_BUILTIN_4 = 13;

// Class members
Robot::Robot() : _lcd(new LiquidCrystal_I2C(0x27,16,2)), _servo(new Servo())
{
  pinMode(Ain1, OUTPUT);
  pinMode(Ain2, OUTPUT);
  pinMode(Bin1, OUTPUT);
  pinMode(Bin2, OUTPUT);
  pinMode(touchSensorPin,INPUT);
  pinMode(lineDetectorLeftPin,INPUT);
  pinMode(lineDetectorRightPin,INPUT);
  pinMode(ultrasonicTrigPin, OUTPUT);
  pinMode(ultrasonicEchoPin,INPUT);
  pinMode(LED_BUILTIN_1,OUTPUT);
  pinMode(LED_BUILTIN_2,OUTPUT);
  pinMode(LED_BUILTIN_3,OUTPUT);
  pinMode(LED_BUILTIN_4,OUTPUT);
  _startTargetAngle = false;
  _motorLisRunning = _motorRisRunning = false;
  _encoderR = _encoderL = 0;
  pinMode(EncAC1,INPUT);
  pinMode(EncAC2,INPUT);
  pinMode(EncBC1,INPUT);
  pinMode(EncBC2,INPUT);
  pointerToClass = this; // assign current instance to pointer (IMPORTANT!!!)
  attachInterrupt(digitalPinToInterrupt(EncAC1), outsideInterruptHandlerEncoderA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncAC2), outsideInterruptHandlerEncoderA2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncBC1), outsideInterruptHandlerEncoderB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncBC2), outsideInterruptHandlerEncoderB2, CHANGE);
}

void Robot::begin (void) // To start LCD and Servo Functions. These functions should not be called in the constructor
{
  _servo->attach(servoPin);
  _servo->write(0);
  _lcd->init();
  _lcd->backlight();
  _lcd->clear();
}

void Robot::displayLCD(uint8_t row, uint8_t column, const char msg[])
{
	_lcd->setCursor(column, row);
	_lcd->print(msg);
}

void Robot::displayLCD(uint8_t row, uint8_t column, const long msg)
{
	_lcd->setCursor(column, row);
	_lcd->print(msg);
}

void Robot::displayLCD(uint8_t row, uint8_t column, const double msg)
{
	_lcd->setCursor(column, row);
	_lcd->print(msg);
}

void Robot::clearLCD(void)
{
	_lcd->clear();
}

void Robot::moveMotor (motorDirection direction, int power)
{
	switch (direction)
	{
		case motorR :
			_motorRisRunning = true;
			if (power > 100)
	  			power = 100;
			else if (power < -100)
	  			power = -100;
			if (power > 0)
			{
				_pwmOutput = 240 - power;
				digitalWrite(Ain2, HIGH);
				analogWrite(Ain1, _pwmOutput);
			}
			else if (power < 0)
			{
				_pwmOutput = 240 + power;
				digitalWrite(Ain1, HIGH);
				analogWrite(Ain2, _pwmOutput);
			}
			else
			{
				digitalWrite(Ain1, HIGH);
				digitalWrite(Ain2, HIGH);
			}
			break;
		case motorL:
			_motorLisRunning = true;
			if (power > 100)
	  			power = 100;
			else if (power < -100)
	  			power = -100;
			if (power > 0)
			{
				_pwmOutput = 240 - power;
				digitalWrite(Bin1, HIGH);
				analogWrite(Bin2, _pwmOutput);
			}
			else if (power < 0)
			{
				_pwmOutput = 240 + power;
				digitalWrite(Bin2, HIGH);
				analogWrite(Bin1, _pwmOutput);
			}
			else
			{
				digitalWrite(Bin1, HIGH);
				digitalWrite(Bin2, HIGH);
			}
			break;
		case motorP:
			if (power > 100)
				power = 100;
			else if (power < 0)
				power = 0;
			_servo->write(power);
			break;
		default :
			_motorLisRunning = _motorRisRunning = false;
			_pwmOutput = 0;
			digitalWrite(Ain1, HIGH);
			digitalWrite(Ain2, HIGH);
			digitalWrite(Bin1, HIGH);
			digitalWrite(Bin2, HIGH);
    }
}

void Robot::setEncoderTarget (motorDirection direction, int targetAngle)
{
	_targetAngle = abs(targetAngle);
	_startTargetAngle = true;
}

void Robot::wait (int time)
{
	if(!_startTargetAngle)
		delay(time);
	else
	{
		if (_motorRisRunning)
		{
			while (abs(getMotorEncoder(motorR)) < _targetAngle)
			{ /* Loop to wait until the movement is finished */	}
		}
		else if (_motorLisRunning)
		{
			while (abs(getMotorEncoder(motorL)) < _targetAngle)
			{ /* Loop to wait until the movement is finished */ }
		}
		moveMotor(stopMotors,0);
		_startTargetAngle = false;
	}
}

double Robot::getMotorEncoder (motorDirection direction)
{
	switch (direction)
	{
		case motorL:
			_angleL = 0.25714 * _encoderL;
			return _angleL;
			break;
		case motorR:
			_angleR = 0.25714 * _encoderR;
			return _angleR;
			break;
		default: return 0 ;
	}
}

void Robot::resetMotorEncoder (motorDirection direction)
{
	switch (direction)
	{
		case motorL:
			_encoderL = 0;
			break;
		case motorR:
			_encoderR = 0;
			break;
		default:
			_encoderL = _encoderR = 0 ;
	}
}

int Robot::getSensorValue(sensorName sensor)
{
	switch (sensor)
	{
		case touchSensor:
			return(digitalRead(touchSensorPin));
		case lineDetectorLeft:
			return(digitalRead(lineDetectorLeftPin));
		case lineDetectorRight:
			return(digitalRead(lineDetectorRightPin));
		case ultrasonic:
			int duration, distance;
			digitalWrite(ultrasonicTrigPin, LOW);
			delayMicroseconds(2);
			digitalWrite(ultrasonicTrigPin, HIGH);
			delayMicroseconds(10);
			digitalWrite(ultrasonicTrigPin, LOW);
			duration = pulseIn(ultrasonicEchoPin, HIGH);
			distance = duration*0.034/2;
			return distance;
		default:
			return -1;
	}
}

void Robot::classInterruptHandlerEncoderA1 (void)
{
  // look for a low-to-high on channel C1
  if ( digitalRead(EncAC1) )
  {
    // check channel C2 to see which way encoder is turning
    if ( digitalRead(EncAC2) )
      _encoderR++;       // CW
    else 
      _encoderR--;       // CCW
  }

  else // must be a high-to-low edge on channel C1
  {
    // check channel C2 to see which way encoder is turning
    if ( !digitalRead(EncAC2) )
      _encoderR++;       // CW
    else 
      _encoderR--;       // CCW
  }
}

void Robot::classInterruptHandlerEncoderA2 (void)
{
  // look for a low-to-high on channel C2
  if ( digitalRead(EncAC2) )
  {
    // check channel C1 to see which way encoder is turning
    if ( !digitalRead(EncAC1) )
      _encoderR++;       // CW
    else 
      _encoderR--;       // CCW
  }
  
  else // must be a high-to-low edge on channel C1
  {
    // check channel C2 to see which way encoder is turning
    if ( digitalRead(EncAC1) )
      _encoderR++;       // CW
    else 
      _encoderR--;       // CCW
  }
}

void Robot::classInterruptHandlerEncoderB1 (void)
{
  // look for a low-to-high on channel C1
  if ( digitalRead(EncBC1) )
  {
    // check channel C2 to see which way encoder is turning
    if ( !digitalRead(EncBC2) )
      _encoderL++;       // CW
    else 
      _encoderL--;       // CCW
  }
  
  else // must be a high-to-low edge on channel C1
  {
    // check channel C2 to see which way encoder is turning
    if ( digitalRead(EncBC2) )
      _encoderL++;       // CW
    else 
      _encoderL--;       // CCW
  }
}

void Robot::classInterruptHandlerEncoderB2 (void)
{
  // look for a low-to-high on channel C2
  if ( digitalRead(EncBC2) )
  {
    // check channel C1 to see which way encoder is turning
    if ( digitalRead(EncBC1) )
      _encoderL++;       // CW
    else 
      _encoderL--;       // CCW
  }
  
  else // must be a high-to-low edge on channel C1
  {
    // check channel C2 to see which way encoder is turning
    if ( !digitalRead(EncBC1) )
      _encoderL++;       // CW
    else 
      _encoderL--;       // CCW
  }
}