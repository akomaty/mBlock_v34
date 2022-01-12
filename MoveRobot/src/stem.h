/*
  stem.h - Header file of the library to control the USAL robot.
  Created by Bilal A. Komati & Ali A. Komaty, December 15, 2020.
  Released for Al Mabarrat Schools.
*/

#ifndef stem_h
#define stem_h

#include "Arduino.h"
#include "LiquidCrystal_I2C.h"
#include "ServoRob.h"
#include <inttypes.h>

#define Ain1 10
#define Ain2 11
#define Bin1 46
#define Bin2 12
#define EncAC1 2
#define EncAC2 3
#define EncBC1 18
#define EncBC2 19
#define touchSensorPin 53
#define ultrasonicTrigPin 1
#define ultrasonicEchoPin 23
#define lineDetectorLeftPin 40
#define lineDetectorRightPin 41
#define servoPin 5

extern const unsigned int LED_BUILTIN_1;
extern const unsigned int LED_BUILTIN_2;
extern const unsigned int LED_BUILTIN_3;
extern const unsigned int LED_BUILTIN_4;

enum motorDirection{
	motorL,
	motorR,
	motorP,
	stopMotors
};

enum sensorName {
	touchSensor,
	ultrasonic,
	lineDetectorLeft,
	lineDetectorRight
};

static void outsideInterruptHandlerEncoderA1(void);
static void outsideInterruptHandlerEncoderA2(void);
static void outsideInterruptHandlerEncoderB1(void);
static void outsideInterruptHandlerEncoderB2(void);

class Robot
{
  public:
	Robot(void);
	void begin (void);
	void displayLCD(uint8_t, uint8_t, const char[]);
    void displayLCD(uint8_t, uint8_t, const long);
    void displayLCD(uint8_t, uint8_t, const double);
	void clearLCD(void);
	void moveMotor (motorDirection direction, int power);
	void setEncoderTarget (motorDirection direction, int targetAngle);
	double getMotorEncoder (motorDirection direction);
	void resetMotorEncoder (motorDirection direction);
	void wait (int = 0);
	int getSensorValue(sensorName);
	void classInterruptHandlerEncoderA1(void);
	void classInterruptHandlerEncoderA2(void);
	void classInterruptHandlerEncoderB1(void);
	void classInterruptHandlerEncoderB2(void);
	
  private:
  	LiquidCrystal_I2C* _lcd;
  	Servo* _servo;
	bool _startTargetAngle;
	bool _motorLisRunning;
	bool _motorRisRunning;
	volatile long _encoderL;
	volatile long _encoderR;
	double _angleL;
	double _angleR;
	int _pwmOutput;
	int _targetAngle;
};

#endif
