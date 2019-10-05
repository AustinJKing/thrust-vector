#include <Wire.h>
#include <Servo.h>
#include <RF24.h>
#include <SPI.h>
#include <Adafruit_MPL3115A2.h>
#include <Math.h>
#include <MPU6050_tockn.h>
#include <nRF24L01.h>
#define CE_PIN   9
#define CSN_PIN 10

// Radio Constants
const uint64_t pipe = 0xE8E8F0F0E1LL; // Define the transmit pipe

RF24 radio(CE_PIN, CSN_PIN); // Create a Radio

// Servo Constants
Servo xservo; // x-axis Servo
Servo yservo; // y-axis Servo

Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2(); // Instance of Adafruit_MPL3115A2
MPU6050 mpu6050(Wire);

void setup() {

	pinMode(3, OUTPUT); // Green LED
	pinMode(4, OUTPUT); // Yellow LED
	pinMode(7, OUTPUT); // Red LED
	pinMode(8, OUTPUT); // Buzzer

	xservo.attach(5); // Servo on D5
	yservo.attach(6); // Servo on D6

	// Temporarily sets leds
	digitalWrite(3, LOW);
	digitalWrite(4, HIGH);
	digitalWrite(7, LOW);

	// 3 Beeps on startup 
	for (int i = 0; i <= 2; i++)
	{
		tone(8, 900);
		delay(100);
		noTone(8);
		delay(100);
	}

	/*if (! baro.begin())
	  {
		  Serial.println("Couldnt find baro sensor");
	  while (1);
	  }
			else {
				Serial.println("Adafruit_MPL3115A2 Connected!");
				 }
  */


	Serial.begin(9600);
	radio.begin(); // Starts Radio
	Wire.begin(); // Starts Wire
	mpu6050.begin(); // Start MPU
	radio.setDataRate(RF24_250KBPS);
	radio.openWritingPipe(pipe);

	mpu6050.calcGyroOffsets(true); // Starts Offset Calc

	digitalWrite(4, LOW); // Standy bye light off after calibration and NRF startup
	digitalWrite(3, HIGH); //Green light on 

	for (int i = 0; i <= 1; i++)
	{
		tone(8, 900);
		delay(100);
		noTone(8);
		delay(100);
	}

}

void loop() {

	mpu6050.update();

	// Altimeter
	//double altm = baro.getAltitude();
	//float tempC = baro.getTemperature();
	//float pascals = baro.getPressure();

	// Sets variables for angle fetch
	float angleY = mpu6050.getAngleY();
	float angleX = mpu6050.getAngleX();

	Serial.print("  angleY: "); Serial.print(angleY);
	Serial.print("  angleX: "); Serial.println(angleX);

	// Writes to Radio
	radio.write(&angleX, sizeof(angleX));
	radio.write(&angleY, sizeof(angleY));

	//Serial.print("  Alt: "); Serial.print(altm); Serial.print(" Meters \t");
	//Serial.print("  Temp: "); Serial.print(tempC); Serial.println(" C \t");

	/* Writes the angle to the servo motors. Servos dont recognize negative angles so I use 90 degrees as the zeroed angle*/
	xservo.write(90 + mpu6050.getAngleX());
	yservo.write(90 + mpu6050.getAngleY());

	delay(20);
}
