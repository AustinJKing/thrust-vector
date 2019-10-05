#include "MPU6050_tockn.h"
#include "Arduino.h"

MPU6050::MPU6050(TwoWire &w){
  wire = &w;
  accCoef = 0.02f; // Percentage for filter
  gyroCoef = 0.98f; // Percentage for filter
}

MPU6050::MPU6050(TwoWire &w, float aC, float gC){
  wire = &w;
  accCoef = aC;
  gyroCoef = gC;
}

void MPU6050::begin(){
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);
  writeMPU6050(MPU6050_CONFIG, 0x00);
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00);
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);
  this->update();
  angleGyroX = 0; 
  angleGyroY = 0;
  angleX = this->getAccAngleX(); // Allows call for angle x in Vector_Control.c
  angleY = this->getAccAngleY(); // Allows call for angle y in Vector_Control.c
  preInterval = millis(); // Millisecond time interval
}

void MPU6050::writeMPU6050(byte reg, byte data){
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(reg);
  wire->write(data);
  wire->endTransmission();
}

byte MPU6050::readMPU6050(byte reg) {
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(reg);
  wire->endTransmission(true);
  wire->requestFrom(MPU6050_ADDR, 1);
  byte data =  wire->read();
  return data;
}

void MPU6050::setGyroOffsets(float x, float y, float z){ // Sets variables for gyro offsets (deviations from x=0, y=0, z=0)
  gyroXoffset = x; 
  gyroYoffset = y;
  gyroZoffset = z;
}

void MPU6050::calcGyroOffsets(bool console, uint16_t delayBefore, uint16_t delayAfter){
	float x = 0, y = 0, z = 0;
	int16_t rx, ry, rz;

  delay(delayBefore); // Tells user (Through serial monitor) not to move while calulating 
	if(console){
    Serial.println();
    Serial.println("========================================"); 
    Serial.println("Calculating gyro offsets");
    Serial.print("DO NOT MOVE MPU6050");
  }

/*This for loop iterates 3000 times to get an average value for the gyro offsets */
  for(int i = 0; i < 3000; i++){   
    if(console && i % 1000 == 0){
      Serial.print(".");
    }
    wire->beginTransmission(MPU6050_ADDR); //Begins connection
    wire->write(0x43);
    wire->endTransmission(false);
    wire->requestFrom((int)MPU6050_ADDR, 6); 

    rx = wire->read() << 8 | wire->read(); //Grabs raw data from gyro 
    ry = wire->read() << 8 | wire->read(); //Grabs raw data from gyro 
    rz = wire->read() << 8 | wire->read(); //Grabs raw data from gyro 

/* This converts raw data into degrees/s (angulate rate). I chose a reading rate of 500 degrees/s which has an associated sensitivity 
factor of 65.5 (LSB)(degrees/s). Dividing the raw data by the sensitivity factor will give you the agular rate (degrees/s).
The reading rate and sensitivity factor values are found in the mpu6050 datasheet on page 12.*/
    x += ((float)rx) / 65.5;  // The angles are added up to be divided for the average value in the next step
    y += ((float)ry) / 65.5;
    z += ((float)rz) / 65.5;
  }
/* The sum of the angles are divided by the total number of readings (3000) to get the average gyroOffset*/
  gyroXoffset = x / 3000;  
  gyroYoffset = y / 3000;
  gyroZoffset = z / 3000;

/* This step prints the Offsets to the serial monitor and starts the program */
  if(console){
    Serial.println();
    Serial.println("Done!");
    Serial.print("X : ");Serial.println(gyroXoffset);
    Serial.print("Y : ");Serial.println(gyroYoffset);
    Serial.print("Z : ");Serial.println(gyroZoffset);
    Serial.println("Program will start after 3 seconds");
    Serial.print("========================================");
		delay(delayAfter);
	}
}

void MPU6050::update(){
	wire->beginTransmission(MPU6050_ADDR); // begins connection 
	wire->write(0x3B);
	wire->endTransmission(false);
	wire->requestFrom((int)MPU6050_ADDR, 14);

  /* This step aquires the raw data from the gryro and accelerometer */
  rawAccX = wire->read() << 8 | wire->read();
  rawAccY = wire->read() << 8 | wire->read();
  rawAccZ = wire->read() << 8 | wire->read();
  rawTemp = wire->read() << 8 | wire->read(); 
  rawGyroX = wire->read() << 8 | wire->read();
  rawGyroY = wire->read() << 8 | wire->read();
  rawGyroZ = wire->read() << 8 | wire->read();

/* The MPU6050 also has a temp sensor. I Have no idea why 12412 is added to the raw data (Bit correction?) 
but the raw data is then divided by the sensitivity factor 340 (LSB)degrees/C to get the temp in Celcius*/
  temp = (rawTemp + 12412.0) / 340.0; 

/* This converts raw data into g's (m/s^2). I chose a sensitivity of +-2g's which has an associated sensitivity factor of 16384 LSB/g.
Dividing the raw data by the sensitivity factor will give you the g value for that axis. The sensitivity and sensitivity factor values
are found in the mpu6050 datasheet on page 13.*/
  accX = ((float)rawAccX) / 16384.0;
  accY = ((float)rawAccY) / 16384.0;
  accZ = ((float)rawAccZ) / 16384.0;

/* The angle from the accelerometer can be calulated using trig in combination with an atan2 function. These equations are 
derived in equations 37 and 28 in "https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf" */
  angleAccX = atan2(accY, accZ + abs(accX)) * 360 / 2.0 / PI;  // 360/2/PI just converts from radians to degrees
  angleAccY = atan2(accX, accZ + abs(accY)) * 360 / -2.0 / PI;

  // Same step as the one in the gyro offset calculation
  gyroX = ((float)rawGyroX) / 65.5;
  gyroY = ((float)rawGyroY) / 65.5;
  gyroZ = ((float)rawGyroZ) / 65.5;

  // Takes the angulate rate and subtracts the offsets to get the true value 
  gyroX -= gyroXoffset; 
  gyroY -= gyroYoffset;
  gyroZ -= gyroZoffset;

/*  This step calulates the interval (dt) for the next step. Basically dt is the time it takes for one iteration/calualtion of the main
    loop */
  interval = (millis() - preInterval) * 0.001; 

/* Angular rate is degrees/s so by multiplying the agular rate by the time it took to caluate will result in just the angle.
  (degrees/s)*s = degrees*/ 
  angleGyroX += gyroX * interval; 
  angleGyroY += gyroY * interval;
  angleGyroZ += gyroZ * interval;

  /* Both an accelrometer and gyroscope have their cons so a filter needs to be applied to aquire a more accurate reading. 
  An accelerometer is great at calculating angles as long as the sensor is not being accelerated and a gryscope has error due to 
  integration (roundoff error) which is known as "drift". Combining data from both sensors through a filter will significantly increase
  accuracy. The following filter is a comlimentary filter and relies heavily on the gryroscope (98% of the final value) but is made 
  more true with the combination of the accelerometer (2% of the final value). I uploaded a graph I made in the drive that shows the 
  benefit of having a filter vs not having one. */
  angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
  angleY = (gyroCoef * (angleY + gyroY * interval)) + (accCoef * angleAccY);
  angleZ = angleGyroZ;

  preInterval = millis(); // updates the time it took for this whole loop to run

}
