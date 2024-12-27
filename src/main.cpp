#include <SD.h>
#include <FS.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>

#define I2C_SCL 22
#define I2C_SDA 21
#define SPI_CS 5
#define SPI_SCK 18
#define SPI_MOSI 19
#define SPI_MISO 23

void setupBMP();
void setupMPU();
void setupSD();
void getBMP();
void getMPU();
void calibrateMPU();
void writeFile(const char*, const char*);
void appendFile(const char*, const char*);
void readFile(const char*);

Adafruit_BMP280 bmp; // I2C
Adafruit_MPU6050 mpu; // SPI

// pressure info: https://www.localconditions.com/weather-richardson-texas/75080/
const double localAirPressure = 30.08 *33.863889532611; // inHg --> hectopascals (hPa)

// global variables
float accX, accY, accZ, gyrX, gyrY, gyrZ;
float accXCalibration, accYCalibration, accZCalibration, 
      gyrXCalibration, gyrYCalibration, gyrZCalibration;

double pressure, altitude, temperature;

File dataFile;
String filePathS = "/log-" + String(esp_random()) + ".csv"; // random number so we dont overwrite previous recordings, could change to a date + timestamp
const char* filePath = filePathS.c_str();
bool printToTerminal;

void setup() {
   printToTerminal = false;
   Serial.begin(115200);
   while (!Serial) delay(100); // wait for native usb

   Wire.begin(I2C_SDA, I2C_SCL); // establish I2C connection
   setupBMP();
   setupMPU();
   calibrateMPU();
   setupSD();

   writeFile(filePath, "timestamp, pressure, altitude, temperature, accX, accY, accZ, gyrX, gyrY, gyrZ"); //Readings are in CSV string
}

/*
loop mechanics
refresh sensor readings
add readings and timestamp
*/
String buffer = "";
const int bufferSize = 50; // upload after 50 data recordings
int totalRecords = 0;
long int lastLogTime = 0;

void loop() {
   unsigned long currentTime = millis();
   
   for(int i=0; i<bufferSize; i++) {
   currentTime = millis();
   getBMP();
   getMPU();
   // Add all data to buffer to only append every 100ms.
   // const char* max len = 32672
   buffer +=   String(currentTime) + "," +
               String(pressure) + "," + String(altitude) + "," + String(temperature) + "," +
               String(accX, 2) + "," + String(accY) + "," + String(accZ, 2) + "," +
               String(gyrX, 2) + "," + String(gyrY, 2) + "," + String(gyrZ, 2) + "\n";

   /* Once the buffer has been filled, append it to log, and reset the buffer*/
   if(i == bufferSize-1) {
      appendFile(filePath, buffer.c_str());
      buffer = "";
      lastLogTime = millis();
   }
   totalRecords+=1;
   delay(20);
   
   }
   Serial.println(totalRecords);
   
   if(totalRecords > 1999) {
      while(1) {delay(100);} // FREEZE THE PROGRAM AFTER 20000 recordings
   }
}

void setupBMP() {
   Serial.println(F("BMP280 test"));
   //unsigned status;
   bool status = bmp.begin(0x76);
   if (!status) {
      Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
      Serial.print("SensorID was: 0x");
      Serial.println(bmp.sensorID(), 16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      while (1) delay(10); // If SD card isn't started correctly, the program will stall here
   }

   /* Default settings from datasheet. 
   bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, 
      Adafruit_BMP280::SAMPLING_X2,
      Adafruit_BMP280::SAMPLING_X16,
      Adafruit_BMP280::FILTER_X16, 
      Adafruit_BMP280::STANDBY_MS_500); 
   */

   bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, /* Operating Mode. */
      Adafruit_BMP280::SAMPLING_X2, /* Temp. oversampling */
      Adafruit_BMP280::SAMPLING_X4, /* Pressure oversampling */
      Adafruit_BMP280::FILTER_X4, /* Filtering. */
      Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
   

}
void setupMPU() {
   bool status = mpu.begin(0x68);
   if(!status) {
      Serial.println("MPU failed to start");
   }
   mpu.setAccelerometerRange(MPU6050_RANGE_16_G); // +/- 16,8,4,2 in g's
   mpu.setFilterBandwidth(MPU6050_BAND_10_HZ); // low pass filter: 184, 94, 44, 21, 10 5 in Hz
   mpu.setGyroRange(MPU6050_RANGE_250_DEG); // 2000, 1000, 500, 250 in deg/sec
   mpu.setCycleRate(MPU6050_CYCLE_5_HZ); //periodic measurement: 1.25, 5, 20, 40 in Hz
}
void setupSD() {
   SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
   bool status = SD.begin(SPI_CS, SPI);
   if(!status) {
      Serial.println("SD Failed to start");
   }

   uint8_t cardType = SD.cardType();
   if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return;
  }
  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC) {
      Serial.println("MMC");
   } else if(cardType == CARD_SD){
      Serial.println("SDSC");
   } else if(cardType == CARD_SDHC){
      Serial.println("SDHC");
   } else {
      Serial.println("UNKNOWN");
   }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}
void calibrateMPU() {
   accXCalibration = -0.8498;
   accYCalibration = 0.1022;
   accZCalibration = -0.9548;
   gyrXCalibration = -0.02;
   gyrYCalibration = -0.02;
   gyrZCalibration = -0.06;
}
// gets the current BMP sensor values and updates the global variable values
void getBMP() {
   temperature = bmp.readTemperature();
   pressure = bmp.readPressure();
   altitude = bmp.readAltitude(localAirPressure);

   if(printToTerminal) {
      Serial.println("Temperature = " + String(temperature) + " *C");
      Serial.println("Pressure = " + String(pressure) + " Pa");
      Serial.println("Approx altitude = " + String(altitude) + " m");
   }
}

// gets the current MPU sensor values and updates the global variable values
void getMPU() {
   sensors_event_t a, g, temp;
   mpu.getEvent(&a, &g, &temp);
   accX = a.acceleration.x + accXCalibration;
   accY = a.acceleration.y + accYCalibration;
   accZ = a.acceleration.z + accZCalibration;
   gyrX = g.gyro.x + gyrXCalibration;
   gyrY = g.gyro.y + gyrYCalibration;
   gyrZ = g.gyro.z + gyrZCalibration;

   if(printToTerminal) {
      Serial.print("Acceleration X: ");
      Serial.print(a.acceleration.x);
      Serial.print(", Y: ");
      Serial.print(a.acceleration.y);
      Serial.print(", Z: ");
      Serial.print(a.acceleration.z);
      Serial.println(" m/s^2");

      Serial.print("Rotation X: ");
      Serial.print(g.gyro.x);
      Serial.print(", Y: ");
      Serial.print(g.gyro.y);
      Serial.print(", Z: ");
      Serial.print(g.gyro.z);
      Serial.println(" rad/s");

      Serial.print("Temperature: ");
      Serial.print(temp.temperature);
      Serial.println(" degC");
   }
}

void writeFile(const char* path, const char* data) {
   dataFile = SD.open(path, "w");
   if(dataFile) {
      //dataFile.println("testing writing!");
      dataFile.println(data);
      dataFile.close();
   }
   else {
      Serial.println(String(path) + ": could not be written to.");
   }
   
}

void appendFile(const char* path, const char* data) {
   dataFile = SD.open(path, "a");
   if(dataFile) {
      //dataFile.println("testing writing!");
      dataFile.println(data);
      dataFile.close();
   }
   else {
      Serial.println(String(path) + ": could not be appended to.");
   }
}

void readFile(const char* path) {
   dataFile = SD.open(path, "r");
   if(dataFile) {
      while (dataFile.available()) {
         Serial.write(dataFile.read());
      }
      dataFile.close();
   } else {
      Serial.println("Error, could not read file");
   }
}