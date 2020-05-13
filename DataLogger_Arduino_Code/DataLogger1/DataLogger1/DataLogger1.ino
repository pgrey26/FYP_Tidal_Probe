// The original logging code based on the Adafruit data logging shield example.
// The sketch is set up for the Arduino Due (version 1) logger
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <TimeLib.h>
#include "RTClib.h"
#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

// A simple data logger for the Arduino analog pins

// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  100 // 10Hz mills between entries (reduce to take more/faster data)

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL 1000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

// The analog pins that connect to the sensors
#define Pin1 1       
#define Pin2 2
#define Pin3 3
#define Pin4 4
#define Pin5 5      

#define aref_voltage 3.3      
 
  RTC_PCF8523 rtc; // define the Real Time Clock object

// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;
// the logging file
File logfile;

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  while(1);
}

void setup(void)
{
   Serial.begin(9600);
  while(!Serial) {}
  Serial.println();
  
  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(chipSelect, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");
  
  // create a new file
  char filename[] = "LOGGER00.csv";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  
  if (! logfile) {
    error("couldnt create file");
  }
  
  Serial.print("Logging to: ");
  Serial.println(filename);
    Serial.print("Log Interval / Sync interval ");
  Serial.print(LOG_INTERVAL);
  Serial.print(" / ");
  Serial.println(SYNC_INTERVAL);

  // connect to RTC
  Wire.begin();  
 // Wire.setClock(400000);
//  if (! rtc.begin()) {
//    logfile.println("RTC failed");
//     while (1);
//    }
  
  rtc.begin();

  logfile.println("millis,datetime,Pin1,Pin2,Pin3,Pin4,Pin5,ax,ay,az,gx,gy,gz,mx,my,mz");    


analogReadResolution(12); // set 12 bit analog resolution

// start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
// setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-250 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  // setting DLPF bandwidth. options: 5HZ 10HZ 20HZ 41HZ 92HZ 184HZ
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_5HZ);
  // Internal sampling rate of IMU
  // setting SRD to 99 for a 10 Hz update rate, 49 FOR 20 Hz, 9 for 100 Hz
  //to avoid aliasing sampling/update rate must be double dlpf bandwidth
  IMU.setSrd(99);
}

void loop(void)
{

  DateTime now;
  // delay for the amount of time we want between readings  
  delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));
  
//  digitalWrite(greenLEDpin, HIGH);
  uint32_t a = micros(); 
  // log milliseconds since starting
  uint32_t m = millis();
  logfile.print(m);           // milliseconds since start
  logfile.print(", ");    


  // fetch the time
now = rtc.now();
  // log time
  logfile.print(now.day(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.year(), DEC); 
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);

  // Read Analogue Signals
  int Pin1Reading = analogRead(Pin1);  
  int Pin2Reading = analogRead(Pin2);  
  int Pin3Reading = analogRead(Pin3);  
  int Pin4Reading = analogRead(Pin4);  
  int Pin5Reading = analogRead(Pin5);  
 
  //Tell IMU to get reading, buffered to built in FIFO buffer on MPU9250
  IMU.readSensor();
  
 //Print Analogue readings to file
  logfile.print(", ");    
  logfile.print(Pin1Reading); 
  logfile.print(", ");    
  logfile.print(Pin2Reading); 
  logfile.print(", "); 
  logfile.print(Pin3Reading); 
  logfile.print(", "); 
  logfile.print(Pin4Reading); 
  logfile.print(", "); 
  logfile.print(Pin5Reading); 
  logfile.print(", "); 

 //Retrieve IMU readings from buffer and print to file, 6 decimal places specified
  logfile.print(IMU.getAccelX_mss(),6);
  logfile.print(", ");
  logfile.print(IMU.getAccelY_mss(),6);
  logfile.print(", ");
  logfile.print(IMU.getAccelZ_mss(),6);
  logfile.print(", ");
  logfile.print(IMU.getGyroX_rads(),6);
  logfile.print(", ");
  logfile.print(IMU.getGyroY_rads(),6);
  logfile.print(", ");
  logfile.print(IMU.getGyroZ_rads(),6);
  logfile.print(", ");
  logfile.print(IMU.getMagX_uT(),6);
  logfile.print(", ");
  logfile.print(IMU.getMagY_uT(),6);
  logfile.print(", ");
  logfile.println(IMU.getMagZ_uT(),6);
  //Optional: IMU has onboard temperature reading for compensation
  // logfile.println(IMU.getTemperature_C(),6);


  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL){ 
    return;
  }
  syncTime = millis(); 
  logfile.flush();
          
}
