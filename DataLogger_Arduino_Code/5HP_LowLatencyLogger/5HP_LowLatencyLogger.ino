// Low latency sketch for logging the five hole probe to the teensy 3.6 logger prototype.
// Currently configured for use with serial monitor interface. function for remote logging
// needs to be developed.
// Could be used with the original Due prototype with some minor changes. 
// Logging up to at least 500 Hz should be possible with the DUE, but it has not be tested 
// with this code.

#include <SPI.h>
#include "SdFat.h"
#include "FreeStack.h"
#include "Logger.h"
#include "Wire.h"
#include "MPU9250.h"
#include <TimeLib.h>
//------------------------------------------------------------------------------
// This example was designed for exFAT but will support FAT16/FAT32. 
// Can use the ExFatFormatter or SdFormatter examples in the SdFat library to format card
// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 3
//------------------------------------------------------------------------------
// Interval between data records in microseconds. Max 1000.
const uint32_t LOG_INTERVAL_USEC = 10000; //MicroSeconds: 1000 = 1kHz, 10000 = 100Hz etc.

// Set USE_RTC nonzero for file timestamps. Not setup yet
#if USE_RTC
#include "RTClib.h"
#endif  // USE_RTC

// LED to light if overruns occur.
#define ERROR_LED_PIN -1

MPU9250 IMU(Wire,0x68); //Define IMU object

//RTC_PCF8523 rtc; // define the Real Time Clock object, uncomment for DUE

//Define the analog read pins:
#define pin1 A14       
#define pin2 A15
#define pin3 A16
#define pin4 A17
#define pin5 A18  
#define aref_voltage 3.3


//Select SS for Teensy, 10 for DUE + data Logger shield
// SDCARD_SS_PIN is defined for the built-in SD on some boards.
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN
//#define SD_CONFIG SdioConfig(FIFO_SDIO)
// FIFO SIZE - 512 byte sectors.  Modify for your board.
#ifdef __AVR_ATmega328P__
// Use 512 bytes for 328 boards.
#define FIFO_SIZE_SECTORS 1
#elif defined(__AVR__)
// Use 2 KiB for other AVR boards.
#define FIFO_SIZE_SECTORS 4
#else  // __AVR_ATmega328P__
// Use 8 KiB for non-AVR boards.
#define FIFO_SIZE_SECTORS 16
#endif  // __AVR_ATmega328P__

// Preallocate file size 128MiB.
const uint32_t PREALLOCATE_SIZE_MiB = 128UL;

// Select the fastest interface. Assumes no other SPI devices.
#if ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI)
#else  // ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI)
#endif  // ENABLE_DEDICATED_SPI

// Save SRAM if 328.
#ifdef __AVR_ATmega328P__
#include "MinimumSerial.h"
MinimumSerial MinSerial;
#define Serial MinSerial
#endif  // __AVR_ATmega328P__
//==============================================================================
// set to use teensy 3.6 internal RTC
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}
 //==============================================================================  
// Function to retrieve a data record
void logRecord(data_t* data, uint16_t overrun) {
   if (overrun) {
    // Add one since this record has no adc data. Could add overrun field.
    overrun++;
    data->ax = 0X8000 | overrun;
  } else {
  
  data->time = micros();
  data-> day = day(); 
  data-> month = month();
  data-> year = year();
  data-> hour = hour();
  data-> minute = minute();
  data-> second = second();
  // uncomment for the Arduino Due logger:
  //  DateTime now = rtc.now();
  //  data-> day = now.day();
  //  data-> month =  now.month();
  //  data-> year =  now.year();
  //  data-> hour =  now.hour();
  //  data-> minute =  now.minute();
  //  data-> second = now.second();
  data-> Pin1 = analogRead(pin1); 
  data-> Pin2 = analogRead(pin2); 
  data-> Pin3 = analogRead(pin3); 
  data-> Pin4 = analogRead(pin4); 
  data-> Pin5 = analogRead(pin5); 
  IMU.readSensor();
  data->ax = IMU.getAccelX_mss();
  data->ay = IMU.getAccelY_mss();
  data->az = IMU.getAccelZ_mss();
  data->gx = IMU.getGyroX_rads();
  data->gy = IMU.getGyroY_rads();
  data->gz = IMU.getGyroZ_rads();
  data->mx = IMU.getMagX_uT();
  data->my = IMU.getMagY_uT();
  data->mz = IMU.getMagZ_uT();
  }
}
//------------------------------------------------------------------------------
void printRecord(Print* pr, data_t* data) {
  static uint32_t nr = 0;
 //Print header row at start. Change this string to change variable names:
 if (!data) {
   pr->println(F("micros,datetime,A1,A2,A3,A4,A5,ax,ay,az,gx,gy,gz,mx,my,mz"));
    nr = 0;
    return;
  }
    if (nr == 0) {
    nr = data->time;
  }
  pr->print(data->time- nr);
  pr->write(',');
  pr->print(data->day, DEC);
  pr->write('/');
  pr->print(data->month, DEC);
  pr->write('/');
  pr->print(data->year, DEC); 
  pr->write(' ');
  pr->print(data->hour, DEC);
  pr->write(':');
  pr->print(data->minute, DEC);
  pr->write(':');
  pr->print(data->second, DEC);
  pr->write(',');
  pr->print(data->Pin1);
  pr->write(',');
  pr->print(data->Pin2);
  pr->write(',');
  pr->print(data->Pin3);
  pr->write(',');
  pr->print(data->Pin4);
  pr->write(',');
  pr->print(data->Pin5);
  pr->write(','); 
  pr->print(data->ax, 6);
  pr->write(',');
  pr->print(data->ay, 6);
  pr->write(',');
  pr->print(data->az, 6);
  pr->write(',');
  pr->print(data->gx, 6);
  pr->write(',');
  pr->print(data->gy, 6);
  pr->write(',');
  pr->print(data->gz, 6);
  pr->write(',');
  pr->print(data->mx, 6);
  pr->write(',');
  pr->print(data->my, 6);
  pr->write(',');
  pr->println(data->mz, 6);
}
//==============================================================================
const uint64_t PREALLOCATE_SIZE  =  (uint64_t)PREALLOCATE_SIZE_MiB << 20;
// Max length of file name including zero byte.
#define FILE_NAME_DIM 40
// Max number of records to buffer while SD is busy.
const size_t FIFO_DIM = 512*FIFO_SIZE_SECTORS/sizeof(data_t);

#if SD_FAT_TYPE == 0
typedef SdFat sd_t;
typedef File file_t;
#elif SD_FAT_TYPE == 1
typedef SdFat32 sd_t;
typedef File32 file_t;
#elif SD_FAT_TYPE == 2
typedef SdExFat sd_t;
typedef ExFile file_t;
#elif SD_FAT_TYPE == 3
typedef SdFs sd_t;
typedef FsFile file_t;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE

sd_t sd;

file_t binFile;
file_t csvFile;
// You may modify the filename.  Digits before the dot are file versions.
char binName[] = "ExFatLogger00.bin";
//------------------------------------------------------------------------------
// This has not be configured but has been left in the code
// Could be used to only print timestamp at file start, then count datetime using micros
// See SdFat example TeensyRtcTimestamp for how to impliment with teensy
#if USE_RTC
 RTC_PCF8523 rtc;

// Call back for file timestamps.  Only called for file create and sync().
void dateTime(uint16_t* date, uint16_t* time, uint8_t* ms10) {
  DateTime now = rtc.now();

  // Return date using FS_DATE macro to format fields.
  *date = FS_DATE(now.year(), now.month(), now.day());

  // Return time using FS_TIME macro to format fields.
  *time = FS_TIME(now.hour(), now.minute(), now.second());

  // Return low time bits in units of 10 ms.
  *ms10 = now.second() & 1 ? 100 : 0;
}
#endif  // USE_RTC
//------------------------------------------------------------------------------
#define error(s) sd.errorHalt(&Serial, F(s))
#define dbgAssert(e) ((e) ? (void)0 : error("assert " #e))
//-----------------------------------------------------------------------------
// Convert binary file to csv file.
// Would be more efficient to write PC programme to convert large .bin files to csv
void binaryToCsv() {
  uint8_t lastPct = 0;
  uint32_t t0 = millis();
  data_t binData[FIFO_DIM];

  if (!binFile.seekSet(512)) {
	  error("binFile.seek faile");
  }
  uint32_t tPct = millis();
  printRecord(&csvFile, nullptr);
  while (!Serial.available() && binFile.available()) {
    int nb = binFile.read(binData, sizeof(binData));
    if (nb <= 0 ) {
      error("read binFile failed");
    }
    size_t nr = nb/sizeof(data_t);
    for (size_t i = 0; i < nr; i++) {
      printRecord(&csvFile, &binData[i]);
    }

    if ((millis() - tPct) > 1000) {
      uint8_t pct = binFile.curPosition()/(binFile.fileSize()/100);
      if (pct != lastPct) {
        tPct = millis();
        lastPct = pct;
        Serial.print(pct, DEC);
        Serial.println('%');
        csvFile.sync();
      }
    }
    if (Serial.available()) {
      break;
    }
  }
  csvFile.close();
  Serial.print(F("Done: "));
  Serial.print(0.001*(millis() - t0));
  Serial.println(F(" Seconds"));
}
//-------------------------------------------------------------------------------
void createBinFile() {
  binFile.close();
  while (sd.exists(binName)) {
    char* p = strchr(binName, '.');
    if (!p) {
      error("no dot in filename");
    }
    while (true) {
      p--;
      if (p < binName || *p < '0' || *p > '9') {
        error("Can't create file name");
      }
      if (p[0] != '9') {
        p[0]++;
        break;
      }
      p[0] = '0';
    }
  }
  if (!binFile.open(binName, O_RDWR | O_CREAT)) {
    error("open binName failed");
  }
  Serial.println(binName);
  if (!binFile.preAllocate(PREALLOCATE_SIZE)) {
    error("preAllocate failed");
  }

  Serial.print(F("preAllocated: "));
  Serial.print(PREALLOCATE_SIZE_MiB);
  Serial.println(F(" MiB"));
}
//-------------------------------------------------------------------------------
bool createCsvFile() {
  char csvName[FILE_NAME_DIM];
  if (!binFile.isOpen()) {
    Serial.println(F("No current binary file"));
    return false;
  }

  // Create a new csvFile.
  binFile.getName(csvName, sizeof(csvName));
  char* dot = strchr(csvName, '.');
  if (!dot) {
    error("no dot in filename");
  }
  strcpy(dot + 1, "csv");
  if (!csvFile.open(csvName, O_WRONLY | O_CREAT | O_TRUNC)) {
    error("open csvFile failed");
  }
  serialClearInput();
  Serial.print(F("Writing: "));
  Serial.print(csvName);
  Serial.println(F(" - type any character to stop"));
  return true;
}
//-------------------------------------------------------------------------------
void logData() {
  int32_t delta;  // Jitter in log time.
  int32_t maxDelta = 0;
  uint32_t maxLogMicros = 0;
  uint32_t maxWriteMicros = 0;
  size_t maxFifoUse = 0;
  size_t fifoCount = 0;
  size_t fifoHead = 0;
  size_t fifoTail = 0;
  uint16_t overrun = 0;
  uint16_t maxOverrun = 0;
  uint32_t totalOverrun = 0;
  uint32_t fifoBuf[128*FIFO_SIZE_SECTORS];
  data_t* fifoData = (data_t*)fifoBuf;

  // Write dummy sector to start multi-block write.
  dbgAssert(sizeof(fifoBuf) >= 512);
  memset(fifoBuf, 0, sizeof(fifoBuf));
  if (binFile.write(fifoBuf, 512) != 512) {
    error("write first sector failed");
  }
  serialClearInput();
  Serial.println(F("Type any character to stop"));

  // Wait until SD is not busy.
  while (sd.card()->isBusy()) {}

  // Start time for log file.
  uint32_t m = millis();

  // Time to log next record.
  uint32_t logTime = micros();
  while (true) {
    // Time for next data record.
    logTime += LOG_INTERVAL_USEC;

    // Wait until time to log data.
    delta = micros() - logTime;
    if (delta > 0) {
      Serial.print(F("delta: "));
      Serial.println(delta);
      error("Rate too fast");
    }
    while (delta < 0) {
      delta = micros() - logTime;
    }

    if (fifoCount < FIFO_DIM) {
      uint32_t m = micros();
      logRecord(fifoData + fifoHead, overrun);
      m = micros() - m;
      if (m > maxLogMicros) {
        maxLogMicros = m;
      }
      fifoHead = fifoHead < (FIFO_DIM - 1) ? fifoHead + 1 : 0;
      fifoCount++;
      if (overrun) {
        if (overrun > maxOverrun) {
          maxOverrun = overrun;
        }
        overrun = 0;
      }
    } else {
      totalOverrun++;
      overrun++;
      if (overrun > 0XFFF) {
        error("too many overruns");
      }
      if (ERROR_LED_PIN >= 0) {
        digitalWrite(ERROR_LED_PIN, HIGH);
      }
    }
    // Save max jitter.
    if (delta > maxDelta) {
      maxDelta = delta;
    }
    // Write data if SD is not busy.
    if (!sd.card()->isBusy()) {
      size_t nw = fifoHead > fifoTail ? fifoCount : FIFO_DIM - fifoTail;
      // Limit write time by not writing more than 512 bytes.
      const size_t MAX_WRITE = 512/sizeof(data_t);
      if (nw > MAX_WRITE) nw = MAX_WRITE;
      size_t nb = nw*sizeof(data_t);
      uint32_t usec = micros();
      if (nb != binFile.write(fifoData + fifoTail, nb)) {
        error("write binFile failed");
      }
      usec = micros() - usec;
      if (usec > maxWriteMicros) {
        maxWriteMicros = usec;
      }
      fifoTail = (fifoTail + nw) < FIFO_DIM ? fifoTail + nw : 0;
      if (fifoCount > maxFifoUse) {
        maxFifoUse = fifoCount;
      }
      fifoCount -= nw;
      if (Serial.available()) {
        break;
      }
    }
  }
  Serial.print(F("\nLog time: "));
  Serial.print(0.001*(millis() - m));
  Serial.println(F(" Seconds"));
  binFile.truncate();
  binFile.sync();
  Serial.print(("File size: "));
  // Warning cast used for print since fileSize is uint64_t.
  Serial.print((uint32_t)binFile.fileSize());
  Serial.println(F(" bytes"));
  Serial.print(F("totalOverrun: "));
  Serial.println(totalOverrun);
  Serial.print(F("FIFO_DIM: "));
  Serial.println(FIFO_DIM);
  Serial.print(F("maxFifoUse: "));
  Serial.println(maxFifoUse);
  Serial.print(F("maxLogMicros: "));
  Serial.println(maxLogMicros);
  Serial.print(F("maxWriteMicros: "));
  Serial.println(maxWriteMicros);
  Serial.print(F("Log interval: "));
  Serial.print(LOG_INTERVAL_USEC);
  Serial.print(F(" micros\nmaxDelta: "));
  Serial.print(maxDelta);
  Serial.println(F(" micros"));
}
//------------------------------------------------------------------------------
void openBinFile() {
  char name[FILE_NAME_DIM];
  serialClearInput();
  Serial.println(F("Enter file name"));
  if (!serialReadLine(name, sizeof(name))) {
    return;
  }
  if (!sd.exists(name)) {
    Serial.println(name);
    Serial.println(F("File does not exist"));
    return;
  }
  binFile.close();
  if (!binFile.open(name, O_RDONLY)) {
    Serial.println(name);
    Serial.println(F("open failed"));
    return;
  }
  Serial.println(F("File opened"));
}
//-----------------------------------------------------------------------------
void printData() {
  if (!binFile.isOpen()) {
    Serial.println(F("No current binary file"));
    return;
  }
  // Skip first dummy sector.
  if (!binFile.seekSet(512)) {
    error("seek failed");
  }
  serialClearInput();
  Serial.println(F("type any character to stop\n"));
  delay(1000);
  printRecord(&Serial, nullptr);
  while (binFile.available() && !Serial.available()) {
    data_t record;
    if (binFile.read(&record, sizeof(data_t)) != sizeof(data_t)) {
      error("read binFile failed");
    }
    printRecord(&Serial, &record);
  }
}
//------------------------------------------------------------------------------
void printUnusedStack() {
#if HAS_UNUSED_STACK  
  Serial.print(F("\nUnused stack: "));
  Serial.println(UnusedStack());
#endif  // HAS_UNUSED_STACK 
}
//------------------------------------------------------------------------------
void serialClearInput() {
  do {
    delay(10);
  } while (Serial.read() >= 0);
}
//------------------------------------------------------------------------------
bool serialReadLine(char* str, size_t size) {
  size_t n = 0;
  while(!Serial.available()) {
    yield();
  }
  while (true) {
    int c = Serial.read();
    if (c < ' ') break;
    str[n++] = c;
    if (n >= size) {
      Serial.println(F("input too long"));
      return false;
    }
    uint32_t m = millis();
    while (!Serial.available() && (millis() - m) < 100){}
    if (!Serial.available()) break;
  }
  str[n] = 0;
  return true;
}
//------------------------------------------------------------------------------
void testSensor() {
  const uint32_t interval = 200000;
  int32_t diff;
  data_t data;
  serialClearInput();
  Serial.println(F("\nTesting - type any character to stop\n"));
  delay(1000);
  printRecord(&Serial, nullptr);
  uint32_t m = micros();
  while (!Serial.available()) {
    m += interval;
    do {
      diff = m - micros();
    } while (diff > 0);
    logRecord(&data, 0);
    printRecord(&Serial, &data);
  }
}
//------------------------------------------------------------------------------
void setup() {
  // set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);
  
  if (ERROR_LED_PIN >= 0) {
    pinMode(ERROR_LED_PIN, OUTPUT);
    digitalWrite(ERROR_LED_PIN, HIGH);
  }
   while(!Serial);
  Serial.begin(9600);
  // Set analog res to 12-bit. 13 can be used with teensy 
  analogReadResolution(12);
setSyncProvider(getTeensy3Time); //rtc.begin();

 // IMU setup: see bolder flight library for details: https://github.com/bolderflight/MPU9250
 IMU.begin();
// setting the accelerometer full scale range to +/-16G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_16G);
  // setting the gyroscope full scale range to +/-250 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  // setting DLPF bandwidth. options: 5HZ 10HZ 20HZ 41HZ 92HZ 184HZ
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
  // Internal sampling rate of IMU
  // setting SRD to 99 for a 10 Hz update rate, 49 FOR 20 Hz, 9 for 100 Hz
  //to avoid aliasing sampling/update rate must be double dlpf bandwidth
  IMU.setSrd(9);
  
  // Wait for USB Serial
  while (!Serial) {
    SysCall::yield();
  }
  
  delay(1000);
  Serial.println(F("Type any character to begin"));
  while (!Serial.available()) {
    yield();
  }
  FillStack();
#if !ENABLE_DEDICATED_SPI
  Serial.println(F(
    "\nFor best performance edit SdFatConfig.h\n"
    "and set ENABLE_DEDICATED_SPI nonzero"));
#endif  // !ENABLE_DEDICATED_SPI

  Serial.print(FIFO_DIM);
  Serial.println(F(" FIFO entries will be used."));

  // Initialize SD.
  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt(&Serial);
  }

  
  
#if USE_RTC
  if (!rtc.begin()) {
    error("rtc.begin failed");
  }
  if (!rtc.isrunning()) {
    // Set RTC to sketch compile date & time.
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    error("RTC is NOT running!");
  }
  // Set callback
  FsDateTime::setCallback(dateTime);
#endif  // USE_RTC
}
//------------------------------------------------------------------------------
void loop() {
// Serial interface currently being used. i.e. inputting 'r' into the serial monitor
// to initiate logging. New loop function required to log remotely.
// A push button to start/stop logging system could be implimented. 
  printUnusedStack();
  // Read any Serial data.
  serialClearInput();

  if (ERROR_LED_PIN >= 0) {
    digitalWrite(ERROR_LED_PIN, LOW);
  }
  Serial.println();
  Serial.println(F("type: "));
  Serial.println(F("b - open existing bin file"));
  Serial.println(F("c - convert file to csv"));
  Serial.println(F("l - list files"));
  Serial.println(F("p - print data to Serial"));
  Serial.println(F("r - record data"));
  Serial.println(F("t - test without logging"));
  while(!Serial.available()) {
    SysCall::yield();
  }
  char c = tolower(Serial.read());
  Serial.println();

  if (c == 'b') {
    openBinFile();
  } else if (c == 'c') {
    if (createCsvFile()) {
      binaryToCsv();
    }
  } else if (c == 'l') {
    Serial.println(F("ls:"));
    sd.ls(&Serial, LS_DATE | LS_SIZE);
  } else if (c == 'p') {
    printData();
  } else if (c == 'r') {
    createBinFile();
    logData();
  } else if (c == 't') {
    testSensor();
  } else {
    Serial.println(F("Invalid entry"));
  }
}
