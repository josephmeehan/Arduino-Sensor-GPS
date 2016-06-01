// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746
// Pick one up today at the Adafruit electronics shop 
// and help support open source hardware & software! -ada

//This code is intended for use with Arduino Leonardo and other ATmega32U4-based Arduinos

#include <Adafruit_GPS.h>
#include <SD.h>
#include <CurieIMU.h>
#include <MadgwickAHRS.h>

#define M_PI 3.141592653589793238462643
#define SPEED_AVG 10

// ******************* Timer Setup **************
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const int streamPeriod = 200;          //To stream at 1Hz without using additional timers (time period(ms) =1000/frequency(Hz))

// ******************* Curie Setup **************

Madgwick filter;
int factor = 180;
float yaw, roll, pitch;

int ax, ay, az;         // accelerometer values
int gx, gy, gz;         // gyrometer values

const int ledPin = 13;      // activity LED pin
boolean blinkState = false; // state of the LED

int calibrateOffsets = 1; // int to determine whether calibration takes place or not

// ******************* GPS Setup **************

// If using hardware serial, comment
// out the above two lines and enable these two lines instead:
Adafruit_GPS GPS(&Serial1);
//HardwareSerial mySerial = Serial1;


// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  false
/* set to true to only log to SD when GPS has a fix, for debugging, keep it false */
#define LOG_FIXONLY false

//*************SD card setup **************

// Set the pins used
#define chipSelect 10
#define ledPin 13

File logfile;

// read a Hex value and return the decimal equivalent
uint8_t parseHex(char c) {
  if (c < '0')
    return 0;
  if (c <= '9')
    return c - '0';
  if (c < 'A')
    return 0;
  if (c <= 'F')
    return (c - 'A')+10;
}

// blink out an error code
void error(uint8_t errno) {

  while(1) {
    uint8_t i;
    for (i=0; i<errno; i++) {
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
      delay(100);
    }
    for (i=errno; i<10; i++) {
      delay(200);
    }
  }
}

void setup()  
{
  Serial.begin(9600);
  delay(5000);
  Serial.println("GPS and Sensor test!");

// ******************* Sensor Initialisation **************

  // initialize device
  Serial.println("Initializing IMU device...");
  CurieIMU.begin();

  // verify connection
  Serial.println("Testing device connections...");
  if (CurieIMU.begin()) {
    Serial.println("CurieIMU connection successful");
  } else {
    Serial.println("CurieIMU connection failed");
  }

  // use the code below to calibrate accel/gyro offset values
  if (calibrateOffsets == 1) {
    Serial.println("Internal sensor offsets BEFORE calibration...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -235
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 168
    Serial.print(CurieIMU.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(CurieIMU.getGyroOffset(Z_AXIS));

    // To manually configure offset compensation values,
    // use the following methods instead of the autoCalibrate...() methods below
    //CurieIMU.setAccelerometerOffset(X_AXIS,128);
    //CurieIMU.setAccelerometerOffset(Y_AXIS,-4);
    //CurieIMU.setAccelerometerOffset(Z_AXIS,127);
    //CurieIMU.setGyroOffset(X_AXIS,129);
    //CurieIMU.setGyroOffset(Y_AXIS,-1);
    //CurieIMU.setGyroOffset(Z_AXIS, 254);

    Serial.println("About to calibrate. Make sure your board is stable and upright");
    delay(5000);

    // The board must be resting in a horizontal position for
    // the following calibration procedure to work correctly!
    Serial.print("Starting Gyroscope calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateGyroOffset();
    Serial.println(" Done");

    Serial.print("Starting Acceleration calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
    Serial.println(" Done");

    Serial.println("Internal sensor offsets AFTER calibration...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -2359
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 1688
    Serial.print(CurieIMU.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(CurieIMU.getGyroOffset(Z_AXIS));
  }

  
// ******************* GPS Initialisation **************

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  //GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  // Ask for firmware version
  Serial1.println(PMTK_Q_RELEASE);

  //************SD card initialisation************
  pinMode(ledPin, OUTPUT);

  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {      
    Serial.println("failed!");
    error(2);
  }   
  char filename[15];
  strcpy(filename, "GPSLOG00.GPX");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i/10;
    filename[7] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    Serial.print("No CR"); 
    Serial.println(filename);
    error(3);
  }
  Serial.print("Writing "); 
  Serial.println(filename);

  // write header to file

/*  char  stringhdr[] = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<gpx version=\"1.1\" \ncreator=\"IR84\"\nxmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\nxmlns=\"http://www.topografix.com/GPX/1/1\"\nxsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">\n<trk>\n<trkseg>\n";
  uint8_t stringsize = strlen(stringhdr);
  logfile.write(stringhdr, stringsize);
  logfile.flush();
  Serial.print(stringhdr);*/
  
}

uint32_t timer = millis();
int first_time = 0;
double p1latitude, p2latitude, p1longitude, p2longitude, p1timestamp, p2timestamp; 
double total_distance = 0, dist_prev = 0, dist_speed = 0;
double t_start=0, t_end = 1, time_s;
int dist_count = 0;
double speed_kph = 0;

void loop() {
  char stringptrgps [200];
  char stringptrsensor [200];
  char stringbuffer[10];
  char timebuffer[25];
   
  
    //************  Read GPS data ************
    
    //  if (! usingInterrupt) {
        // read data from the GPS in the 'main loop'
        char c = GPS.read();
        // if you want to debug, this is a good time to do it!
        if (GPSECHO)
          if (c) Serial.print(c);
    //  }
    
    //************  Read Sensor data ************
    
      // read raw accel/gyro measurements from device
      CurieIMU.readMotionSensor(ax, ay, az, gx, gy, gz);
    
      filter.updateIMU(gx / factor, gy / factor, gz / factor, ax, ay, az);
    
      yaw = filter.getYaw()*180/3.1415;
      roll = filter.getRoll()*180/3.1415;
      pitch = filter.getPitch()*180/3.1415;
    

  if ((millis() - lastStreamTime) >= streamPeriod)
  {
    lastStreamTime = millis();
    
    // Print Sensor data to SDcard
    
        strcpy(stringptrsensor,"sensor ");
        strcat(stringptrsensor, "t ");
        dtostrf(millis()/1000, 8, 2, stringbuffer);
        strncat(stringptrsensor,stringbuffer, 10);
        strncat(stringptrsensor, " aX ", 5);   
        dtostrf(ax, 6, 2, stringbuffer);
        strncat(stringptrsensor, stringbuffer,10);
        strncat(stringptrsensor, " aY ", 5);   
        dtostrf(ay, 6, 2, stringbuffer);
        strncat(stringptrsensor, stringbuffer,10);
        strncat(stringptrsensor, " aZ ", 5);   
        dtostrf(az, 6, 2, stringbuffer);
        strncat(stringptrsensor, stringbuffer,10);
        strncat(stringptrsensor, " yaw ", 6); 
        dtostrf(yaw, 6, 2, stringbuffer);
        strncat(stringptrsensor, stringbuffer,10);
        strncat(stringptrsensor, " roll ", 6); 
        dtostrf(roll, 6, 2, stringbuffer);
        strncat(stringptrsensor, stringbuffer,10);
        strncat(stringptrsensor, " pitch ", 7); 
        dtostrf(pitch, 6, 2, stringbuffer);
        strncat(stringptrsensor, stringbuffer,10);
        strncat(stringptrsensor, "\r\n", 2);


    
        uint8_t stringsize = strlen(stringptrsensor);
          if (stringsize != logfile.write(stringptrsensor, stringsize))    //write the string to the SD file
              error(4);
          if (strstr(stringptrsensor, "sensor")){
            logfile.flush();
//            Serial.print(stringptrsensor);
          }

   }
  // end of timer loop for sensor data 
  
      // if a sentence is received, we can check the checksum, parse it...
      if (GPS.newNMEAreceived()) {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences! 
        // so be very wary if using OUTPUT_ALLDATA and trying to print out data
        
        // Don't call lastNMEA more than once between parse calls!  Calling lastNMEA 
        // will clear the received flag and can cause very subtle race conditions if
        // new data comes in before parse is called again.
    
        if (!GPS.parse(GPS.lastNMEA())){   // this also sets the newNMEAreceived() flag to false
          Serial.println("not parsed");
          return;  // we can fail to parse a sentence in which case we should just wait for another
          
        }
        // Sentence parsed! 
        Serial.println("OK");
        if (LOG_FIXONLY && !GPS.fix) {
          Serial.print("No Fix");
          return;
        }
    
    // Calculate speed and distance
    
    p1latitude = GPS.latitudeDegrees;
    p1longitude = GPS.longitudeDegrees;
    p1timestamp = GPS.seconds;

    if(first_time == 0){
      Serial.print("First Time");
        p2latitude = p1latitude;
        p2longitude = p1longitude;
        first_time = 1;
    }
    
      double dist = distance_on_geoid(p1latitude, p1longitude, p2latitude, p2longitude);
      if(isnan(dist)) dist = dist_prev;

      dist_speed += dist;
      t_end = GPS.seconds;
      if (dist_count == SPEED_AVG){
        if (t_end < t_start) t_end += 60;
        time_s = t_end - t_start;
        double speed_mps = dist_speed / time_s;
        speed_kph = (speed_mps * 3600.0) / 1000.0;
        dist_speed = 0;
        t_start = GPS.seconds;
        dist_count = 0;
      }
      dist_count++;
      
      dist_prev = dist;
      p2latitude = p1latitude;
      p2longitude = p1longitude;
  
      total_distance += dist;


    
    //Print out GPS data to SDcard
    
        strcpy(stringptrgps,"lat ");
        dtostrf(GPS.latitudeDegrees, 7, 4, stringbuffer);
        strncat(stringptrgps,stringbuffer, 10);
        strncat(stringptrgps," lon ", 10);
        dtostrf(GPS.longitudeDegrees, 7, 4, stringbuffer);
        strncat(stringptrgps,stringbuffer, 10);       
        strncat(stringptrgps," ele ", 10);
        dtostrf(GPS.altitude, 4, 0, stringbuffer);
        strncat(stringptrgps,stringbuffer, 10);
        strncat(stringptrgps," time ", 12);
        sprintf(timebuffer,"20%d-%d-%dT%d:%d:%dZ",GPS.year,GPS.month,GPS.day,GPS.hour,GPS.minute,GPS.seconds);
        strncat(stringptrgps,timebuffer,25);
        strncat(stringptrgps," speed1 ", 10);
        dtostrf(GPS.speed, 4, 1, stringbuffer);
        strncat(stringptrgps,stringbuffer, 10);
        strncat(stringptrgps," speed2 ", 10);
        dtostrf(speed_kph, 4, 1, stringbuffer);
        strncat(stringptrgps,stringbuffer, 10);
        strncat(stringptrgps," dist ", 10);
        dtostrf(total_distance, 6, 0, stringbuffer);
        strncat(stringptrgps,stringbuffer, 10);
        strncat(stringptrgps, "\r\n", 2);
    
    
        // Rad. lets log it!
        Serial.println("Log");
    
        uint8_t stringsize = strlen(stringptrgps);
        if (stringsize != logfile.write(stringptrgps, stringsize))    //write the string to the SD file
            error(4);
        if (strstr(stringptrgps, "lat")){
          logfile.flush();
          Serial.print(stringptrgps);
        }
    //    Serial.println();
      }



}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;

  return a;
}

//
//
double distance_on_geoid(double lat1, double lon1, double lat2, double lon2) {

  // Convert degrees to radians
  lat1 = lat1 * M_PI / 180.0;
  lon1 = lon1 * M_PI / 180.0;

  lat2 = lat2 * M_PI / 180.0;
  lon2 = lon2 * M_PI / 180.0;

  // radius of earth in metres
  double r = 6378100;

  // P
  double rho1 = r * cos(lat1);
  double z1 = r * sin(lat1);
  double x1 = rho1 * cos(lon1);
  double y1 = rho1 * sin(lon1);

  // Q
  double rho2 = r * cos(lat2);
  double z2 = r * sin(lat2);
  double x2 = rho2 * cos(lon2);
  double y2 = rho2 * sin(lon2);

  // Dot product
  double dot = (x1 * x2 + y1 * y2 + z1 * z2);
  double cos_theta = dot / (r * r);

  double theta = acos(cos_theta);

  // Distance in Metres
  return r * theta;
}
