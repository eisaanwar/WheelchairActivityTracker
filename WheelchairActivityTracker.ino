/*
 * This file contains some code written by consulting library tutorials for the respective components. * 
 */
 
/*#include <Arduino.h>*/
#include <ArduinoJson.h>
#include <Encoder.h>
/*#include <SPI.h>*/
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"
#include <Adafruit_GPS.h>
#include <Wire.h>
#include "RTClib.h"
/*#include <Adafruit_Sensor.h>*/
#include <Adafruit_BME280.h>
#include "BluefruitConfig.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define FACTORYRESET_ENABLE         1 // for Adafruit BLE Module
#define MINIMUM_FIRMWARE_VERSION    "0.6.6" // for Adafruit BLE Module
#define MODE_LED_BEHAVIOUR          "MODE" // for Adafruit BLE Module


boolean fullDebug = false; // set to true to output raw data and bebug data from each sensor


#define mySerial Serial2 // Serial for Adafruit Ultimate GPS Breakout
Adafruit_GPS GPS(&mySerial); // Set the GPS to the serial connection
#define GPSECHO false // don't print out all GPS information
boolean usingInterrupt = false; 
void useInterrupt(boolean);


// timers for reading sensor data
uint32_t timerDistanceTravelled = millis(); // timer for distance travelled
uint32_t timerGPS = millis(); // timeer for GPS data
uint32_t timerIMU = millis(); // time for IMU data collection
uint32_t timerEnvironment = millis(); // time for environment data
uint32_t timerPrintData = millis(); // timer to print information
uint32_t timerSendBLE = millis(); // time between sending BLE packets


RTC_DS3231 rtc; // Real Time Clock
DateTime now;

Adafruit_BNO055 bno = Adafruit_BNO055(); // Inertial Measurement Unit
Adafruit_BME280 bme; // Environment Sensor

Encoder myEnc1(30, 31); // Optical Rotary Encoders
Encoder myEnc2(32, 33);

Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN); // Serial for Bluetooth Low Energy Module

StaticJsonBuffer<1024> json;
StaticJsonBuffer<1024> jsonReceive;


/******************************************************************/
/*   Setup is called once the the begining of the program */
/******************************************************************/
void setup(void)
{
  Serial.begin(115200); // start serial communication

  setupBluetooth(); // setup bluetooth communications
  
  setupRealTimeClock(); // setup the real time clock

  setupGobalPositioningSystem(); // setup the GPS

  setupInertialMeasurementUnit(); // setup the IMU

  setupEnvironmentSensor(); // setup the environment sensor
}


/******************************************************************/
/*   Setup Adafruit Bluefruit Low Energy UART Friend */
/******************************************************************/
void setupBluetooth () {
  if (!ble.begin(VERBOSE_MODE)) // if BLE cannot be started
  {
    Serial.println("Bluetooth Not Detected");
  }

  if (FACTORYRESET_ENABLE)
  {
    if ( ! ble.factoryReset() ) {
      Serial.println("Couldn't factory reset");
    }
  }

  ble.echo(false); // Disable echo
  ble.info(); // Bleutooth Module Information
  ble.verbose(false);  // debug info is a little annoying after this point!
  delay(1000); // wait for console opening

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  }
}



/******************************************************************/
/*   Setup ZS-042 Real Time Clock */
/******************************************************************/
void setupRealTimeClock() {
  if (! rtc.begin()) {
    Serial.println("RTC Not Detected");
  }

  if (rtc.lostPower()) { // if the RTC has lost power then time is lost
    Serial.println("RTC Time Lost. Resetting");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Set Date and Time to time sketch was uploaded
  }

  now = rtc.now(); // replace with now
}


/******************************************************************/
/*   Setup Adarufit Ultimate GPS Breakout */
/******************************************************************/
void setupGobalPositioningSystem() {  
  //GPS
  mySerial.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
#ifdef __arm__
  usingInterrupt = false;  //NOTE - we don't want to use interrupts on the Due
#else
  useInterrupt(true);
#endif
  delay(100);
  mySerial.println(PMTK_Q_RELEASE);
}


/******************************************************************/
/*   Setup Adafruit BNO055 Inertial Measurement Unit */
/******************************************************************/
void setupInertialMeasurementUnit() {
  if (!bno.begin())
  {
    Serial.print("IMU Not Connected");
  }
  delay(500);
  bno.setExtCrystalUse(true);
  delay(500);
}

// Setup BME280 Environment Sensor
void setupEnvironmentSensor() {
  bool status;
  // default settings
  status = bme.begin();
  if (!status) {
    Serial.println("Environement Sensor Not Connected");
  }
  delay(500);
}




// Interrupt Setup for the Adafruit Ultimate GPS Breakout
#ifdef __AVR__
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
  // writing direct to UDR0 is much much faster than Serial.print
  // but only one character can be written at a time.
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
#endif //#ifdef__AVR__




/******************************************************************/
/*   Print data to Serial console */
/******************************************************************/
void PrintDataToSerial(int* TimeValue, float* Distances, float intervalDistance, float totalRotations, float currentSpeed, float currentAcceleration, float distance15Sec, float* IMUValue, float* EnvironmentValue, bool incommingRequest) {
  Serial.print("OUT-> ");

  Serial.print(" MILLIS: ");
  Serial.print(","); Serial.print(millis()); Serial.print(",");


  Serial.print("Time: ");

  Serial.print(","); Serial.print(TimeValue[0]); Serial.print(",");
  Serial.print(":");
  Serial.print(","); Serial.print(TimeValue[1]); Serial.print(",");
  Serial.print(":");
  Serial.print(","); Serial.print(TimeValue[2]); Serial.print(",");

  Serial.print(" |-| ");

  Serial.print("Dist: ");
  Serial.print(","); Serial.print(Distances[0] / 1000); Serial.print(",");
  Serial.print(" |-| ");

  Serial.print("Speed: ");
  Serial.print(","); Serial.print(currentSpeed); Serial.print(",");
  Serial.print(" |-| ");

  Serial.print("Acc: ");
  Serial.print(","); Serial.print(currentAcceleration); Serial.print(",");
  Serial.print(" |-| ");

  Serial.print("Rotations: ");
  Serial.print(","); Serial.print(Distances[1]); Serial.print(","); //
  //Serial.print(","); Serial.print(totalRotations); Serial.print(","); //
  Serial.print(" |-| ");

  if (fullDebug) {
    Serial.print("DistInt: ");
    Serial.print(","); Serial.print(intervalDistance / 1000); Serial.print(","); // 
    Serial.print(" |-| ");
  }

  Serial.print("GPS: ");
  Serial.print(" |-| ");


  Serial.print("IMU:");
  Serial.print(" x: ");
  Serial.print(","); Serial.print(IMUValue[0]); Serial.print(",");
  Serial.print(" y: ");
  Serial.print(","); Serial.print(IMUValue[1]); Serial.print(",");
  Serial.print(" z: ");
  Serial.print(","); Serial.print(IMUValue[2]); Serial.print(",");

  Serial.print(" |-| ");


  Serial.print("ENVT:");
  Serial.print(" Temp: ");
  Serial.print(","); Serial.print(EnvironmentValue[0]); Serial.print(",");
  Serial.print(" Press: ");
  Serial.print(","); Serial.print(EnvironmentValue[1]); Serial.print(",");
  Serial.print(" Hum: ");
  Serial.print(","); Serial.print(EnvironmentValue[2]); Serial.print(",");
  Serial.print(" |-| ");


  Serial.print("BLE:");
  if (incommingRequest) {
    Serial.print(","); Serial.print("Y"); Serial.print(",");
    //Serial.print(" Data Requested ");
  } else {
    Serial.print(","); Serial.print("N"); Serial.print(",");
    //Serial.print(" No Request ");
  }

  Serial.println(""); // new line
}


/******************************************************************/
/*   Calculate Speed */
/******************************************************************/
float getSpeed(float newDistance, float oldDistance, int newTime, int oldTime) {
  float distanceDifference = (newDistance - oldDistance);
  if(fullDebug) Serial.println(distanceDifference);

  int timeDifference = (newTime - oldTime);
  if(fullDebug) Serial.println(timeDifference);
  
  float currentSpeed = distanceDifference / timeDifference;
  
  if(fullDebug) Serial.println(currentSpeed);
  return currentSpeed;
}


/******************************************************************/
/*   Calculate Acceleration */
/******************************************************************/
float getAcceleration(float currentSpeed, int newTime, int oldTime, float accerlationX, float accerlationZ) { // Fuse with IMU

  float imuAcceleration = sqrt( sq(accerlationX) + sq(accerlationZ)); // should it just be the X?
  float encoderAcceleration;

  float averageAcceleration;
  // combine
  //Serial.print("X");
  //Serial.println(accerlationX); // x and y magnitude
  //Serial.print("Y");
  //Serial.println(accerlationY); // x and y magnitude
  int timeDifference = (newTime - oldTime);
  encoderAcceleration = currentSpeed / timeDifference;

  averageAcceleration = (imuAcceleration + encoderAcceleration) / 2;

  //Serial.println("Acc");
  //Serial.println(imuAcceleration);
  //Serial.println(encoderAcceleration);
  return averageAcceleration;


}



/******************************************************************/
/*   Check for Information Requests */
/******************************************************************/
int * UpdateTime() {
  now = rtc.now(); // update date and time

  static int timeValues[3];

  timeValues[0] = now.hour();
  timeValues[1] = now.minute();
  timeValues[2] = now.second();

  return timeValues;
}



imu::Vector<3> euler;
imu::Vector<3> linAccelerometer;

/******************************************************************/
/*   Get Data From Adafuirt BNO055 IMU */
/******************************************************************/
float * GetInertailData() {

  float heading = 1.1234;
  float forwardInclination;
  float crossInclination;

  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); // euler.x .y .z for data
  linAccelerometer = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL );

  //for debug
  heading = euler.x();
  forwardInclination = euler.y(); // double check this
  crossInclination = euler.z();


  // FILTERING

  static float imuValues[6];

  imuValues[0] = euler.x(); // inclination angles
  imuValues[1] = euler.y()  - 4.50; // calibrated for angle
  imuValues[2] = euler.z();

  imuValues[3] = linAccelerometer.x();
  imuValues[4] = linAccelerometer.y();
  imuValues[5] = linAccelerometer.z();

  return imuValues;

}


/******************************************************************/
/*   Get Data From Adafuirt Ultimate GPS Breakout */
/******************************************************************/
float GetGPSData() {
  //Serial.print("GPS Method");
  char c = GPS.read();

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    GPS.lastNMEA();
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return -1; // we can fail to parse a sentence in which case we should just wait for another
  }

  //debug
  if (fullDebug) {
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
  }
  */


  if (GPS.fix) {
    if (fullDebug) Serial.println("Signal Found");
    if (fullDebug) Serial.println(GPS.speed);
    return (GPS.speed);
  } else {
    return -1; // no reading
  }
  return -1;
  if (fullDebug) {
    Serial.print("Location: ");
    Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
    Serial.print("Speed (knots): "); Serial.println(GPS.speed);
    Serial.print("Angle: "); Serial.println(GPS.angle);
    Serial.print("Altitude: "); Serial.println(GPS.altitude);
    Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
  }
}



/******************************************************************/
/*   Get Data From BME280 (Environment Sensor) */
/******************************************************************/
float* GetEnvironmentSensorData() {
  float temperature;
  float pressure;
  float humidity;

  static float environmentValues[3];

  temperature = bme.readTemperature();
  pressure = bme.readPressure() / (float)100;
  humidity = bme.readHumidity();


  environmentValues[0] = temperature;
  environmentValues[1] = pressure;
  environmentValues[2] = humidity;

  return environmentValues;

}

/******************************************************************/
/*   Check for Information Requests */
/******************************************************************/

String receivedData; //BLE
String receivedDataBuffer; // buffer for whole input
String endOfBuffer;

const char* requestMessage = "APP_GET_SAMPLES";

long timeReceived;
char jsonCharArray[1024];


bool CheckForIncommingRequest() {
  //Serial.println("CheckForIncommingRequest begin");
  // Check for incoming characters from Bluefruit
  ble.println("AT+BLEUARTRX");
  ble.readline();
  if (strcmp(ble.buffer, "OK") == 0) { // check if the buffer is empty
    // no data
    //Serial.println("No Request");
    //Serial.println("CheckForIncommingRequest end");

    return false; //
  }
  // Some data was found, its in the buffer
  Serial.println("DataFound");
  receivedData = ble.buffer;

  receivedDataBuffer = receivedDataBuffer + receivedData; // add the data to the buffer

  Serial.println(receivedData);
  //Serial.print(F("[Recv] ")); Serial.println(ble.buffer);

  ble.waitForOK();

  endOfBuffer = receivedDataBuffer.substring(receivedDataBuffer.length() - 2); // end of the buffered string
  Serial.println("Reached End of Buffer");


  //Serial.println(receivedData.substring(0, 4));
  if ((receivedData.equals("r"))) {
    return true;
  }

  //if (endOfBuffer.equals("\r\n")) { // reached the end of a JSON transmission
  if (endOfBuffer.equals("}}") || endOfBuffer.equals("]}")) { // reached the end of a JSON transmission

    // JSON deserialise
    Serial.println("FULL MESSAGE RECEIVED");
    receivedDataBuffer.toCharArray(jsonCharArray, 1024);

    JsonObject& root = jsonReceive.parseObject(jsonCharArray);
    //JsonObject& root = jsonReceive.parseObject(receivedDataBuffer);

    // Test if parsing succeeds.
    if (!root.success()) {
      Serial.println("parseObject() failed");
      endOfBuffer = "";
      //stringReceivedMessages = "";
      receivedDataBuffer = "";
      jsonReceive.clear();
      return false; // failed for other reasons
    }

    if(fullDebug) Serial.println(receivedDataBuffer);

    const char* receivedMessage = root["_"];  // read target section of JSON

    String stringReceivedMessages(receivedMessage);
    if(fullDebug) Serial.println(stringReceivedMessages);

    if (stringReceivedMessages.equals("APP_GET_SAMPLES")) {

      Serial.println("data requested");
      // clear data buffers
      stringReceivedMessages = "";
      receivedDataBuffer = "";
      endOfBuffer = "";
      receivedMessage = "";
      jsonReceive.clear();

      return true;

    } else {
      //Serial.println("No Request");
      receivedMessage = "";
      stringReceivedMessages = "";
      receivedDataBuffer = "";
      endOfBuffer = "";
      jsonReceive.clear();

      return false;
    }

  }
  jsonReceive.clear();
  return false;
}


/******************************************************************/
/*   Send Requested Information via Bluetooth */
/******************************************************************/

String* prepareBleData(float intervalDistance) {

  int timestamp = now.unixtime();
  auto roundId = timestamp - (timestamp % 15); // 60 for 1 min

  JsonObject& msg = json.createObject(); // new JSON
  msg["_"] = "DEVICE_SAMPLES"; // main purpose
  msg["t"] = timestamp;
  JsonArray& samples = json.createArray();
  JsonObject& sample = json.createObject();
  sample["1"] = roundId;
  sample["4"] = timestamp;
  sample["2"] = 15;
  sample["3"] = "MAC";
  sample["0"] = "COMBINED"; //COMBINED_ROUND_DATA"
  sample["6"] = ((intervalDistance) / 1000); // distance travelled
  sample["9"] = 2; //forwardInclination;
  sample["a"] = 2; //crossInclination;
  sample["c"] = 2; //heading;


  samples.add(sample);
  msg["p"] = samples;

  String output;
  msg.printTo(output);

  output = output + "\\r\\n";

  static String outputs[3];

  outputs[0] = output.substring(0, 128);
  outputs[1] = output.substring(128, 256);
  return outputs;
}

int dataPacketToSend = 0; // size of outputs

bool SendRequestedData(String * outputs) {
  bool bleSendSuccess = true;

  if (dataPacketToSend <= 1) { // size of outputs
    if (millis() - timerSendBLE > 500) {
      //ble.print("AT+BLEUARTFIFO=TX"); //debug. Check buffer size
      char inputs[BUFSIZE + 1];
      outputs[dataPacketToSend].toCharArray(inputs, BUFSIZE + 1);

      // Send characters to Bluefruit
      if(fullDebug)Serial.print("[Send] ");
      if(fullDebug)Serial.println(inputs);

      timerSendBLE = millis(); // reset the timer
      //GetGPSData(); //Get GPS Data

      ble.print("AT+BLEUARTTX=");
      ble.println(inputs);

      //Serial.println("BeforeSend");
      if (! ble.waitForOK() ) {
        Serial.println("Sending Failed");
        bleSendSuccess = false;

      }
      //Serial.println("AfterSend");
      dataPacketToSend++;
    }
  }

  if (dataPacketToSend > 1) { //finsihed
    Serial.println("Finished Send");
    json.clear();
    dataPacketToSend = 0; // start next data packet from the beginning
    return bleSendSuccess;
  } else { // not finished
    bleSendSuccess = false;
    return bleSendSuccess;
  }
}


/******************************************************************/
/*   Update distance from the Optical Rotary Encoders */
/******************************************************************/

const float radiusRatio = 33500; // 1 wheel turn for 10000 encoder turns
const int wheelCircumference = 1885; // in mm
const int wheelbase = 540; // in mm
const float roationLenght = 2 * wheelbase * 3.14159; //

int magnitude1; // magnitudes of rotation
int magnitude2;

int difference1; // difference of encoder position
int difference2;

float averageEncoderRotation;
float distanceTravelled;

long oldPosition1 = 0;
long oldPosition2 = 0;

float* DistanceTravelled() {

  float distanceLength;
  static float distances[2]; // total and rotations
  int roationDifference;

  long newPosition1 = myEnc1.read();
  long newPosition2 = myEnc2.read();
  if(fullDebug) {Serial.print("pos1: ");Serial.println(newPosition1);}
  if(fullDebug) {Serial.print("pos1: ");Serial.println(newPosition2);}

  if ((newPosition1 != oldPosition1) || (newPosition2 != oldPosition2)) { // check to see if any of the encoders have moved
    if (newPosition1 != oldPosition1) {
      if (oldPosition1 > newPosition1) { // encoder has moved forwards
        difference1 = oldPosition1 - newPosition1; // find difference
      } else {
        difference1 = newPosition1 - oldPosition1; // find difference
      }
      oldPosition1 = newPosition1; // update encoder position
      magnitude1 = (magnitude1 + difference1); // negate the number as both the encoders are moving in the opposite direction
    }

    if (newPosition2 != oldPosition2) {
      if (oldPosition2 > newPosition2) { // encoder has moved forwards
        difference2 = oldPosition2 - newPosition2; // find difference
      } else {
        difference2 = newPosition2 - oldPosition2; // find difference
      }
      oldPosition2 = newPosition2; // update encoder position
      magnitude2 = magnitude2 + difference2; // negate the number as both the encoders are moving in the opposite direction
    }

    averageEncoderRotation = (magnitude1 + magnitude2) / 2; // calculate the average rotation
    distanceTravelled = (averageEncoderRotation / radiusRatio) * wheelCircumference;

    if ((magnitude1 - magnitude2) >= 0) {
      roationDifference = (magnitude1 - magnitude2);
    } else {
      roationDifference = (magnitude2 - magnitude1);
    }

    //distanceLength = ((float)roationDifference / (float)radiusRatio) * (float)wheelCircumference;
    if(fullDebug) {Serial.print("mag1"); Serial.println(magnitude1);}
    if(fullDebug) {Serial.print("mag1");Serial.println(magnitude2);}

    if(fullDebug) Serial.println(roationDifference);
    distanceLength = ((roationDifference / radiusRatio) * wheelCircumference);
    
    if(fullDebug) Serial.println(distanceLength);    
    if(fullDebug) Serial.println(distanceLength / roationLenght);
  }
  distances[1] = (distanceLength / roationLenght);
  distances[0] = distanceTravelled;
  return distances;
}


/******************************************************************/
/*   Update the roation using the IMU */
/******************************************************************/

float lastRotation = 0;
bool firstMeasurement = true;

float getIMURotationDifference () {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag); // get the calibration state

  if (mag >= 2) { // calibrated
    if (firstMeasurement == true) { // if this is the first measurement then return -1
      lastRotation = euler.x(); // first mesurement
      firstMeasurement = false; // have an inital measrement now
      if(fullDebug) Serial.println("First Measurement");
      return -1; // only first measurement
    }

  } else {
    if(fullDebug) Serial.println("NOT CALIBRATED");
    return -1; // not calibrated
  }
  float currentRotation = euler.x(); // read the current heading
  float rotationDifference;

  rotationDifference = abs (lastRotation - currentRotation); // calcualte the differnce in roation
  if(fullDebug) Serial.println(rotationDifference);

  if (rotationDifference > 300) { // if the difference is above 300 then moved past north (0 degrees)
    rotationDifference = 360 - rotationDifference;
  }

  lastRotation = currentRotation;
  
  return rotationDifference / 360; // 1 is 1 complete turn
}



/******************************************************************************************************************/
/*   Low Pass Filter to smooth out the data produced by the IMU*/
/* this method was produced by flowwing instructions at: https://helpful.knobs-dials.com/index.php/Low-pass_filter */
/******************************************************************************************************************/

float previousOut[6];  // to store the data for the IMU
float currentOut[6];
float input[6];

int LowPassFilter(int index, int sampleRate, int samples, float alpha, char usePrevious, float value) {

  float oneMinusAlpha = 1.0 - alpha; // for the denominator of the low pass filter formula
  int microDelay = max(100, (1000000 / sampleRate) - 160);

  if (usePrevious == false) { // setting for each call
    input[index] = value; // read in value
    previousOut[index] = input[index]; // assign to previous
  }

  int i;
  for (i = samples; i > 0; i--) {
    delayMicroseconds(microDelay);
    input[index] = value; // read in value
    currentOut[index] = alpha * input[index] + oneMinusAlpha * previousOut[index]; // calculate output
    previousOut[index] = currentOut[index]; // assign to previous
  }
  return currentOut[index];
}


/****************************************************
   MIAN LOOP
 *****************************************************/

bool idleMovement = true;
bool incommingRequest = false;
float oldDistance;
int oldTime;
float distance;
float totalRotations = 0;
float encoderRotations = 0;
float intervalDistance;
float intervalEncoderRotations = 0;
float oldInterval;
float oldRotations = 0;
int distance15Sec;
float currentSpeed;
float encoderSpeed;
float currentAcceleration;
bool sentIntervalBleData = false;
bool dataToSend = false;
float GPSSpeed;

void loop(void) {

  // keep total distance travelled here
  // interate it in increments
  float IMURotations = 0;


  static int *TimeValue; // array to store values from the Real Time Clock
  static float *IMUValue; // array to store values from the Inertial Measurment Unit
  //String timeValues = ""; 
  static float *EnvironmentValue; // array to store values from the Environement Sensor
  static float *Distances;

  static String *JsonDataToSend;

  // restant timers if they fill up
  if (timerGPS > millis()) timerGPS = millis();
  if (timerIMU > millis()) timerGPS = millis();
  if (timerEnvironment > millis()) timerEnvironment = millis();
  if (timerDistanceTravelled > millis()) timerDistanceTravelled = millis();
  if (timerPrintData > millis()) timerPrintData = millis();
  if (timerSendBLE > millis()) timerSendBLE = millis();


  // UPDATE TIME (5 times per second)
  TimeValue = UpdateTime();

  // DISTANCE // ROTATIONS // SPEED // ACCELERATION  
  if (millis() - timerDistanceTravelled > 50) { // method called once every 50 milliseconds
    oldTime = millis();
    timerDistanceTravelled = millis(); // reset the timer
    Distances = DistanceTravelled(); // update the distance
    intervalDistance = Distances[0] - oldInterval;
    intervalEncoderRotations = Distances[1] - oldRotations;

    if (sentIntervalBleData == true) {
      oldInterval = Distances[0]; // reset the interval distacne if the data has been sent
      sentIntervalBleData = false;
    }
    IMURotations = getIMURotationDifference(); // update the IMU rotation difference

    if (IMURotations != -1) { // only use if calibrated
      totalRotations = totalRotations + ((intervalEncoderRotations + IMURotations)/2);
    } else {
      totalRotations = totalRotations + intervalEncoderRotations; // if IMU not calibrated then just use encoders
    }

    currentSpeed = getSpeed(Distances[0], oldDistance, millis(), oldTime); // claculate the speed
    currentAcceleration = getAcceleration(currentSpeed, millis(), oldTime, IMUValue[3], IMUValue[5]); // calcualte the acceleration
    oldDistance = Distances[0];
    oldRotations = Distances[1];    
  }


  // GPS
  if (millis() - timerGPS > 200) {
    //Serial.print("GPS: ");
    timerGPS = millis(); // reset the timer
    GPSSpeed = GetGPSData() / 1.944; //Get GPS Data and convert to metres per second
    if (GPSSpeed != -1) { // gps has signal
      //Serial.print("Singal  ");
      //Serial.print(","); Serial.print(GPSSpeed); Serial.print(",");
    }
  }

  // INCLINATION DATA
  if (millis() - timerIMU > 10) {
    //Serial.print("IMU: ");
    timerIMU = millis(); // reset the timer
    IMUValue = GetInertailData(); // Get acceleromter data
    //currentAcceleration = getAcceleration(currentSpeed, millis(), oldTime, IMUValue[3], IMUValue[5]);
  }

  //ENVIRONMENT
  if (millis() - timerEnvironment > 5000) {
    //Serial.print("Environment:");
    timerEnvironment = millis(); // reset the timer
    EnvironmentValue = GetEnvironmentSensorData();    // Get Environment sensor data every 1 second;
  }

  //DATA REQUEST
  incommingRequest = CheckForIncommingRequest();
  if (incommingRequest) {
    JsonDataToSend = prepareBleData(intervalDistance);
    dataToSend = true;
  }

  //SEND DATA
  if (dataToSend) {
    sentIntervalBleData = SendRequestedData(JsonDataToSend);
    if (sentIntervalBleData == true) {
      dataToSend = false; // data has been sent
    }
  }

  //PRINT DATA
  if (millis() - timerPrintData > 20) { // control time for data granularity
    timerPrintData = millis(); // reset the timer
    PrintDataToSerial(TimeValue, Distances, totalRotations, intervalDistance, currentSpeed, currentAcceleration
    , distance15Sec, IMUValue, EnvironmentValue, incommingRequest); // print all data to the serial console
  }
}
