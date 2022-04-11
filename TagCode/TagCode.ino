/*
   Copyright (c) 2020 by Ashutosh Dhekne <dhekne@gatech.edu>
   Peer-peer protocol

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

   @file PeerProtocol_test01.ino


*/

#include <SPI.h>
#include <math.h>
//#include <DW1000.h>
//#include "ProtocolConsts.h"
#include "genericFunctions.h"
#include "RangingContainer.h"
//#include "LocationContainer.h"
#include "Adafruit_LSM9DS1.h"
#include <SdFat.h>
#include <time.h>
#include<TimeLib.h>
#include "RTClib.h"
#include<Wire.h>

#define VBATPIN A2
#define LED_PIN 12
#define NOISE_PIN 11
#define GOOD_PIN 6
#define SILENCE_PIN 5
#define DEV_INDICATOR_PIN 13

int AlarmNoise = 0;// = rx_packet[LED_IDX] & 0x01;
int AlarmLed = 0;// = rx_packet[LED_IDX] & 0x02;

#define INIT_RTC_ALWAYS 0

#define TAG 1
#define USB_CONNECTION 0
#define PRINT_CIR 1
#define INITIATOR 0
#define IGNORE_IMU 1
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
#define IMU_READINGS_MAX 18*4
byte imu_buffer[IMU_READINGS_MAX];
#define IMU_SINGLE_READING_BUFF_SIZE 6
int disable_imu = 1;
//int chck =2390;

#define DEBUG_PRINT 1
// connection pins
#define OUR_UWB_FEATHER 1
#define AUS_UWB_FEATHER 0


#if(OUR_UWB_FEATHER==1)
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 17; // irq pin
const uint8_t PIN_SS = 19; // spi select pin
#endif
/*
  #if(AUS_UWB_FEATHER==1)
  const uint8_t PIN_RST = 2; // reset pin
  const uint8_t PIN_IRQ = 3; // irq pin
  const uint8_t PIN_SS = 4; // spi select pin
  #endif
*/


// DEBUG packet sent status and count
volatile boolean received = false;
volatile boolean error = false;
volatile int16_t numReceived = 0; // todo check int type
volatile boolean sendComplete = false;
volatile boolean RxTimeout = false;
String message;

byte tx_poll_msg[MAX_POLL_LEN] = {POLL_MSG_TYPE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte rx_resp_msg[MAX_RESP_LEN] = {RESP_MSG_TYPE, 0x02, 0, 0, 0, 0, 0};
byte tx_final_msg[MAX_FINAL_LEN] = {FINAL_MSG_TYPE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int response_counter = 0;
int num_nodes = 3;

#define msg_len 700

int packet_count = 1;

char rx_msg_char[msg_len];

Ranging thisRange[15];
float anchor_fp_pw[15];
uint32_t ToA_mat[15][15];
//Location location;

int anchors[30][2];

byte rx_packet[MAX_POLL_LEN];
uint8_t myAcc[1000];

int ACC_BUFFER_SIZE = 300;
int ACC_DATA_COUNT = ACC_BUFFER_SIZE/10;


typedef enum states {STATE_IDLE, STATE_POLL, STATE_RESP_EXPECTED, STATE_FINAL_SEND, STATE_TWR_DONE, STATE_RESP_SEND, STATE_FINAL_EXPECTED, STATE_OTHER_POLL_EXPECTED, STATE_RESP_PENDING, STATE_DIST_EST_EXPECTED, STATE_DIST_EST_SEND, STATE_TIGHT_LOOP,
                     STATE_RECEIVE, STATE_PRESYNC, STATE_SYNC, STATE_ANCHOR, STATE_TAG, STATE_FIRST_START, STATE_OBLIVION, STATE_ACK_EXPECTED
                    } STATES;
volatile uint8_t current_state = STATE_IDLE;
unsigned long silenced_at = 0;
#define SILENCE_PERIOD 120
long randNumber;
int currentSlots = 8;
int myDevID = INITIATOR ? 0 : INITIATOR + 1;

typedef struct Acc_Data {
  long timestamp;
  int acc_x;
  int acc_y;
  int acc_z;
};

//Timer for implementing timeouts
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

void startTimer(int frequencyHz);
void setTimerFrequency(int frequencyHz);

void TC3_Handler();

#define MAX_TIMEOUTS 2
volatile boolean timeout_established[MAX_TIMEOUTS];
volatile boolean timeout_triggered[MAX_TIMEOUTS];
volatile boolean timeout_overflow[MAX_TIMEOUTS];
volatile uint64_t timeout_time[MAX_TIMEOUTS];

SdFat sd;
SdFile store_distance; //The root file system useful for deleting all files on the SDCard
char filename[14];
int filenum = 0;
int entries_in_file = 0;
int SDChipSelect = 10;

int SDEnabled = 0;

int CIR_count = 0;

#define DIST_ALARM 1500
#define DIST_WARN 2500

//Time
RTC_PCF8523 rtc;

typedef struct DeviceRespTs {
  int deviceID;
  uint64_t respRxTime;
};

#define MAX_DEVICES_TOGETHER 20
DeviceRespTs deviceRespTs[MAX_DEVICES_TOGETHER];
int currentDeviceIndex = 0;


void receiver(uint16_t rxtoval = 0 ) {
  received = false;
  DW1000.newReceive();
  //  DW1000.setDefaults();
  DW1000.setDefaults_longMsg();

  // we cannot don't need to restart the receiver manually
  DW1000.receivePermanently(false);
  if (rxtoval > 0) {
    DW1000.setRxTimeout(rxtoval);
  } else {
    //Serial.print("Resetting Timeout to  ");
    //Serial.println(rxtoval);
    DW1000.setRxTimeout(rxtoval);
  }
  DW1000.startReceive();
  //Serial.println("Started Receiver");
}


//void receiver_perm(void ) {
//  received = false;
//  DW1000.newReceive();
//  DW1000.setDefaults();
//  // we cannot don't need to restart the receiver manually
//  DW1000.receivePermanently(true);
//
//  DW1000.startReceive();
//  //Serial.println("Started Receiver");
//}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(NOISE_PIN, OUTPUT);
  pinMode(GOOD_PIN, OUTPUT);
  pinMode(DEV_INDICATOR_PIN, OUTPUT);
  pinMode(SILENCE_PIN, INPUT_PULLUP);
  digitalWrite(GOOD_PIN, HIGH);
  analogReadResolution(10);
  // DEBUG monitoring
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
#if(USB_CONNECTION==0)
    break;
#endif
  }
  Serial.print("Waiting...");
  delay(5000);
  Serial.print("Should see this...");
  //Setting up the RTC Clock
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    //while (1);
  }

  if (! rtc.initialized()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Serial.println("Setting new time");
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    rtc.adjust(DateTime(2020, 10, 17, 19, 40, 0));
  }

  //In production, INIT_RTC_ALWAYS should be 0.
  //Only turn this to 1 when testing
#if (INIT_RTC_ALWAYS == 1)
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
#endif
  //rtc.adjust(DateTime(2019, 11, 13, 10, 06, 00));
  //SoftRTC.begin(rtc.now());  // Initialize SoftRTC to the current time

  //while(1) {
  Serial.println("Current Time");
  DateTime now = rtc.now();
  Serial.print(now.year());
  Serial.print("/");
  Serial.print(now.month());
  Serial.print("/");
  Serial.print(now.day());
  Serial.print(" ");
  Serial.print(now.hour());
  Serial.print(":");
  Serial.print(now.minute());
  Serial.print(":");
  Serial.print(now.second());
  delay(1000);
  //}
  Serial.print("Initializing SD card...");
  //delay(1000);
  if (!sd.begin(SDChipSelect, SPI_FULL_SPEED)) {
    Serial.println("SDCard Initialization failed!");
    SDEnabled = 0;
  } else {
    Serial.println("SDCard Initialization done.");
    SDEnabled = 1;
  }

  if (SDEnabled == 1) {
    sprintf(filename, "dist%03d.txt", filenum);
    if (!store_distance.open(filename, O_WRITE | O_CREAT)) {
      Serial.println("Could not create file");
      delay(10000);
    }
  }
  randomSeed(analogRead(0));
  Serial.println(F("Peer-peer ranging protocol"));
  Serial.println("Free memory: ");
  Serial.println(freeMemory());
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  Serial.println(F("DW1000 initialized ..."));
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults_longMsg();
  //  DW1000.setDefaults();

  DW1000.setDeviceAddress(6);
  DW1000.setNetworkId(10);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  DW1000.commitConfiguration();
  Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);
  // attach callback for (successfully) received messages
  DW1000.attachReceivedHandler(handleReceived);
  DW1000.attachReceiveTimeoutHandler(handleRxTO);
  DW1000.attachReceiveFailedHandler(handleError);
  DW1000.attachErrorHandler(handleError);
  DW1000.attachSentHandler(handleSent);
  // start reception

#if (INITIATOR==0)
  disable_imu = 1;
  receiver(0);

#endif
  current_state = STATE_IDLE;

  for (int i = 0; i < MAX_TIMEOUTS; i++) {
    timeout_established[i] = false;
    timeout_triggered[i] = false;
    timeout_overflow[i] = false;
    timeout_time[i] = 0;
  }

#if (INITIATOR == 1)
  digitalWrite(DEV_INDICATOR_PIN, 1);
  delay(500);
  digitalWrite(DEV_INDICATOR_PIN, 0);
  delay(500);
  digitalWrite(DEV_INDICATOR_PIN, 1);
  delay(500);
  digitalWrite(DEV_INDICATOR_PIN, 0);
  delay(500);
  digitalWrite(DEV_INDICATOR_PIN, 1);
  delay(5000);
  digitalWrite(DEV_INDICATOR_PIN, 0);
  //delay(500);
#else
  digitalWrite(DEV_INDICATOR_PIN, 1);
  delay(1000);
  digitalWrite(DEV_INDICATOR_PIN, 0);
  delay(500);
  digitalWrite(DEV_INDICATOR_PIN, 1);
  delay(200);
  digitalWrite(DEV_INDICATOR_PIN, 0);
  delay(500);
  digitalWrite(DEV_INDICATOR_PIN, 1);
  delay(500);
  digitalWrite(DEV_INDICATOR_PIN, 0);
#endif

#if (INITIATOR==1)
  while (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    delay(1000);
#if IGNORE_IMU==1
    disable_imu = 1;
    break;
#endif
  }
  if (disable_imu == 0) {
    Serial.println("Found LSM9DS1 9DOF");
    // 1.) Set the accelerometer range
    lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
    //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
    //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
    //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

    // 2.) Set the magnetometer sensitivity
    lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
    //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
    //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
    //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

    // 3.) Setup the gyroscope
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
    //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
    //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
  }
#endif

  //startTimer(100);

  receiver(0);

}



void handleSent() {
  // status change on sent success
  sendComplete = true;
  //Serial.println("Send complete");
}


void handleReceived() {
  // status change on reception success

  DW1000.getData(rx_packet, DW1000.getDataLength());

    Serial.println("Received something...");
  received = true;
  //  show_packet_8B(rx_packet);

}

void handleError() {
  error = true;
  Serial.println("Error ");

  receiver(0);
}

void handleRxTO() {
  current_state = STATE_IDLE;
  RxTimeout = true;


#if (DEBUG_PRINT==1)
  Serial.println("Rx Timeout");
  Serial.println("State: ");
  Serial.println(current_state);
#endif
  receiver(0);
}


float gravity_f = 0.0f;
uint16_t seq = 0;
uint16_t recvd_poll_seq = 0;
uint16_t recvd_resp_seq = 0;
uint16_t recvd_final_seq = 0;

volatile int unlock_waiting = 0;
long loop_counter = 0;
volatile long waiting = 0;
DW1000Time currentDWTime;
uint64_t currentTime;
uint64_t final_sent_time;
uint64_t init_time;
double elapsed_time;
double TIME_UNIT = 1.0 / (128 * 499.2e6); // seconds

double start_time_us = 0, current_time_us = 0;

//Added
int same_init_count = 1;
uint8_t previous_INIT = 0;


void print_CIR_POWER()
{
  float RX_POWER = DW1000.getReceivePower();
  float FP_POWER = DW1000.getFirstPathPower();
  char buff[70];
  sprintf(buff, "RX_PW=%f dbm, FP_PW=%f dbm", RX_POWER, FP_POWER);
  Serial.println(buff);

  // float FP_POWER = DW1000.getFirstPathPower();
  //  char buff[40];
  //  sprintf(buff, "FP_PW=%f dbm", FP_POWER);
  //  Serial.println(buff);
}
/*

  //Yunzhi CIR Code
  void print_CIR()
  {
  char buff[140];
  char long_buff[1400];
  uint8_t myAcc[4 * NCIR_FULL + 6];
  int starttap = 720;
  int endtap = 816;
  int16_t RealData = 0;
  int16_t ImaginaryData = 0;

    byte firstPath[2];
    DW1000.readBytes(RX_TIME, RX_TIME_FP_INDEX_OFFSET, firstPath, 2);
    uint16_t firstpath = uint16_t(firstPath[1]<<8 | firstPath[0]);
    uint16_t firstPath_integer = (firstpath & 0xFFC0) >> 6;
    uint16_t firstPath_fraction = (firstpath & 0x003F);
    float RX_POWER = DW1000.getReceivePower();
    float FP_POWER = DW1000.getFirstPathPower();
  //    Serial.print(packet_count);
  //    Serial.print(",2,3,4,5,6,7,");
  //    Serial.print(firstPath_integer);
  //    Serial.print(",");
  //    Serial.print(firstPath_fraction);
  //    Serial.println(",10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26, 4 d00");

    sprintf(buff, "FP_IDX[%d,%d,%d]   FP_POWER[%d,%f,%f]", packet_count, firstPath_integer, firstPath_fraction, packet_count,RX_POWER,FP_POWER);
  //    sprintf(buff, "FP_IDX[%d,%d,%d,%d]  FP_POWER[%d,%d,%f,%f]", packet_type,packet_count, firstPath_integer, firstPath_fraction, packet_type, packet_count,RX_POWER,FP_POWER);
    Serial.println(buff);

    strcpy(long_buff, "CIR");
    DW1000.getAccMem(myAcc, 0, endtap + 1); //myAcc will contain 16 bit real imaginary pairs

    for (int i = starttap ; i < endtap; i++) {
      RealData = myAcc[(i * 4) + 2] << 8 | myAcc[(i * 4) + 1];
      ImaginaryData = myAcc[(i * 4) + 4] << 8 | myAcc[(i * 4) + 3];
      // int16_t RealData = myAcc[(i * 4) + 1] << 8 | myAcc[(i * 4)];
      // int16_t ImaginaryData = myAcc[(i * 4) + 3] << 8 | myAcc[(i * 4) + 2];
      //Question
      sprintf(buff, "[%d,%d,%d,d%d]", packet_count, RealData, ImaginaryData, i + 1);
      strcat(long_buff, buff);
    }
    Serial.println(long_buff);


  //  strcpy(long_buff, "CIR");
  //  DW1000.getAccMem(myAcc, starttap, endtap + 1); //myAcc will contain 16 bit real imaginary pairs
  //
  //  for (int i = 0 ; i < endtap+1-starttap; i++) {
  //    RealData = myAcc[(i * 4) + 2] << 8 | myAcc[(i * 4) + 1];
  //    ImaginaryData = myAcc[(i * 4) + 4] << 8 | myAcc[(i * 4) + 3];
  //    // int16_t RealData = myAcc[(i * 4) + 1] << 8 | myAcc[(i * 4)];
  //    // int16_t ImaginaryData = myAcc[(i * 4) + 3] << 8 | myAcc[(i * 4) + 2];
  //    //Question
  //    sprintf(buff, "[%d,%d,%d,d%d]", packet_count, RealData, ImaginaryData, i + 1 + starttap);
  //    strcat(long_buff, buff);
  //  }
  //  Serial.println(long_buff);



  //   for (int i = 0 ; i< (endtap - starttap + 1); i++){
  //  RealData = myAcc[(i * 4) + 2] << 8 | myAcc[(i * 4) + 1];
  //  ImaginaryData = myAcc[(i * 4) + 4] << 8 | myAcc[(i * 4) + 3];
  //   // int16_t RealData = myAcc[(i * 4) + 1] << 8 | myAcc[(i * 4)];
  //  // int16_t ImaginaryData = myAcc[(i * 4) + 3] << 8 | myAcc[(i * 4) + 2];
  //   //Question
  //   sprintf(buff, "%d,%d,%d,d%d\n", packet_count, RealData, ImaginaryData, i + 1 + starttap);
  //   Serial.write(buff);
  // }

  }

*/
void my_generic_receive(void)
{
  if (RxTimeout == true) {
    //    Serial.println("RxTimeout == true");
    receiver(0);
    RxTimeout = false;
  }

  if (received)
  {
//    Serial.print("Rx, i=0: ");
//    Serial.println(rx_packet[0]);
    received = false;
//    byte2char(rx_packet, MAX_POLL_LEN);
//      Serial.println(rx_msg_char);
    //    receiver(0);


    if (rx_packet[0] == POLL_MSG_TYPE)
    {
      Serial.println("Poll received");
      DW1000Time rxTS;
      DW1000.getReceiveTimestamp(rxTS);
      receiver(0);
      CIR_count = 0;
      byte2char(rx_packet, 30);
      Serial.println(rx_msg_char);
      seq = rx_packet[SEQ_IDX] +  ((uint16_t)rx_packet[SEQ_IDX + 1] << 8);
      //      char buff[50];
      //      sprintf(buff, "Pkt %u: ", seq);
      //      Serial.print(buff);
      //      print_CIR_POWER();

      digitalWrite(LED_PIN, LOW);
      if (seq % 300 == 0)
        digitalWrite(LED_PIN, HIGH);

      currentDeviceIndex = rx_packet[SRC_IDX];


      //      if (PRINT_CIR)
      //      {
      //        packet_count = seq;
      //        print_CIR();
      //      }

      //Check whether from the same INITIATOR
      if (currentDeviceIndex == previous_INIT)
      {
        same_init_count++;
        //          Serial.println(same_init_count);

      } else {
        same_init_count = 1;
      }
      previous_INIT = currentDeviceIndex;


      //float IMU_data[200];
      //float num_of_IMU_data;
      //TODO: why is num IMU data a float?
      int num_of_IMU_data;
      num_of_IMU_data = rx_packet[POLL_MSG_IMU_NUM_IDX];

//      Serial.println((int)num_of_IMU_data * sizeof(float));
      struct Acc_Data currentAcc_Data[ACC_DATA_COUNT];
      if (num_of_IMU_data>0)
      {
        //while ((num_of_IMU_data--) > 0)
        //{
          memcpy(&currentAcc_Data, &rx_packet[POLL_MSG_IMU_DATA_IDX], sizeof(Acc_Data)*ACC_DATA_COUNT);
        //}
      }
      
//      byte2char(rx_packet, 30);
//      Serial.println(rx_msg_char);
      
      //Serial.println(num_of_IMU_data);
//      char IMU_buff[500];
//      strcpy(IMU_buff, "POLL, ");
//      for (int i = 0; i < (int)num_of_IMU_data; i++)
//      {
//        char temp[30];
//        //          sprintf(temp, "t: %0.1f,  acc_x: %0.2f \n", IMU_data[7*i+1]);
//        sprintf(temp, "%0.4f, ", IMU_data[i] );
//        strcat(IMU_buff, temp);
//        if (i % 4 == 3 && i<num_of_IMU_data-1)
//          strcat(IMU_buff, "\nPOLL, ");
//      }
//      Serial.println(IMU_buff);

      uint64_t PollTxTime_64 = 0L;
      any_msg_get_ts(&rx_packet[POLL_MSG_POLL_TX_TS_IDX], &PollTxTime_64);
      num_nodes = (int)rx_packet[POLL_MSG_POLL_NUM_RESP_IDX];
      //Serial.println(num_nodes);

      for (int i = 0; i < num_nodes; i++)
      {
        thisRange[i].initialize();
        thisRange[i].prev_seq = thisRange[i].seq;

        thisRange[i].PollTxTime = DW1000Time((int64_t)PollTxTime_64);
        thisRange[i].PollRxTime_T = rxTS;
      }
      response_counter = 0;

      float volts = getVoltage();
      if (SDEnabled)
      {
        store_distance.print("Voltage: ");
        store_distance.println(volts);
        if (SDEnabled)
        {
          entries_in_file++;
          if (entries_in_file > 100)
          {
            entries_in_file = 0;
            Serial.println("Changed files");
            SdFile::dateTimeCallback(dateTime);

            store_distance.close();
            filenum++;
            sprintf(filename, "dist%03d.txt", filenum);
            if (!store_distance.open(filename, O_WRITE | O_CREAT)) {
              Serial.println("Could not create file");
              delay(1000);
            }

          }
        }
      }

    }
    else if (rx_packet[0] == RESP_MSG_TYPE)
    {

      //      byte2char(rx_packet, 40);
      //      Serial.println(rx_msg_char);
      DW1000Time rxTS;
      DW1000.getReceiveTimestamp(rxTS);
      receiver(0);
      currentDeviceIndex = rx_packet[SRC_IDX];
      recvd_resp_seq = rx_packet[SEQ_IDX] +  ((uint16_t)rx_packet[SEQ_IDX + 1] << 8);
      //Serial.println(currentDeviceIndex);
      anchor_fp_pw[currentDeviceIndex] = DW1000.getFirstPathPower();

      if (recvd_resp_seq == seq)
      {
        CIR_count++;

        for (int i = 0; i < num_nodes; i++)
        {
          if (thisRange[i].device_ID == rx_packet[SRC_IDX])
          {

            uint64_t prev_DB = 0L, prev_RB = 0L;
            any_msg_get_ts(&rx_packet[RESP_MSG_PREV_DB_IDX], &prev_DB);
            any_msg_get_ts(&rx_packet[RESP_MSG_PREV_RB_IDX], &prev_RB);
            thisRange[i].prev_Db = DW1000Time((int64_t)prev_DB);
            thisRange[i].prev_Rb = DW1000Time((int64_t)prev_RB);

            if (thisRange[i].prev_seq + 1 == recvd_resp_seq && same_init_count > 1) {
              int range = thisRange[i].calculateTDoARange();
              uint32_t ToA = thisRange[i].calculateRange();

              ToA_mat[currentDeviceIndex][previous_INIT] = (uint32_t)(0.8 * ToA_mat[currentDeviceIndex][previous_INIT] + 0.2 * ToA);
              ToA_mat[previous_INIT][currentDeviceIndex] = (uint32_t)(0.8 * ToA_mat[previous_INIT][currentDeviceIndex] + 0.2 * ToA);

              char buff[200];
              sprintf(buff, "Pkt %d - ToA %d to %d: %u mm", recvd_resp_seq, currentDeviceIndex, previous_INIT, ToA);
              Serial.println(buff);
              /*
                //              uint16_t RX_POWER_fp = (uint16_t)rx_packet[RESP_MSG_POLL_PW_IDX] << 8 + rx_packet[RESP_MSG_POLL_PW_IDX + 1];
                //              uint16_t FP_POWER_fp = (uint16_t)rx_packet[RESP_MSG_POLL_PW_IDX + 2] << 8 + rx_packet[RESP_MSG_POLL_PW_IDX + 3];
                //
                //              float RX_POWER = fixed_point_to_float(RX_POWER_fp);
                //              float FP_POWER = fixed_point_to_float(FP_POWER_fp);

                            //            sprintf(buff, "%d to %d: FP_PW=%u dBm; RX_PW=%u dBm;", currentDeviceIndex, previous_INIT, RX_POWER_fp, FP_POWER_fp);
                            //            Serial.println(buff);
                //
                //              sprintf(buff, "Pkt %u - %d to %d: FP_PW=%f dBm; RX_PW=%f dBm; TDoA %d to %d: %d mm; ToA %d to %d: %u mm", recvd_resp_seq, currentDeviceIndex, previous_INIT, FP_POWER, RX_POWER, currentDeviceIndex, previous_INIT, range, currentDeviceIndex, previous_INIT, ToA);
                //              Serial.println(buff);
              */
              digitalWrite(LED_PIN, HIGH);

            }

            break;
          }
        }

        deviceRespTs[response_counter].deviceID = currentDeviceIndex;
        thisRange[response_counter].device_ID = currentDeviceIndex;
        thisRange[response_counter].RespRxTime_T = rxTS;

        response_counter++;
      }
    }
    /*
      else if(rx_packet[0] == RESP_MSG_TYPE)
      {
      receiver(0);
      //      byte2char(rx_packet, 40);
      //      Serial.println(rx_msg_char);
      DW1000Time rxTS;
      DW1000.getReceiveTimestamp(rxTS);
      currentDeviceIndex = rx_packet[SRC_IDX];
      recvd_resp_seq = rx_packet[SEQ_IDX] +  ((uint16_t)rx_packet[SEQ_IDX+1] << 8);
      Serial.println(currentDeviceIndex);
      int32_t anc_loc_x,anc_loc_y;
      //      char loc_msg[50];
      //      any_msg_get_location(&rx_packet[RESP_MSG_SELF_LOCATION_X],&anc_loc_x);
      //      any_msg_get_location(&rx_packet[RESP_MSG_SELF_LOCATION_Y],&anc_loc_y);
      //      sprintf(loc_msg, "Anchor %u [%d,%d]",(uint8_t)rx_packet[SRC_IDX], anc_loc_x, anc_loc_y);
      //      Serial.println(loc_msg);

      if(recvd_resp_seq==seq)
      {
      CIR_count++;

      for(int i=0;i<num_nodes;i++)
      {
      if(thisRange[i].device_ID==rx_packet[SRC_IDX])
      {

        uint64_t prev_DB=0L, prev_RB=0L;
        any_msg_get_ts(&rx_packet[RESP_MSG_PREV_DB_IDX], &prev_DB);
        any_msg_get_ts(&rx_packet[RESP_MSG_PREV_RB_IDX], &prev_RB);
        thisRange[response_counter].prev_Db = DW1000Time((int64_t)prev_DB);
        thisRange[response_counter].prev_Rb = DW1000Time((int64_t)prev_RB);

        if(thisRange[response_counter].prev_seq+1 == recvd_resp_seq && same_init_count>1){
          int range = thisRange[response_counter].calculateTDoARange();
          uint32_t ToA = thisRange[response_counter].calculateRange();
          char buff[200];
      //            sprintf(buff, "Pkt %d - TDoA %d to %d: %d mm; ToA %d to %d: %u mm", recvd_resp_seq, currentDeviceIndex, previous_INIT, range,currentDeviceIndex, previous_INIT, ToA);
      //            Serial.println(buff);

          uint16_t RX_POWER_fp = (uint16_t)rx_packet[RESP_MSG_POLL_PW_IDX]<<8 + rx_packet[RESP_MSG_POLL_PW_IDX+1];
          uint16_t FP_POWER_fp = (uint16_t)rx_packet[RESP_MSG_POLL_PW_IDX+2]<<8 + rx_packet[RESP_MSG_POLL_PW_IDX+3];

          float RX_POWER = fixed_point_to_float(RX_POWER_fp);
          float FP_POWER = fixed_point_to_float(FP_POWER_fp);

      //            sprintf(buff, "%d to %d: FP_PW=%u dBm; RX_PW=%u dBm;", currentDeviceIndex, previous_INIT, RX_POWER_fp, FP_POWER_fp);
      //            Serial.println(buff);

          sprintf(buff, "Pkt %u - %d to %d: FP_PW=%f dBm; RX_PW=%f dBm; TDoA %d to %d: %d mm; ToA %d to %d: %u mm", recvd_resp_seq, currentDeviceIndex, previous_INIT, FP_POWER, RX_POWER, currentDeviceIndex, previous_INIT, range,currentDeviceIndex, previous_INIT, ToA);
          Serial.println(buff);
          print_CIR_POWER();


      //            if (PRINT_CIR)
      //            {
      //              packet_count = recvd_resp_seq;
      //              print_CIR();
      //            }
          digitalWrite(LED_PIN,HIGH);

        }
        break;
      }
      }

      deviceRespTs[response_counter].deviceID = currentDeviceIndex;
      thisRange[response_counter].device_ID = currentDeviceIndex;
      thisRange[response_counter].RespRxTime_T = rxTS;

      response_counter++;
      }
      }
    */
    else if (rx_packet[0] == FINAL_MSG_TYPE)
    {
//      Serial.println("FINAL");
      //      char buff[50];
      //      sprintf(buff, "Pkt %u: ", seq);
      //      Serial.println(buff);
      CIR_count = 0;

      receiver(0);
      currentDeviceIndex = rx_packet[SRC_IDX];
      recvd_resp_seq = rx_packet[SEQ_IDX] +  ((uint16_t)rx_packet[SEQ_IDX + 1] << 8);
      DW1000Time rxTS;
      DW1000.getReceiveTimestamp(rxTS);
      //      float IMU_ts,IMU_acc_x;
      //      memcpy(&IMU_ts, &rx_packet[FINAL_MSG_IMU_DATA_IDX],sizeof(float));
      //      memcpy(&IMU_acc_x, &rx_packet[FINAL_MSG_IMU_DATA_IDX+4],sizeof(float));
      //      char buff[40];
      //      sprintf(buff, "At %f ms, accX: %f m/s2",IMU_ts,IMU_acc_x);
      //      Serial.println(buff);

      /*float IMU_data[200];
      float num_of_IMU_data;
      memcpy(&num_of_IMU_data, &rx_packet[FINAL_MSG_IMU_DATA_IDX], sizeof(float));

//      Serial.println((int)num_of_IMU_data * sizeof(float));
      if (num_of_IMU_data > 0)
      {
        memcpy(IMU_data, &rx_packet[FINAL_MSG_IMU_DATA_IDX + 4], (int)num_of_IMU_data * sizeof(float));
      }
      Serial.println(num_of_IMU_data);
      char IMU_buff[500];
      strcpy(IMU_buff, "FIN, ");
      for (int i = 0; i < (int)num_of_IMU_data; i++)
      {
        char temp[30];
        //          sprintf(temp, "t: %0.1f,  acc_x: %0.2f \n", IMU_data[7*i+1]);
        sprintf(temp, "%0.4f, ", IMU_data[i] );
        strcat(IMU_buff, temp);
        if (i % 4 == 3 && i<num_of_IMU_data-1)
          strcat(IMU_buff, "\nFIN, ");
      }
      Serial.println(IMU_buff);
*/

      //      byte2char(rx_packet, 60);
      //      Serial.println(rx_msg_char);
      //        Serial.println("=================================");
      if (recvd_resp_seq == seq)
      {
        uint64_t FinalTxTime_64 = 0L;
        any_msg_get_ts(&rx_packet[FINAL_MSG_FINAL_TX_TS_IDX], &FinalTxTime_64);

        for (int i = 0; i < response_counter; i++)
        {

          for (int j = 0; j < num_nodes; j++)
          {
            if (thisRange[i].device_ID == rx_packet[FINAL_MSG_RESP_IDX + FINAL_MSG_ONE_RESP_ENTRY * j])
            {

              thisRange[i].seq = recvd_resp_seq;
              thisRange[i].FinalTxTime = DW1000Time((int64_t)FinalTxTime_64);
              thisRange[i].FinalRxTime_T = rxTS;
              //                Serial.println(j);
              uint64_t RespRxTime_64 = 0L;
              any_msg_get_ts(&rx_packet[FINAL_MSG_RESP_RX_TS_IDX + FINAL_MSG_ONE_RESP_ENTRY * j], &RespRxTime_64);
              thisRange[i].RespRxTime = DW1000Time((int64_t)RespRxTime_64);
              thisRange[i].calculate_TIME();
              int32_t T_AB_minus_rAB = thisRange[i].calculateTDoARange2();
              //              char buff[200];
              //              sprintf(buff, "Pkt %u - %d to %d: FP_PW=%f dBm; TDoA %d to %d: %d mm;", recvd_resp_seq, thisRange[i].device_ID, currentDeviceIndex, anchor_fp_pw[thisRange[i].device_ID], thisRange[i].device_ID, currentDeviceIndex, T_AB_minus_rAB + ToA_mat[thisRange[i].device_ID][currentDeviceIndex]);
              //              Serial.println(buff);
              //print_CIR_POWER();
              break;
            }
          }

        }

      }
      response_counter = 0;

    }
  }
}
//
//int count = 0;
//int true_range[4] = {7338,8628,5495,7168};
//void imitate_ToA()
//{
//    int pkd = count/4;
//    char buff[200];
//
//    switch(count%4)
//    {
//      case 0:
//          sprintf(buff, "Pkt %d - ToA %d to 0: %d mm", pkd, count%4+1, (int)true_range[count%4]+random(0,40)-20);
//      break;
//      case 1:
//          sprintf(buff, "Pkt %d - ToA %d to 0: %d mm", pkd, count%4+1, (int)true_range[count%4]+random(0,40)-20);
//
//      break;
//      case 2:
//          sprintf(buff, "Pkt %d - ToA %d to 0: %d mm", pkd, count%4+1, (int)true_range[count%4]+random(0,40)-20);
//      break;
//      case 3:
//          sprintf(buff, "Pkt %d - ToA %d to 0: %d mm", pkd, count%4+1, (int)true_range[count%4]+random(0,40)-20);
//      break;
//    }
//    count++;
//    Serial.println(buff);
//    delay(20);
//
//}

void loop()
{
  //  Serial.println("Pkt 50 - ToA 1 to 0: 800 mm");
  my_generic_receive();
  //imitate_ToA();

}


void show_packet(byte packet[], int num) {
#if (DEBUG_PRINT==1)
  for (int i = 0; i < num; i++) {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println(" ");
#endif

}


void show_packet_8B(byte packet[]) {

  long msg1 = 0;
  long msg2 = 0;
#if (DEBUG_PRINT==1)
  for (int i = 0; i < 4; i++) {
    msg1 = msg1 << 8;
    msg2 = msg2 << 8;
    msg1 += packet[i];
    msg2 += packet[i + 4];
  }
  Serial.print(msg1, HEX);
  Serial.println(msg2, HEX);

#endif

}

void byte2char(byte packet[], int len) {

  const char * hex = "0123456789ABCDEF";
  char msg[msg_len];
  int i = 0;
  for (; i < len;) {
    msg[2 * i] = hex[packet[i] >> 4 & 0x0F];
    msg[2 * i + 1] = hex[packet[i] & 0x0F];
    i = i + 1;
  }
  msg[2 * i] = '\0';
  strcpy(rx_msg_char, msg);
}

//Timer Functions
//Timer functions.
void setTimerFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;

  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  //Serial.println(TC->COUNT.reg);
  //Serial.println(TC->CC[0].reg);
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void startTimer(int frequencyHz) {

  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;

  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  setTimerFrequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;


  NVIC_SetPriority(TC3_IRQn, 3);
  NVIC_EnableIRQ(TC3_IRQn);


  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}


void collect_imu_data(byte *imu_buff, int *imu_buffer_counter) {
  if (disable_imu == 0)
  {
    lsm.readBuffer(XGTYPE, 0x80 | lsm.LSM9DS1_REGISTER_OUT_X_L_XL, IMU_SINGLE_READING_BUFF_SIZE, &imu_buff[(*imu_buffer_counter)]);
    (*imu_buffer_counter) += IMU_SINGLE_READING_BUFF_SIZE;



    lsm.readBuffer(MAGTYPE, 0x80 | lsm.LSM9DS1_REGISTER_OUT_X_L_M, IMU_SINGLE_READING_BUFF_SIZE, &imu_buff[(*imu_buffer_counter)]);
    (*imu_buffer_counter) += IMU_SINGLE_READING_BUFF_SIZE;


    lsm.readBuffer(XGTYPE, 0x80 | lsm.LSM9DS1_REGISTER_OUT_X_L_G, IMU_SINGLE_READING_BUFF_SIZE, &imu_buff[(*imu_buffer_counter)]);
    (*imu_buffer_counter) += IMU_SINGLE_READING_BUFF_SIZE;
  } else {
    (*imu_buffer_counter) += IMU_SINGLE_READING_BUFF_SIZE * 3;
  }
}

void set_imu_data(float data[], int type) {
  float ax, ay, az;
  float acc_unit = 2 * 9.8 / 32768;
  float mag_unit = 0.14;
  float gyro_unit = 0.00875;
  int unit = 1;
  int imu_idx = 6 * type;
  if (type == 0) {
    data[0] = (int16_t)((imu_buffer[0 + imu_idx] + ((int16_t)imu_buffer[1 + imu_idx] << 8))) * acc_unit;
    data[1] = (int16_t)((imu_buffer[2 + imu_idx] + ((int16_t)imu_buffer[3 + imu_idx] << 8))) * acc_unit;
    data[2] = (int16_t)((imu_buffer[4 + imu_idx] + ((int16_t)imu_buffer[5 + imu_idx] << 8))) * acc_unit;
  } else if (type == 1) {
    data[0] = (int16_t)((imu_buffer[0 + imu_idx] + ((int16_t)imu_buffer[1 + imu_idx] << 8))) * mag_unit;
    data[1] = (int16_t)((imu_buffer[2 + imu_idx] + ((int16_t)imu_buffer[3 + imu_idx] << 8))) * mag_unit;
    data[2] = (int16_t)((imu_buffer[4 + imu_idx] + ((int16_t)imu_buffer[5 + imu_idx] << 8))) * mag_unit;
  } else {
    data[0] = (int16_t)((imu_buffer[0 + imu_idx] + ((int16_t)imu_buffer[1 + imu_idx] << 8))) * gyro_unit;
    data[1] = (int16_t)((imu_buffer[2 + imu_idx] + ((int16_t)imu_buffer[3 + imu_idx] << 8))) * gyro_unit;
    data[2] = (int16_t)((imu_buffer[4 + imu_idx] + ((int16_t)imu_buffer[5 + imu_idx] << 8))) * gyro_unit;
  }
}


void TC3_Handler()
{
  DW1000Time currectTS;
  uint64_t currentUWBTime;
  for (int i = 0; i < MAX_TIMEOUTS; i++) {
    if (timeout_established[i]) {
      DW1000.getSystemTimestamp(currectTS);
      currentUWBTime = currectTS.getTimestamp();
      break; //Any timeout if established will populate the currentUWBTime
    }
  }
  for (int i = 0; i < MAX_TIMEOUTS; i++) {
    if (timeout_established[i]) {
      if (currentUWBTime > timeout_time[i]) {
        timeout_established[i] = false;
        timeout_time[i] = INFINITE_TIME;
        timeout_overflow[i] = false;
        timeout_triggered[i] = true;
      } else if (timeout_overflow[i] == true && currentUWBTime > (timeout_time[i] - 2 ^ 40)) {
        timeout_established[i] = false;
        timeout_time[i] = INFINITE_TIME;
        timeout_overflow[i] = false;
        timeout_triggered[i] = true;
      }
    }
  }
}

void set_timeout(int whichTO, uint32_t delayTime) {
  DW1000Time currectTS;
  uint64_t currentUWBTime;
  DW1000.getSystemTimestamp(currectTS);
  currentUWBTime = currectTS.getTimestamp();
  DW1000Time deltaTime = DW1000Time(delayTime, DW1000Time::MILLISECONDS);
  timeout_time[whichTO] = (currectTS + deltaTime).getTimestamp();
  if (timeout_time[whichTO] > 2 ^ 40) {
    timeout_overflow[whichTO] = true;
  } else {
    timeout_overflow[whichTO] = false;
  }
}

double get_time_us() {
  DW1000.getSystemTimestamp(currentDWTime);
  currentTime = currentDWTime.getTimestamp();
  return currentTime * TIME_UNIT * 1e6;
}

uint64_t get_time_u64() {
  DW1000.getSystemTimestamp(currentDWTime);
  return currentDWTime.getTimestamp();
}


//Utility functions
void dateTime(uint16_t* date, uint16_t* time_) {
  DateTime now = rtc.now();
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day());

  // return time using FAT_TIME macro to format fields
  *time_ = FAT_TIME(now.hour(), now.minute(), now.second());
  printDateTime();
}

float getVoltage()
{

  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return measuredvbat;
}

void printDateTime()
{
  DateTime now = rtc.now();
  Serial.print(now.year());
  Serial.print("/");
  Serial.print(now.month());
  Serial.print("/");
  Serial.print(now.day());
  Serial.print(" ");
  Serial.print(now.hour());
  Serial.print(":");
  Serial.print(now.minute());
  Serial.print(":");
  Serial.println(now.second());
}

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__



int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}
