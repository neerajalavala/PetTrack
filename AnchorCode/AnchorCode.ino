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
#include "Neighbor.h"
#include "Adafruit_LSM9DS1.h"
#include <SdFat.h>
//#include "sdios.h"
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
// Yunzhi CIR Code
#define NCIR_FULL 1016
#define RX_TIME_FP_INDEX_OFFSET 5
int packet_count = 1;
int packet_type = 0;

int AlarmNoise = 0;// = rx_packet[LED_IDX] & 0x01;
int AlarmLed = 0;// = rx_packet[LED_IDX] & 0x02;

#define INIT_RTC_ALWAYS 0
#define SWITCH_INITIATOR 0
#define PRINT_CIR 0
#define USB_CONNECTION 0
//#define INITIATOR 1         /******CHANGE THIS******/
int INITIATOR = 0;
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
byte rx_resp_msg[MAX_RESP_LEN] = {RESP_MSG_TYPE, 0x02, 0, 0, 0, 0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
byte tx_final_msg[MAX_FINAL_LEN] = {FINAL_MSG_TYPE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int response_counter = 0;
int num_nodes = 11;          /******CHANGE THIS******/
int myDevID = 21;            /******CHANGE THIS******/

uint8_t num_initiators;
uint8_t initiator_list[10];
int IN_INITIATOR_LIST=1;
uint8_t pw_threshold = 95;

Ranging thisRange;

char rx_msg_char[200];
byte rx_packet[200];
uint8_t myAcc[1000];

typedef enum states {STATE_IDLE, STATE_POLL, STATE_RESP_EXPECTED, STATE_FINAL_SEND, STATE_TWR_DONE, STATE_RESP_SEND, STATE_FINAL_EXPECTED, STATE_OTHER_POLL_EXPECTED, STATE_RESP_PENDING, STATE_DIST_EST_EXPECTED, STATE_DIST_EST_SEND, STATE_TIGHT_LOOP,
                     STATE_RECEIVE, STATE_PRESYNC, STATE_SYNC, STATE_ANCHOR, STATE_TAG, STATE_FIRST_START, STATE_OBLIVION, STATE_ACK_EXPECTED
                    } STATES;
volatile uint8_t current_state = STATE_IDLE;
unsigned long silenced_at = 0;
#define SILENCE_PERIOD 120
long randNumber;
int currentSlots = 8;


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
int SDLOG = 0;


#define DIST_ALARM 1500
#define DIST_WARN 2500

//Time
RTC_PCF8523 rtc;

typedef struct DeviceRespTs {
  uint8_t deviceID;
  uint8_t priority; // Added for token passing
  uint64_t respRxTime;
};

// FOR PLUG N PLAY
bool first_time_flag = 1;
Neighbor neighbor;
uint8_t next_init;
int RECEIVE_TO_COUNT = 0;
int RX_TO_COUNT = 0;
int FIRST_INITIATOR = 0;
int ranging_count = 0;


#define MAX_DEVICES_TOGETHER 20
DeviceRespTs deviceRespTs[MAX_DEVICES_TOGETHER];
int currentDeviceIndex = 0;


void clear_deviceRespTs()
{
  for (int i = 0; i < neighbor.num_neighbors; i++)
  {
    deviceRespTs[i].deviceID = 0;
    deviceRespTs[i].priority = 0;
    deviceRespTs[i].respRxTime = 0;
  }
}

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
    myDevID = SDGetline();
    Serial.print("myDevID");
    Serial.println(myDevID);
  }
  
  if (myDevID == 0)
    {
      FIRST_INITIATOR = 1;
      INITIATOR = 1;
      Serial.print("Initiator ");
    } else {
      INITIATOR = 0;
      Serial.print("Responder ");
    }

  if (SDEnabled == 1) {
    sprintf(filename, "dist%03d.txt", filenum);
    if (!store_distance.open(filename, O_WRITE | O_CREAT)) {
      Serial.println("Could not create file");
      delay(10000);
    }

    Serial.println(filename);
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
//  DW1000.setDefaults();
  DW1000.setDefaults_longMsg();

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

  if (INITIATOR == 0) {
    disable_imu = 1;
    receiver(0);
  }
  //  #endif

  current_state = STATE_IDLE;

  for (int i = 0; i < MAX_TIMEOUTS; i++) {
    timeout_established[i] = false;
    timeout_triggered[i] = false;
    timeout_overflow[i] = false;
    timeout_time[i] = 0;
  }

  if (INITIATOR)
  {

    while (!lsm.begin()) {
      Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
      delay(1000);
#if   IGNORE_IMU==1
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
  }
  //#endif

  //startTimer(100);
}

void handleSent() {
  // status change on sent success
  sendComplete = true;
  //Serial.println("Send complete");
}

void handleReceived() {
  RX_TO_COUNT = 0;
  // status change on reception success

  DW1000.getData(rx_packet, DW1000.getDataLength());
  //  Serial.println("Received something...");
  received = true;
  byte2char(rx_packet, 24);
  Serial.println(rx_msg_char);
}

void handleError() {
  if (current_state == STATE_RECEIVE) {
    current_state = STATE_RECEIVE;
  } else if(current_state == STATE_ANCHOR) {
    current_state = STATE_ANCHOR;
  } else {
    current_state = STATE_IDLE;
  }
  RxTimeout = true;
  error = true;
  Serial.println("ERROR");
}

void handleRxTO() {
  RX_TO_COUNT++;
  Serial.println("RXTO");
  if (current_state == STATE_RESP_EXPECTED) {
    current_state = STATE_FINAL_SEND;
  } else if (current_state == STATE_RESP_EXPECTED) {
    current_state = STATE_RESP_EXPECTED;
  } else if (current_state == STATE_ANCHOR) {
    current_state = STATE_ANCHOR;
  } else {
    current_state = STATE_IDLE;
  }
  RxTimeout = true;
#if (DEBUG_PRINT==1)
//  Serial.println("Rx Timeout");
  // Serial.println("State: ");
  //Serial.println(current_state);
#endif

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

#define YIFENG_TEST 0
int test_flag = 0;

double start_time_us = 0, current_time_us = 0;

//Haige print RX and FP power only
void print_CIR_POWER()
{
  float RX_POWER = DW1000.getReceivePower();
  float FP_POWER = DW1000.getFirstPathPower();
  char buff[70];
  sprintf(buff, "RX_Power=%f dbm, FP_Power=%f dbm", RX_POWER, FP_POWER);
  Serial.println(buff);
}


//Yunzhi CIR Code
void print_CIR()
{
//    packet_count++;
    char buff[140];
    char long_buff[1400];
  
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
    
  uint8_t myAcc[4 * NCIR_FULL + 6];
  int starttap = 720;
  int endtap = 816;
  int16_t RealData = 0;
  int16_t ImaginaryData = 0;
  
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

void nchoosek(int n,int k, int* out)
{
  int arr[n];
  for(int i=0; i<n; i++)
  {
    arr[i] = i;  
  }
  
  for (int i= 0; i< n; i++) 
  {
    int pos = random(n);
    int t = arr[i];   
    arr[i] = arr[pos];
    arr[pos] = t;
  }

  for(int i=0;i<k;i++)
  {
    out[i] = arr[i];
  }
  return;  
}

void loop() {

  if (INITIATOR == 1)
  { digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  switch (current_state) {
//    case STATE_ANCHOR:{
//          
//          Serial.println("STATE_ANCHOR");
//          digitalWrite(LED_PIN,HIGH);
//          if(RxTimeout == true){
//            RxTimeout = false;
//            receiver(60);
//          }
//          if (received ) {
//            received = false;
//            byte2char(rx_packet, 24);
//            Serial.println(rx_msg_char);
//            
//            if(rx_packet[0]==ANCHOR_MSG_ALL_TYPE)
//            {
//              num_initiators = rx_packet[ANCHOR_MSG_NUM_INITIATOR_IDX];
//              for(int i=0;i<num_initiators;i++)
//              {
//                Serial.println(rx_packet[ANCHOR_MSG_INITIATOR_LIST_IDX+i]);
//                initiator_list[i] = rx_packet[ANCHOR_MSG_INITIATOR_LIST_IDX+i];                
//              }
//              
//              if(myDevID==rx_packet[ANCHOR_MSG_INITIATOR_LIST_IDX])
//              {
//                Serial.println("I'm Initiator");
//                INITIATOR = 1;
//              }else{
//                Serial.println("I'm not Initiator");
//
//                INITIATOR = 0;               
//              }         
//              receiver(60);
//              Serial.println("receiver");
//            }
//            else if(rx_packet[0]==BACK_MSG_ALL_TYPE)
//            {
//              Serial.println("Going Idle");
//              current_state = STATE_IDLE;
//              if(INITIATOR==0)
//              {
//                receiver(60);
//              }
//            }                     
//            
//          }
//      
//    break;
//      
//    }
    case STATE_IDLE: {
//        Serial.println("State: IDLE");


        if (RxTimeout == true) {
          //Serial.println("TO restart");
          RxTimeout = false;

          //Added
          if (FIRST_INITIATOR && RX_TO_COUNT > 50) //FIXME
          {
            INITIATOR = 1;
            RX_TO_COUNT = 0;
          }

          //Added
          if (myDevID==1 && RX_TO_COUNT > 300) //FIXME
          {
            INITIATOR = 1;
            RX_TO_COUNT = 0;
          }

          if (INITIATOR == 0)
          {
            //Serial.println("start receiver");
            receiver(60);
          }
          //        #endif
        }
        if (received ) {
          received = false;
          if (rx_packet[0] == POLL_MSG_TYPE)
          {
//            packet_count++;
            if (PRINT_CIR)
            {
              packet_count = rx_packet[SEQ_IDX] +  ((uint16_t)rx_packet[SEQ_IDX + 1] << 8);

              packet_type = 1;
              print_CIR();
            }

            thisRange.initialize();
            Serial.println("Setting state from IDLE to STATE_RESP_SEND");
            current_state = STATE_RESP_SEND;

#if (DEBUG_PRINT==1)
            //show_packet(rx_packet, DW1000.getDataLength());

            //Serial.println("******************");
//            Serial.println("Going to resp send");
#endif
          }else if(rx_packet[0]==PW_THRESH_MSG_ALL_TYPE)
          {
              pw_threshold = rx_packet[PW_THRESH_PW_IDX];
//              Serial.println(-(int)pw_threshold);
              receiver(60);
          }
          
          
          else if(rx_packet[0]==ANCHOR_MSG_ALL_TYPE)
          {
//            Serial.println("Going STATE_ANCHOR");
//            current_state = STATE_ANCHOR;
//            //RxTimeout == true;
//            Serial.println("receiver(60);");
//            receiver(60);

              num_initiators = rx_packet[ANCHOR_MSG_NUM_INITIATOR_IDX];
              if(num_initiators==0)
              {
                IN_INITIATOR_LIST = 1;
                if(myDevID==0)
                  FIRST_INITIATOR = 1;
              }else
              {
                for(int i=0;i<num_initiators;i++)
                {
//                  Serial.println(rx_packet[ANCHOR_MSG_INITIATOR_LIST_IDX+i]);
                  initiator_list[i] = rx_packet[ANCHOR_MSG_INITIATOR_LIST_IDX+i];                
                }
                
                if(initiator_list[0]==myDevID)
                  FIRST_INITIATOR=1;
                else
                  FIRST_INITIATOR=0;
                  
                if(is_in_initiator_list(myDevID))
                {
//                  Serial.println("I'm Initiator");
                  IN_INITIATOR_LIST = 1;
                }else{
//                  Serial.println("I'm not Initiator");
                  IN_INITIATOR_LIST = 0;
                }
              }
              
              receiver(60);
            }
          else {
            receiver(60);
          }

        }
        if (INITIATOR == 1)
        {
          //Randomly begin the POLL process.
          delay(8);      /******FOR TESTING ONLY******/
          waiting = 0;
          received = false;
          sendComplete = false;
          //Switch to POLL state
          current_state = STATE_POLL;
          unlock_waiting = 0;
          response_counter = 0;
        }
        //      #endif
        break;
      }
    case STATE_POLL: {
        //Send POLL here
        //Serial.println("State: POLL");
        clear_deviceRespTs();
        seq++;
        currentDeviceIndex = 0;

        tx_poll_msg[SRC_IDX] = myDevID;
        tx_poll_msg[DST_IDX] = BROADCAST_ID;
        tx_poll_msg[SEQ_IDX] = seq & 0xFF;
        tx_poll_msg[SEQ_IDX + 1] = seq >> 8;

//          tx_poll_msg[POLL_MSG_POLL_NUM_RESP_IDX] = neighbor.num_neighbors;          
//          for (int i = 0; i < neighbor.num_neighbors; i++)
//          {
//            tx_poll_msg[POLL_MSG_POLL_RESP_ORDER_IDX + i] = neighbor.neighbor_id[i];
//          }     
          uint8_t scheduled_neighbor_count = 0;    
          for (int i = 0; i < neighbor.num_neighbors; i++)
          {
            if(neighbor.neighbor_FP_PW[i]>-105)
            {
            tx_poll_msg[POLL_MSG_POLL_RESP_ORDER_IDX + scheduled_neighbor_count] = neighbor.neighbor_id[i];
            scheduled_neighbor_count++;
            }
          }   
          tx_poll_msg[POLL_MSG_POLL_NUM_RESP_IDX] = scheduled_neighbor_count;   

        currentTime = get_time_u64();
//        Serial.println("POLL MESSAGE:");
//        show_packet_8B(tx_poll_msg);
        FIXED_DELAY = 6;
        generic_send(tx_poll_msg, sizeof(tx_poll_msg), POLL_MSG_POLL_TX_TS_IDX, SEND_DELAY_FIXED);

        current_state = STATE_RESP_EXPECTED;

        while (!sendComplete) {
        };

        current_time_us = get_time_us();
        sendComplete = false;
        receiver(TYPICAL_RX_TIMEOUT);
        DW1000.getSystemTimestamp(currentDWTime);

        init_time = currentDWTime.getTimestamp();
        break;
      }
    /* Dont delete, TBC*/
    case STATE_RESP_SEND: {

        //uint8_t current_poll_rand = rx_packet[POLL_MSG_RAND_BOUND_IDX];
        Serial.println("State: RESP SEND");

        seq = rx_packet[SEQ_IDX] +  ((uint16_t)rx_packet[SEQ_IDX + 1] << 8);

        // Add the current initiator as neighbor
        neighbor.add_neighbor(rx_packet[SRC_IDX]);

        uint64_t PollTxTime_64 = 0L;
        any_msg_get_ts(&rx_packet[POLL_MSG_POLL_TX_TS_IDX], &PollTxTime_64);
        thisRange.PollTxTime = DW1000Time((int64_t)PollTxTime_64);


        float RX_POWER = DW1000.getReceivePower();
        float FP_POWER = DW1000.getFirstPathPower();
        neighbor.add_neighbor_PW(rx_packet[SRC_IDX],FP_POWER);

        int in_schedule = 0;
        uint8_t num_responder = rx_packet[POLL_MSG_POLL_NUM_RESP_IDX];
        //num_nodes = rx_packet[POLL_MSG_POLL_NUM_RESP_IDX];
        for (int i = 0; i < num_responder ; i++)
        {
          if (rx_packet[POLL_MSG_POLL_RESP_ORDER_IDX + i] == myDevID)
          {
            Serial.print(i);
            Serial.println("th to send");
            FIXED_DELAY = i * 6 + 6;
            in_schedule = 1;
            first_time_flag = 0; //flipped the flag so that first_time code is not run again
            break;
          }
        }
        //TODO: how to deal with lost packets where the device ID of the current device is not found in the poll message? where do you reset the first_time_flag?
        Serial.print("in_schedule: ");
        Serial.println(in_schedule);
        if (in_schedule == 0)
        {
          if (first_time_flag)
          {
            Serial.println("First time!");
            //changed from num_responder to myDevID - predefining the order
            FIXED_DELAY = myDevID * 6 + 6;
          }
          else {
            current_state = STATE_IDLE;
//            Serial.println("Not scheduled");
            receiver(60);
            break;
          }
        }

        //Serial.println("Now the time thing");
        DW1000Time rxTS;
        DW1000.getReceiveTimestamp(rxTS);
        thisRange.PollRxTime = rxTS;
//        Serial.println("About to start generic send: ");

        rx_resp_msg[DST_IDX] = rx_packet[SRC_IDX];
        rx_resp_msg[SRC_IDX] = myDevID;
        rx_resp_msg[SEQ_IDX] = seq & 0xFF;
        rx_resp_msg[SEQ_IDX + 1] = seq >> 8;
        
        uint16_t RX_POWER_fp = float_to_fixed_point(RX_POWER);
        uint16_t FP_POWER_fp = float_to_fixed_point(FP_POWER);
        
        rx_resp_msg[RESP_MSG_POLL_PW_IDX] = RX_POWER_fp>>8 & 0xFF;
        rx_resp_msg[RESP_MSG_POLL_PW_IDX+1] = RX_POWER_fp & 0xFF;
        rx_resp_msg[RESP_MSG_POLL_PW_IDX+2] = FP_POWER_fp>>8 & 0xFF;
        rx_resp_msg[RESP_MSG_POLL_PW_IDX+3] = FP_POWER_fp & 0xFF;

        if(IN_INITIATOR_LIST)
        {
        rx_resp_msg[RESP_MSG_PRIORITY_IDX] = neighbor.return_priority(); //Embed my priority for next INITIATOR
        if(FP_POWER<-pw_threshold)
          rx_resp_msg[RESP_MSG_PRIORITY_IDX] = 0;

        }else{
        rx_resp_msg[RESP_MSG_PRIORITY_IDX] = 0;
        }
        uint64_t Db_u64 = thisRange.Db.getTimestamp();
        uint64_t Rb_u64 = thisRange.Rb.getTimestamp();
        any_msg_set_ts(&rx_resp_msg[RESP_MSG_PREV_DB_IDX], Db_u64);
        any_msg_set_ts(&rx_resp_msg[RESP_MSG_PREV_RB_IDX], Rb_u64);
//        show_packet_8B(rx_resp_msg);
        Serial.println("Sending Response ...");
        generic_send(rx_resp_msg, sizeof(rx_resp_msg), POLL_MSG_POLL_TX_TS_IDX, SEND_DELAY_FIXED);

        while (!sendComplete);
        sendComplete = false;
        DW1000Time txTS;
        DW1000.getTransmitTimestamp(txTS);
        thisRange.RespTxTime = txTS;

        Serial.println("Response sent");
        receiver(0);
        current_state = STATE_FINAL_EXPECTED;
        break;
      }
    //*/
    case STATE_RESP_EXPECTED: {

        /*if (sendComplete) {
          sendComplete = false;
          receiver();
          }*/
        if (received ) {
          received = false;
          if(rx_packet[0]==ANCHOR_MSG_ALL_TYPE)
          {
              num_initiators = rx_packet[ANCHOR_MSG_NUM_INITIATOR_IDX];
              if(num_initiators==0)
              {
                IN_INITIATOR_LIST = 1;
                if(myDevID==0)
                  FIRST_INITIATOR = 1;
              }else
              {
                for(int i=0;i<num_initiators;i++)
                {
//                  Serial.println(rx_packet[ANCHOR_MSG_INITIATOR_LIST_IDX+i]);
                  initiator_list[i] = rx_packet[ANCHOR_MSG_INITIATOR_LIST_IDX+i];                
                }
                
                if(initiator_list[0]==myDevID)
                  FIRST_INITIATOR=1;
                else
                  FIRST_INITIATOR=0;
                  
                if(is_in_initiator_list(myDevID))
                {
//                  Serial.println("I'm Initiator");
                  IN_INITIATOR_LIST = 1;
                }else{
//                  Serial.println("I'm not Initiator");
                  IN_INITIATOR_LIST = 0;
                }
              }
              received = false;
              receiver(60);
              break;
          }

          if(rx_packet[0]==PW_THRESH_MSG_ALL_TYPE)
          {
              pw_threshold = rx_packet[PW_THRESH_PW_IDX];
//              Serial.println(-(int)pw_threshold);
              received = false;
              receiver(60);
              break;
          }

          if (rx_packet[DST_IDX] == myDevID && rx_packet[0] == RESP_MSG_TYPE) {
            
            //Serial.println("Recieved response!");
            recvd_resp_seq = rx_packet[SEQ_IDX] +  ((uint16_t)rx_packet[SEQ_IDX + 1] << 8);
                      
            //Add the responding nodes as neighbors
            currentDeviceIndex = rx_packet[SRC_IDX];
            neighbor.add_neighbor(currentDeviceIndex);
//            float FP_POWER = DW1000.getFirstPathPower();
//            neighbor.add_neighbor_PW(currentDeviceIndex,FP_POWER);
            deviceRespTs[response_counter].priority = rx_packet[RESP_MSG_PRIORITY_IDX];

            deviceRespTs[response_counter].deviceID = rx_packet[SRC_IDX];
            DW1000Time rxTS;
            DW1000.getReceiveTimestamp(rxTS);
            deviceRespTs[response_counter].respRxTime = rxTS.getTimestamp();
            deviceRespTs[response_counter].priority = rx_packet[RESP_MSG_PRIORITY_IDX];

            response_counter++;
            //num_nodes = neighbor.num_neighbors+1;
            
            if (response_counter < num_nodes)
            {
              current_state = STATE_RESP_EXPECTED;
              //Serial.println("back to STATE_RESP_EXPECTED");
              receiver(25);
            } else {
              current_state = STATE_FINAL_SEND;
              response_counter = 0;
            }

            if (PRINT_CIR)
            {
              packet_count = recvd_resp_seq;
              packet_type = 2;
              print_CIR();
            }  
//            char buff[80];
//            sprintf(buff, "Response no. %u from ID %u with priority %u", response_counter, currentDeviceIndex, rx_packet[RESP_MSG_PRIORITY_IDX]);
//            Serial.println(buff);

          } else {
            received = false;
            receiver(TYPICAL_RX_TIMEOUT);
          }
        } else {
          /*waiting++;
            if (waiting>200000)
            {
            current_state=STATE_IDLE;
            }*/
        }
        if (unlock_waiting == 1)
        {
          waiting++;
          if (waiting > 50000)
          {
            waiting = 0;
//            Serial.println("Waiting TO");
            current_state = STATE_FINAL_SEND;
          }
          if (timeout_triggered[1] == true) {
//            Serial.println("RX TO");
            current_state = STATE_FINAL_SEND;
          }
        }
        break;
      }
    case STATE_FINAL_SEND: {
        // Serial.println("State: FINAL SEND");
        neighbor.push_new((uint8_t)0);
        tx_final_msg[SRC_IDX] = myDevID;
        tx_final_msg[DST_IDX] = BROADCAST_ID;
        tx_final_msg[SEQ_IDX] = recvd_resp_seq & 0xFF;
        tx_final_msg[SEQ_IDX + 1] = recvd_resp_seq >> 8;
        ranging_count++;

        if (SWITCH_INITIATOR)
        {
          if (ranging_count < 3)
          {
            next_init = myDevID;
          }
          else {
            next_init = find_next_init();
            ranging_count = 0;
          }
        } else {
          next_init = myDevID;
        }
        tx_final_msg[FINAL_MSG_NEXT_INIT_IDX] = next_init;
//        Serial.println(next_init);

        //tx_final_msg[FINAL_MSG_NUM_NODES_IDX] = currentDeviceIndex;
        int imu_buffer_counter = 0;
        currentTime = get_time_u64();
        //num_nodes = response_counter;
        for (int i = 0; i < num_nodes; i++)
        {
          tx_final_msg[FINAL_MSG_RESP_IDX + (i * FINAL_MSG_ONE_RESP_ENTRY)] = deviceRespTs[i].deviceID;
          any_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX  + (i * FINAL_MSG_ONE_RESP_ENTRY)], deviceRespTs[i].respRxTime);
        }
//        show_packet_8B(tx_final_msg);
        FIXED_DELAY = 6;
        generic_send(tx_final_msg, MAX_FINAL_LEN, FINAL_MSG_FINAL_TX_TS_IDX, SEND_DELAY_FIXED);
        while (!sendComplete);
        sendComplete = false;

        DW1000.getSystemTimestamp(currentDWTime);
        final_sent_time = currentDWTime.getTimestamp();

        if (next_init == myDevID)
        {
          current_state = STATE_IDLE;
        } else
        {
          current_state = STATE_RECEIVE;
          RECEIVE_TO_COUNT = 0;
          INITIATOR = 0;
          receiver(60);
        }

        break;
      }
    case STATE_ACK_EXPECTED: {
        //Serial.println("State: ACK EXPECTED");
        if (received) {
          received = false;
          //Serial.println("ACK: receive something!");
          if (rx_packet[0] == TWR_DONE_TYPE) {
            //Serial.println("ACK: receive a TWR DONE packet!");
            recvd_resp_seq = rx_packet[SEQ_IDX] +  ((uint16_t)rx_packet[SEQ_IDX + 1] << 8);
            if (recvd_resp_seq == seq) {
              //Serial.println("Recieved ACK!");
              current_state = STATE_POLL;
              break;
            }
          }
        }

        DW1000.getSystemTimestamp(currentDWTime);
        currentTime = currentDWTime.getTimestamp();
        elapsed_time = (currentTime - final_sent_time) * TIME_UNIT * 1000;
        if (elapsed_time >= TYPICAL_RX_TIMEOUT) {
          current_state = STATE_POLL;
          break;
        }
        start_time_us = get_time_us();
        break;
      }
    /* Dont delete, TBC*/
    case STATE_FINAL_EXPECTED: {
        //Serial.println("State: FINAL EXPECTED");

        if (received) {
          received = false;

          if (rx_packet[0] == FINAL_MSG_TYPE) {
            recvd_resp_seq = rx_packet[SEQ_IDX] +  ((uint16_t)rx_packet[SEQ_IDX + 1] << 8);
            if (PRINT_CIR)
            {
              packet_count = recvd_resp_seq;
              packet_type = 3;
              print_CIR();
            }
            
            DW1000Time rxTS;
            DW1000.getReceiveTimestamp(rxTS);
            thisRange.FinalRxTime = rxTS;
//            Serial.println((unsigned int)rxTS.getTimestamp(), HEX);
            neighbor.push_new((uint8_t)1);
            for (int i = 0; i < num_nodes; i++) {
              if (rx_packet[FINAL_MSG_RESP_IDX + (i * FINAL_MSG_ONE_RESP_ENTRY)] == myDevID) {
                Serial.print("Place in FINAL");
                Serial.println(i);
                uint64_t RespRxTime_64 = 0L;
                any_msg_get_ts(&rx_packet[FINAL_MSG_RESP_RX_TS_IDX + (i * FINAL_MSG_ONE_RESP_ENTRY)], &RespRxTime_64);
                thisRange.RespRxTime = DW1000Time((int64_t)RespRxTime_64);
                break;
              }
            }
            uint64_t FinalTxTime_64 = 0L;
            any_msg_get_ts(&rx_packet[FINAL_MSG_FINAL_TX_TS_IDX], &FinalTxTime_64);
            seq = rx_packet[SEQ_IDX] +  ((uint16_t)rx_packet[SEQ_IDX + 1] << 8);
            //seq = rx_packet[SEQ_IDX];
            uint8_t next_init = rx_packet[FINAL_MSG_NEXT_INIT_IDX];
//            Serial.println(next_init);

            if (next_init == myDevID)
            {
              INITIATOR = 1;
              ranging_count = 0;
            } else {
              INITIATOR = 0;
              receiver(); //Enable the receiver quickly to allow the next POLL to work
            }

            thisRange.FinalTxTime = DW1000Time((int64_t)FinalTxTime_64);

            int dist = thisRange.calculateRange();
            //thisRange.printAll();
            //Serial.print(seq);
            char buf[60];
            sprintf(buf, "Packet%d  Dist:%d mm", recvd_resp_seq, dist);
            Serial.println(buf);

/* temporarily commented out - don't delete
            if (dist > 0 && dist < 40000) //only keep value between 0 and 40m
            {
              if (SDEnabled)
              {
                char buff[60];
                sprintf(buff, "ToA %u - %u: %d mm", myDevID, rx_packet[SRC_IDX], dist);
                store_distance.println(buff);
                entries_in_file++;
                if (entries_in_file > 100)
                {
                  entries_in_file = 0;
//                  Serial.println("Changed files");
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
**/
            current_state = STATE_IDLE;

          } else if (rx_packet[0] == POLL_MSG_TYPE) {
            current_state = STATE_IDLE;
            receiver(0);
          } else if(rx_packet[0]==PW_THRESH_MSG_ALL_TYPE)
          {
              pw_threshold = rx_packet[PW_THRESH_PW_IDX];
              //Serial.println(-(int)pw_threshold);
              current_state = STATE_IDLE;
              receiver(0);
          }
          else if(rx_packet[0]==ANCHOR_MSG_ALL_TYPE)
          {
              num_initiators = rx_packet[ANCHOR_MSG_NUM_INITIATOR_IDX];
              if (num_initiators==0)
              {
                IN_INITIATOR_LIST = 1;
                if(myDevID==0)
                  FIRST_INITIATOR = 1;
              }else
              {
                for (int i=0;i<num_initiators;i++)
                {
//                  Serial.println(rx_packet[ANCHOR_MSG_INITIATOR_LIST_IDX+i]);
                  initiator_list[i] = rx_packet[ANCHOR_MSG_INITIATOR_LIST_IDX+i];                
                }
                if (initiator_list[0]==myDevID)
                  FIRST_INITIATOR=1;
                else
                  FIRST_INITIATOR=0;
  
                  
                if (is_in_initiator_list(myDevID))
                {
//                  Serial.println("I'm Initiator");
                  IN_INITIATOR_LIST = 1;
                }else{
//                  Serial.println("I'm not Initiator");
                  IN_INITIATOR_LIST = 0;
                }
              }
              current_state = STATE_IDLE;
              receiver(0);
          }
          else {
            received = false;
            receiver(0);
          }
        }

        break;
      }
    case STATE_RECEIVE: //This is the state where after INITIATOR is passed to others, the previous INITIATOR listens to see whether Token is successfully passed
      {
//        Serial.println("STATE_RECEIVE");
        if (RxTimeout == true)
        {
          RxTimeout = false;
          RECEIVE_TO_COUNT++;
          if (RECEIVE_TO_COUNT > 20) //CHANGE ME
          {
//            Serial.println("RECEIVE_TO_COUNT > 20");

            INITIATOR = 1;
            current_state = STATE_IDLE;
          } else {
            current_state = STATE_RECEIVE;
            receiver(60);
          }
        }

        if (received ) {
          received = false;

          if (rx_packet[0] == POLL_MSG_TYPE && rx_packet[SRC_IDX] == next_init)
          {

            thisRange.initialize();
            current_state = STATE_RESP_SEND;
#if (DEBUG_PRINT==1)
            //show_packet(rx_packet, DW1000.getDataLength());
            //Serial.println("******************");
//            Serial.println("Going to resp send");
#endif
          } else {
            receiver(60);
            RECEIVE_TO_COUNT++;
          }

        }
        break;
      }


    //*/
    case STATE_OBLIVION: {
        //Do nothing!
      }


  }

}

bool is_in_initiator_list(uint8_t id)
{
    for(int i=0;i<num_initiators;i++)
    {
       if(initiator_list[i]==id)
       {
          return true;  
        }
    }
    return false;
}


/*Find the next initiator*/
uint8_t find_next_init()
{
  uint8_t max_idx = myDevID;
  uint8_t max_val = neighbor.return_priority();

  for (int i = 0; i < neighbor.num_neighbors; i++)
  {
    if (deviceRespTs[i].priority > max_val)
    {
      max_idx = (uint8_t)deviceRespTs[i].deviceID;
      max_val = deviceRespTs[i].priority;
    }
    else if (deviceRespTs[i].priority == max_val) // If equal priority, then randomly pick one
    {
      int rand_num = random(0, 2);
      if (rand_num == 0)
      {
        max_idx = (uint8_t)deviceRespTs[i].deviceID;
        max_val = deviceRespTs[i].priority;
      }
    }
  }
  if(max_val==0)
    max_idx =  myDevID;
  return max_idx;
}


/*Find the next initiator*/
uint8_t find_next_init_initiator_list()
{
  uint8_t max_idx = myDevID;
  uint8_t max_val = neighbor.return_priority();

  for (int i = 0; i < neighbor.num_neighbors; i++)
  {
    if(is_in_initiator_list(deviceRespTs[i].deviceID))
    {
      if (deviceRespTs[i].priority > max_val)
      {
        max_idx = (uint8_t)deviceRespTs[i].deviceID;
        max_val = deviceRespTs[i].priority;
      }
      else if (deviceRespTs[i].priority == max_val) // If equal priority, then randomly pick one
      {
        int rand_num = random(0, 2);
        if (rand_num == 0)
        {
          max_idx = (uint8_t)deviceRespTs[i].deviceID;
          max_val = deviceRespTs[i].priority;
        }
      }
    }
  }
  return max_idx;
}



void show_packet(byte packet[], int num) {
#if (DEBUG_PRINT==1)
  for (int i = 0; i < num; i++) {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");
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
  char msg[200];
  int i = 0;
  for (; i < len;) {
    msg[3 * i] = hex[packet[i] >> 4 & 0x0F];
    msg[3 * i + 1] = hex[packet[i] & 0x0F];
    msg[3 * i + 2] = ' ';
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

int SDGetline() {

  File myfile = sd.open("ID.txt");
  int ret = -1;
  if (myfile) {
    ret = myfile.read() - '0';
    myfile.close();
  } else {
    Serial.println("error opening ID.txt");
  }

  return ret;
}
