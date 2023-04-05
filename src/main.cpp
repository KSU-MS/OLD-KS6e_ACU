#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Metro.h>
#include <MDB_labels.h>

#define DEBUG true

// BIG DEFINES FOR MODULES & SWITCH CASE (absolute cancer code)
#define NUMBER_OF_CELLS 72
#define NUMBER_OF_MODULES 6
#define CELLS_PER_MODULE 12
#define CELLS_1A CELLS_PER_MODULE*0/2
#define CELLS_1B CELLS_PER_MODULE*1/2
#define CELLS_2A CELLS_PER_MODULE*2/2
#define CELLS_2B CELLS_PER_MODULE*3/2
#define CELLS_3A CELLS_PER_MODULE*4/2
#define CELLS_3B CELLS_PER_MODULE*5/2
#define CELLS_4A CELLS_PER_MODULE*6/2
#define CELLS_4B CELLS_PER_MODULE*7/2
#define CELLS_5A CELLS_PER_MODULE*8/2
#define CELLS_5B CELLS_PER_MODULE*9/2
#define CELLS_6A CELLS_PER_MODULE*10/2
#define CELLS_6B CELLS_PER_MODULE*11/2

int8_t batteryTemps[NUMBER_OF_CELLS];

// Canbus stuff
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CAN_1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> ACC_1;
#define NUM_TX_MAILBOXES 2
#define NUM_RX_MAILBOXES 6

// Can IDs
#define BMS_ID 0x7E3
#define ThermistorToBMS_ID 0x9839F380
#define ThermistorAddressClaim_ID 0x98EEFF80 // probably unnecessary
#define BMS_Response_ID 0x7EB

// Can bytes
CAN_message_t rxMsg;
int moduleNo = 0;                                                         // byte0
int enabledTherm;                                                         // byte4
byte getLowestTemp[] = {0x03, 0x22, 0xF0, 0x28, 0x55, 0x55, 0x55, 0x55};  // lowest temp request
byte getHighestTemp[] = {0x03, 0x22, 0xF0, 0x29, 0x55, 0x55, 0x55, 0x55}; // lowest temp request
Metro sendTempRate = Metro(100);
Metro getTempRate = Metro(500);
Metro doThingsRate = Metro(100);
Metro heartBeat = Metro(100);

// Globals
int globalHighTherm = 25, globalLowTherm = 25;

// Function defs
int ReadACC_1(CAN_message_t &msg);
void updateAccumulatorCAN();
void getTempData();
void sendTempData();

// Setup -----------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  delay(400);

  // This is the main battery temp array
  Serial.println("Battery temp array: ");
  for (int i = 0; i < NUMBER_OF_CELLS; i++)
  {
    Serial.print("Cell number: ");
    Serial.print(i);
    Serial.print(" Value: ");
    batteryTemps[i] = 0; // init default temps as a safe value
    Serial.println(batteryTemps[i]);
  }

  // CAN 1 setup
  CAN_1.begin();
  CAN_1.setBaudRate(500000);
  CAN_1.setMaxMB(16);
  CAN_1.enableFIFO();
  CAN_1.enableFIFOInterrupt();
  CAN_1.mailboxStatus();
  // Serial.println("Send something on serial to continue...");
  // while(!Serial.available());{ }
  
  // Isolated ACC CAN setup
  ACC_1.begin();
  ACC_1.setBaudRate(500000);
  ACC_1.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
  for (int i = 0; i < NUM_RX_MAILBOXES; i++)
  {
    ACC_1.setMB((FLEXCAN_MAILBOX)i, RX, STD);   
  }
  for (int i = NUM_RX_MAILBOXES; i < (NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++)
  {
    ACC_1.setMB((FLEXCAN_MAILBOX)i, TX, STD);
  }
}
// Setup -----------------------------------------------------------------------

// Main loop -----------------------------------------------------------------------
void loop()
{
  if(heartBeat.check()){
    digitalToggle(LED_BUILTIN);
  }
  
  CAN_1.events();
  
  if (doThingsRate.check()) {
    updateAccumulatorCAN();
  }
  if (sendTempRate.check() == 1){
    sendTempData();
  }
}
// Main loop -----------------------------------------------------------------------

// Updates battery temp array with the values from the isolated ACC canbus via a switch case
void updateAccumulatorCAN()
{  
  if (ReadACC_1(rxMsg))
  {
    #ifdef DEBUG
    Serial.print("MB "); Serial.print(rxMsg.mb);
    Serial.print("  OVERRUN: "); Serial.print(rxMsg.flags.overrun);
    Serial.print("  LEN: "); Serial.print(rxMsg.len);
    Serial.print(" EXT: "); Serial.print(rxMsg.flags.extended);
    Serial.print(" TS: "); Serial.print(rxMsg.timestamp);
    Serial.print(" ID: "); Serial.print(rxMsg.id, HEX);
    Serial.print(" Buffer: ");
    for ( uint8_t i = 0; i < rxMsg.len; i++ ) {
      Serial.print(rxMsg.buf[i], HEX); Serial.print(" ");
    } 
    Serial.println();
    #endif

    switch (rxMsg.id)  // This is cancer probably and could better be implemented with a loop I imagine
    {
      case (MODULE_1_A): 
      {
        Serial.println("Module 1 A");
        memcpy(batteryTemps+CELLS_1A,rxMsg.buf,6);
        break;
      }
      case (MODULE_1_B): 
      {
        Serial.println("Module 1 B");
        memcpy(batteryTemps+CELLS_1B,rxMsg.buf,6);
        break;
      }
      case (MODULE_2_A): 
      {
        Serial.println("Module 2 A");
        memcpy(batteryTemps+CELLS_2A,rxMsg.buf,6);
        break;
      }
      case (MODULE_2_B): 
      {
        Serial.println("Module 2 B");
        memcpy(batteryTemps+CELLS_2B,rxMsg.buf,6);
        break;
      }
      case (MODULE_3_A): 
      {
        Serial.println("Module 3 A");
        memcpy(batteryTemps+CELLS_3A,rxMsg.buf,6);
        break;
      }
      case (MODULE_3_B): 
      {
        Serial.println("Module 3 B");
        memcpy(batteryTemps+CELLS_3B,rxMsg.buf,6);
        break;
      }
      case (MODULE_4_A): 
      {
        Serial.println("Module 4 A");
        memcpy(batteryTemps+CELLS_4A,rxMsg.buf,6);
        break;
      }
      case (MODULE_4_B): 
      {
        Serial.println("Module 4 B");
        memcpy(batteryTemps+CELLS_4B,rxMsg.buf,6);
        break;
      }
      case (MODULE_5_A): 
      {
        Serial.println("Module 5 A");
        memcpy(batteryTemps+CELLS_5A,rxMsg.buf,6);
        break;
      }
      case (MODULE_5_B): 
      {
        Serial.println("Module 5 B");
        memcpy(batteryTemps+CELLS_5B,rxMsg.buf,6);
        break;
      }
      case (MODULE_6_A): 
      {
        Serial.println("Module 6 A");
        memcpy(batteryTemps+CELLS_6A,rxMsg.buf,6);
        break;
      }
      case (MODULE_6_B): 
      {
        Serial.println("Module 6 B");
        memcpy(batteryTemps+CELLS_6B,rxMsg.buf,6);
        break;
      }
      default:
      {
        break;
      }
    }
  }
}

// Read the ACC can
int ReadACC_1(CAN_message_t &msg)
{    
  int rxMSG = ACC_1.read(msg);
  return rxMSG;
}

// Sending highest/lowest temperature to the BMS
void sendTempData()
{
  CAN_message_t sendTempMsg;
  sendTempMsg.flags.extended = 1;      // extended id
  sendTempMsg.len = 8;                 // per protocol
  sendTempMsg.id = ThermistorToBMS_ID; // Temp broadcast ID
  enabledTherm = NUMBER_OF_CELLS - 1;  // number of cells 0 based
  int lowTherm = batteryTemps[0];
  int lowestThermId = 0;
  int highTherm = batteryTemps[0];
  int highestThermId = 0;
  for (int i = 0; i < NUMBER_OF_CELLS; i++)
  { // get lowest and highest
    #ifdef DEBUG
    // Serial.print("Cell number: ");
    // Serial.print(i);
    // Serial.print(" Value: ");
    // Serial.println(batteryTemps[i]);
    #endif
    if (batteryTemps[i] < lowTherm)
    {
      lowTherm = batteryTemps[i];
      lowestThermId = i;
    }
    if (batteryTemps[i] > highTherm)
    {
      highTherm = batteryTemps[i];
      highestThermId = i;
    }
    #ifdef DEBUG
    // Serial.printf("Iter: %d Highest: %d Lowest: %d\n",i,highTherm,lowTherm);
    #endif
  }

  int avgTherm = (lowTherm + highTherm) / 2;                                                                          // yep
  int checksum = moduleNo + lowTherm + highTherm + avgTherm + enabledTherm + highestThermId + lowestThermId + 57 + 8; // 0x39 and 0x08 added to checksum per orion protocol
  byte tempdata[] = {moduleNo, lowTherm, highTherm, avgTherm, enabledTherm, highestThermId, lowestThermId, checksum};
  #ifdef DEBUG
  // Serial.println(tempdata[2]);
  #endif
  memcpy(sendTempMsg.buf, tempdata, sizeof(sendTempMsg.buf));
  CAN_1.write(sendTempMsg);
  // GLobal ints for tracking
  globalHighTherm = highTherm;
  globalLowTherm = lowTherm;
}

// getting one of the max temps from BMS (high or low not sure lol)
void getTempData()
{
  CAN_message_t getTempMsg;
  getTempMsg.flags.extended = 1;
  getTempMsg.len = 8;
  getTempMsg.id = BMS_ID; // OUR BMS
  memcpy(getTempMsg.buf, getLowestTemp, sizeof(getTempMsg.buf));
  CAN_1.write(getTempMsg);
  Serial.println("Requesting Lowest Temp Data...");
  memcpy(getTempMsg.buf, getHighestTemp, sizeof(getTempMsg.buf));
  CAN_1.write(getTempMsg);
  Serial.println("Requesting Highest Temp Data...");
}