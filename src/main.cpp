#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Metro.h>
#include <WireIMXRT.h>
#include <FreqMeasureMulti.h>
#include <MDB_labels.h>

enum ACUSTATE
{
  BOOTUP,
  RUNNING,
  FAULT
};
int acuState = 0;
#define runningFoReal
#define DEBUG true

// BIG DEFINES FOR MODULES (absolute cancer code)
#define NUMBER_OF_CELLS 72
#define NUMBER_OF_MODULES 6
#define CELLS_PER_MODULE (NUMBER_OF_CELLS/NUMBER_OF_MODULES)

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

#ifdef runningFoReal
#define NUMBER_OF_LTCs 5

uint8_t gettingTempState = 0; // 0=set 1=wait 2=get
// Init ADCs
// Ltc2499 theThings[6];
float batteryTempvoltages[NUMBER_OF_CELLS];
// uint8_t ltcAddressList[] = {ADDR_Z00, ADDR_Z0Z, ADDR_0Z0, // one two three
//                             ADDR_ZZ0, ADDR_0ZZ};          // first 6 configurable addresses in the mf datasheet
// byte ADCChannels[] = {CHAN_SINGLE_0P, CHAN_SINGLE_1P, CHAN_SINGLE_2P, CHAN_SINGLE_3P,
//                       CHAN_SINGLE_4P, CHAN_SINGLE_5P, CHAN_SINGLE_6P, CHAN_SINGLE_7P,
//                       CHAN_SINGLE_8P, CHAN_SINGLE_9P, CHAN_SINGLE_10P, CHAN_SINGLE_11P};
elapsedMillis conversionTime; // wait 80ms for conversion to be ready
#endif

// ACC Canbus stuff
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CAN_1;
FlexCAN_T4 <CAN2, RX_SIZE_256, TX_SIZE_16> ACU_;
#define NUM_TX_MAILBOXES 2
#define NUM_RX_MAILBOXES 6
CAN_message_t rxMsg;
uint8_t temp1[8];
uint8_t temp2[8];



int currentChannel = 0;
int globalHighTherm = 25, globalLowTherm = 25;
int HBstate = 0;

// Can IDs
#define BMS_ID 0x7E3
#define ThermistorToBMS_ID 0x9839F380
#define ThermistorAddressClaim_ID 0x98EEFF80 // probably unnecessary
#define BMS_Response_ID 0x7EB

// Can bytes
int moduleNo = 0;                                                         // byte0
int enabledTherm;                                                         // byte4
byte getLowestTemp[] = {0x03, 0x22, 0xF0, 0x28, 0x55, 0x55, 0x55, 0x55};  // lowest temp request
byte getHighestTemp[] = {0x03, 0x22, 0xF0, 0x29, 0x55, 0x55, 0x55, 0x55}; // lowest temp request
Metro sendTempRate = Metro(100);
Metro getTempRate = Metro(500);
Metro doThingsRate = Metro(100);
Metro heartbeat = Metro(100);



// printing received to serial
void canSniff(const CAN_message_t &msg);
void getTempData();
void sendTempData();
void ACUStateMachine();
int setChannels(int channelNo);
void setChannelsSwitchCase(int channelNo);
void getTemps(int channelNo);
void DebuggingPrintout();

int ReadBatteryTemps(CAN_message_t &msg);
void updateAccumulatorCAN();
void init_ACU_CAN();

void setup()
{
  Wire.setClock(100000);
  Wire.begin();
  Serial.begin(115200);
  delay(400);

  // This is temp array
  Serial.println("Battery temp array: ");
  for (int i = 0; i < NUMBER_OF_CELLS; i++)
  {
    Serial.print("Cell number: ");
    Serial.print(i);
    Serial.print(" Value: ");
    batteryTemps[i] = 0; // init default temps as a safe value
    Serial.println(batteryTemps[i]);
  }

  CAN_1.begin();
  CAN_1.setBaudRate(500000);
  CAN_1.setMaxMB(16);
  CAN_1.enableFIFO();
  CAN_1.enableFIFOInterrupt();
#ifdef DEBUG
  CAN_1.onReceive(canSniff);
#endif
  CAN_1.mailboxStatus();
  // Serial.println("Send something on serial to continue...");
  // while(!Serial.available());{ }
  
  init_ACU_CAN();
}

// Main loop -----------------------------------------------------------------------
void loop()
{
  if(heartbeat.check())
  {
    HBstate=!HBstate;
    digitalWrite(LED_BUILTIN, HBstate);
  }
  
  CAN_1.events();
  
  if (doThingsRate.check())
  {
    updateAccumulatorCAN();
  }
  if (sendTempRate.check() == 1)
  {
    sendTempData();
    #if DEBUG
    // DebuggingPrintout();
    #endif
  }
}
// Main loop -----------------------------------------------------------------------

void init_ACU_CAN()
{
    ACU_.begin();
    ACU_.setBaudRate(500000);
    ACU_.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    for (int i = 0; i < NUM_RX_MAILBOXES; i++)
    {
      ACU_.setMB((FLEXCAN_MAILBOX)i, RX, STD);   
    }
    for (int i = NUM_RX_MAILBOXES; i < (NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++)
    {
      ACU_.setMB((FLEXCAN_MAILBOX)i, TX, STD);
    }
}

int ReadBatteryTemps(CAN_message_t &msg)
{    
  int rxMSG = ACU_.read(msg);
  return rxMSG;
}

void updateAccumulatorCAN()
{  
  if (ReadBatteryTemps(rxMsg))
  {
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
    switch (rxMsg.id)  // This is cancer and could better be implemented with a loop I imagine
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
    default:
      break;
    }
  }
}

void canSniff(const CAN_message_t &msg)
{
  if(msg.id==BMS_Response_ID){
    Serial.print("MB "); Serial.print(msg.mb);
    Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
    Serial.print("  LEN: "); Serial.print(msg.len);
    Serial.print(" EXT: "); Serial.print(msg.flags.extended);
    Serial.print(" TS: "); Serial.print(msg.timestamp);
    Serial.print(" ID: "); Serial.print(msg.id, HEX);
    Serial.print(" Buffer: ");
    for ( uint8_t i = 0; i < msg.len; i++ ) {
      Serial.print(msg.buf[i], HEX); Serial.print(" ");
    } 
    Serial.println();
  }
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

// Sending highest/lowest temperature to the BMS
void sendTempData()
{
  CAN_message_t sendTempMsg;
  sendTempMsg.flags.extended = 1;      // extended id
  sendTempMsg.len = 8;                 // per protocol
  sendTempMsg.id = ThermistorToBMS_ID; // Temp broadcast ID
  enabledTherm = NUMBER_OF_CELLS - 1;  // number of cells 0 based
  int lowTherm = batteryTemps[0], lowestThermId, highTherm = batteryTemps[0], highestThermId;
  for (int i = 0; i < NUMBER_OF_CELLS; i++)
  { // get lowest and highest
    #ifdef DEBUG
    Serial.print("Cell number: ");
    Serial.print(i);
    Serial.print(" Value: ");
    Serial.println(batteryTemps[i]);
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
  memcpy(sendTempMsg.buf, tempdata, sizeof(sendTempMsg.buf));
  CAN_1.write(sendTempMsg);
  // GLobal ints for tracking
  globalHighTherm = highTherm;
  globalLowTherm = lowTherm;
}

// int setChannels(int channelNo)
// {
//   for (int i = 0; i < NUMBER_OF_LTCs; i++)
//   {
//     theThings[i].changeChannel(ADCChannels[channelNo]);
//     return channelNo;
//   }
// }

// void setChannelsSwitchCase(int channelNo)
// {
//   switch (channelNo)
//   {
//   case 0:
//   {
//     for (int i = 0; i < NUMBER_OF_LTCs; i++)
//     {
//       theThings[i].changeChannel(CHAN_SINGLE_0P);
//     }
//   }
//   break;
//   case 1:
//   {
//     for (int i = 0; i < NUMBER_OF_LTCs; i++)
//     {
//       theThings[i].changeChannel(CHAN_SINGLE_1P);
//       break;
//     }
//   }
//   break;
//   case 2:
//   {
//     for (int i = 0; i < NUMBER_OF_LTCs; i++)
//     {
//       theThings[i].changeChannel(CHAN_SINGLE_2P);
//       break;
//     }
//   }
//   break;
//   case 3:
//   {
//     for (int i = 0; i < NUMBER_OF_LTCs; i++)
//     {
//       theThings[i].changeChannel(CHAN_SINGLE_3P);
//     }
//   }
//   break;
//   case 4:
//   {
//     for (int i = 0; i < NUMBER_OF_LTCs; i++)
//     {
//       theThings[i].changeChannel(CHAN_SINGLE_4P);
//     }
//   }
//   break;
//   case 5:
//   {
//     for (int i = 0; i < NUMBER_OF_LTCs; i++)
//     {
//       theThings[i].changeChannel(CHAN_SINGLE_5P);
//     }
//   }
//   break;
//   case 6:
//   {
//     for (int i = 0; i < NUMBER_OF_LTCs; i++)
//     {
//       theThings[i].changeChannel(CHAN_SINGLE_6P);
//       break;
//     }
//   }
//   break;
//   case 7:
//   {
//     for (int i = 0; i < NUMBER_OF_LTCs; i++)
//     {
//       theThings[i].changeChannel(CHAN_SINGLE_7P);
//     }
//   }
//   break;
//   case 8:
//   {
//     for (int i = 0; i < NUMBER_OF_LTCs; i++)
//     {
//       theThings[i].changeChannel(CHAN_SINGLE_8P);
//     }
//   }
//   break;
//   case 9:
//   {
//     for (int i = 0; i < NUMBER_OF_LTCs; i++)
//     {
//       theThings[i].changeChannel(CHAN_SINGLE_9P);
//     }
//   }
//   break;
//   case 10:
//   {
//     for (int i = 0; i < NUMBER_OF_LTCs; i++)
//     {
//       theThings[i].changeChannel(CHAN_SINGLE_10P);
//     }
//   }
//   break;
//   case 11:
//   {
//     for (int i = 0; i < NUMBER_OF_LTCs; i++)
//     {
//       theThings[i].changeChannel(CHAN_SINGLE_11P);
//     }
//   }
//   break;
//   }
// }

// void getTemps(int channelNo)
// {
//   CAN_message_t energus_voltages;
//   energus_voltages.id = 0x6B3;
//   energus_voltages.buf[0] = channelNo;
//   energus_voltages.len = 8;
//   for (int i = 0; i < NUMBER_OF_LTCs; i++)
//   {

//     float v = theThings[i].readVoltage();
//     int cellNum = (i * 12) + channelNo;
//     batteryTempvoltages[cellNum] = v;
//     float vv = v * 100;
//     energus_voltages.buf[i + 1] = (int)vv;
//     Serial.printf("Can Message Test: %d\n", energus_voltages.buf[i + 1]);
//     float temp = (v * -79.256) + 168.4;
//     batteryTemps[cellNum] = temp;
// #ifdef DEBUG
//     char buffer[100];
//     sprintf(buffer, "LTC Number: %d CellNum: %d Channel: %d Reading: %f TempC: %f ", i, cellNum, channelNo, v, temp);
//     Serial.println(buffer);
// #endif
//   }
//   CAN_1.write(energus_voltages);
// }

// #ifdef runningFoReal
// void getAllTheTemps(int channelNo)
// {
//   for (int i = 0; i < NUMBER_OF_LTCs; i++)
//   {
//     theThings[i].changeChannel(CHAN_SINGLE_0P);
//   }
//   conversionTime = 0;
//   delay(100);
//   for (int i = 0; i < NUMBER_OF_LTCs; i++)
//   {
//     float voltageX = theThings[i].readVoltage();
//     Serial.println(voltageX);
//     int readingNumber = (i * 12) + channelNo;
//     batteryTempvoltages[readingNumber] = voltageX;
// #ifdef DEBUG
//     char buffer[50];
//     sprintf(buffer, "LTC Number: %d  Channel: %d Voltage: %f Reading Number: %d", i, channelNo, voltageX, readingNumber);
//     Serial.println(buffer);
// #endif
//   }
//   // Serial.println("=================");
// }
// #endif


// // void ACUStateMachine()
// {
//   // Serial.println("State:");
//   // Serial.println(acuState);
//   switch (acuState)
//   {
//   case 0:
//   {
//     setChannelsSwitchCase(currentChannel);
//     conversionTime = 0;
//     acuState = 1;
//     Serial.println("Setting Channels: ");
//     break;
//   }
//   case 1:
//   {
//     if (conversionTime >= 100)
//     {
//       acuState = 2;
//       Serial.println("Going to get readings");
//     }
//     break;
//   }
//   case 2:
//   {
//     Serial.println("reading channels");
//     getTemps(currentChannel);
//     acuState = 0;
//     currentChannel++;
//     if (currentChannel > 11)
//     {
//       currentChannel = 0;
//     }
//     break;
//   }
//   }
// }

void DebuggingPrintout()
{
  for (int i = 0; i < NUMBER_OF_CELLS; i++)
  {
    Serial.print("Cell Number: ");
    Serial.print(i + 1);
    Serial.print(" Temp: ");
    Serial.print(batteryTemps[i]);
    Serial.println();
  }
}

