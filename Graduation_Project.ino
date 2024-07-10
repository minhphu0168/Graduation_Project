#include <SPI.h>
#include <stdlib.h>
#include <string.h>
#include <Arduino.h>

#include "mcp2515_can.h"
#define CAN_2515

#include <SoftwareSerial.h>
SoftwareSerial espSerial = SoftwareSerial(4,5);

#include "VirtuinoCM.h"
VirtuinoCM virtuino;   
            
#define V_memory_count 7          
float V[V_memory_count]; 
boolean debug = true;
float V1_lastValue = 0;
int counterDemo = 0; 
float V_old[3];

String empty_String = "   ";

String fullString[6] = {"P0113 - IAT",
                        "P0222 - TPS",
                        "P0123 - TPS",
                        "P0201 - INJ",
                        "P0351 - IGN",
                        "P0400 - EGR"};
String codeString[6] = {"P0113",
                        "P0222",
                        "P0123",
                        "P0201",
                        "P0351",
                        "P0400"};  

const int SPI_CS_PIN = 10;
const int CAN_INT_PIN = 2;
mcp2515_can CAN_bus(SPI_CS_PIN);

#define EngineSpeedPID        0x0C
#define VehicleSpeedPID       0x0D
#define CoolantTempPID        0x05
#define PedalPositionPID      0x49

#define nodeID                0x7DF

uint8_t data_query[8] = {0x02, 0x01, 0x0C, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC};
uint8_t query_PID_list[] = {EngineSpeedPID, VehicleSpeedPID, CoolantTempPID, PedalPositionPID};
uint8_t query_pos = 0, data_len, receive_data_value[8];
uint8_t data_readStored_DTCs[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t data_clear_DTCs[8] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t data_readPending_DTCs[8] = {0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint32_t old_time, now_time, receive_id;
uint8_t CoolantTemp, VehicleSpeed;
float Pedal, EngineSpeed;
char read_buf;
uint8_t numOfDTCs, temp_decode;
uint8_t dataDecodeDTCsBuf[20];
bool FulldataDecodeDTCsBuf = false;

char DTCs[16][5];
String sDTCs[16];

void set_mask_filter()
{
  CAN_bus.init_Mask(0, 0, 0x7FC);  
  CAN_bus.init_Mask(1, 0, 0x7FC); 
  CAN_bus.init_Filt(0, 0, 0x7E8);
  CAN_bus.init_Filt(1, 0, 0x7E8);
  CAN_bus.init_Filt(2, 0, 0x7E8);
  CAN_bus.init_Filt(3, 0, 0x7E8);
  CAN_bus.init_Filt(4, 0, 0x7E8);
  CAN_bus.init_Filt(5, 0, 0x7E8);
}

void Query(uint8_t PID)
{
  data_query[2] = PID;
  CAN_bus.sendMsgBuf(nodeID, 0, 8, data_query);
}

void ReadStoredDTCs()
{
  Serial.println("Read stored DTCs.");
  CAN_bus.sendMsgBuf(nodeID, 0, 8, data_readStored_DTCs);
}

void ClearDTCs()
{
  Serial.println("Clear DTCs.");
  CAN_bus.sendMsgBuf(nodeID, 0, 8, data_clear_DTCs);
}

void decode_DTCs()
{
  for (uint8_t j = 0; j < numOfDTCs; j++)
  {
    switch (dataDecodeDTCsBuf[j*2] >> 6)
    {
      case 0:  DTCs[j][0] = 'P';
              break;
      case 1:  DTCs[j][0] = 'C';
              break;
      case 2:  DTCs[j][0] = 'B';
              break;
      case 3:  DTCs[j][0] = 'U';
              break;
      
    }
    DTCs[j][1] = ((dataDecodeDTCsBuf[j*2]&0b00111111) >> 4) + 48;
    temp_decode = dataDecodeDTCsBuf[j*2]&0b00001111;
    if (temp_decode < 10)
      DTCs[j][2] = temp_decode + 48;
    else
      DTCs[j][2] = temp_decode + 55;
    temp_decode = dataDecodeDTCsBuf[j*2 + 1] >> 4;
    if (temp_decode < 10)
      DTCs[j][3] = temp_decode + 48;
    else
      DTCs[j][3] = temp_decode + 55;
    temp_decode = dataDecodeDTCsBuf[j*2 + 1]&0b00001111;
    if (temp_decode < 10)
      DTCs[j][4] = temp_decode + 48;
    else
      DTCs[j][4] = temp_decode + 55;
  }
  printDTCs();
  numOfDTCs = 0;
  FulldataDecodeDTCsBuf = false;
}

void printDTCs()
{
  if (numOfDTCs == 0)
  {
    Serial.println("No DTCs.");
    sDTCs[0] = empty_String;
    sDTCs[1] = empty_String;
    sDTCs[2] = "No DTC";
    sDTCs[3] = empty_String;
    sDTCs[4] = empty_String;
    sDTCs[5] = empty_String;
  }
    

  else
  {
    Serial.print("The engine has ");
    Serial.print(numOfDTCs);
    Serial.println(" error codes:");
    for (uint8_t j = 0; j < numOfDTCs; j++)
    {
      sDTCs[j] = "";
      for (uint8_t i = 0; i < 5; i++)
      {
        sDTCs[j].concat(DTCs[j][i]);
      }  
      for (uint8_t k = 0; k < 6; k++)
      {
        if (sDTCs[j] == codeString[k])  
        {    
          sDTCs[j] = fullString[k];
        }
      }      
      Serial.println(sDTCs[j]);
      
    }
    if (numOfDTCs < 6)
    {
      for (uint8_t l = numOfDTCs; l < 6; l++)
        sDTCs[l] = empty_String;
    }
  }
}

void ReadPendingDTCs()
{
  Serial.println("Read pending DTCs.");
  CAN_bus.sendMsgBuf(nodeID, 0, 8, data_readPending_DTCs);
}

void setup()
{
  pinMode(13, OUTPUT);

  if (debug)
  {
    Serial.begin(115200);
    while (!Serial) continue;
  }
  
  espSerial.begin(115200);  
  espSerial.setTimeout(50);
  virtuino.begin(onReceived, onRequested, 511); 
 
 while (CAN_OK != CAN_bus.begin(CAN_500KBPS))
 {         
   Serial.println("CAN init fail, retry...");
   delay(100);
 }
  Serial.println("CAN init ok!");
  set_mask_filter();
}


void loop()
{
  virtuinoRun();
  now_time = millis();  
  if ((now_time - old_time) >= 100)
  {
    old_time = now_time;
    digitalWrite(13, !digitalRead(13));
    Query(query_PID_list[query_pos]);
    query_pos = (++query_pos)%4;    
  }

  if (CAN_MSGAVAIL == CAN_bus.checkReceive())
  {
    receive_id = CAN_bus.getCanId();
    CAN_bus.readMsgBuf(&data_len, receive_data_value);  
//    Serial.print(receive_id, HEX);
//    for (uint8_t i = 0; i < 8; i++)
//    {
//      Serial.print('\t');
//      Serial.print(receive_data_value[i], HEX);
//    }
//    Serial.println();      
    if ((receive_data_value[1]) == 0x41)
    {
      switch (receive_data_value[2])
      {
        case 0x05:  CoolantTemp = receive_data_value[3] - 40;
//                    Serial.print("CoolantTemp: ");
//                    Serial.println(CoolantTemp);
                      V[3] = CoolantTemp;
                    break;
        case 0x0C:  EngineSpeed = (float)(receive_data_value[3]*235 + receive_data_value[4])/4.0;
//                    Serial.print("EngineSpeed: ");
//                    Serial.println(EngineSpeed);
                      V[1] = EngineSpeed;
                    break;
        case 0x0D:  VehicleSpeed = receive_data_value[3];
//                    Serial.print("VehicleSpeed: ");
//                    Serial.println(VehicleSpeed);
                      V[0] = VehicleSpeed;
                    break;
        case 0x5A:  Pedal = (float)receive_data_value[3]*100/255.0;
//                    Serial.print("Pedal: ");
//                    Serial.println(Pedal);
                      V[2] = Pedal;
                    break;
                    
      }
    }
    if ((receive_data_value[1] == 0x43)||(receive_data_value[1] == 0x47))
    {
      if (receive_data_value[2]*2 == (receive_data_value[0] - 2))
      {
        numOfDTCs = receive_data_value[2];
        dataDecodeDTCsBuf[0] = receive_data_value[3];
        dataDecodeDTCsBuf[1] = receive_data_value[4];
        dataDecodeDTCsBuf[2] = receive_data_value[5];
        dataDecodeDTCsBuf[3] = receive_data_value[6];
        FulldataDecodeDTCsBuf = true;
      }
    }
    if (receive_data_value[0] == 0x10)
    {
      if ((receive_data_value[2] == 0x43)||(receive_data_value[2] == 0x47))
      {
        if (receive_data_value[3]*2 == (receive_data_value[1] - 2))
        {
          numOfDTCs = receive_data_value[3];
          dataDecodeDTCsBuf[0] = receive_data_value[4];
          dataDecodeDTCsBuf[1] = receive_data_value[5];
          dataDecodeDTCsBuf[2] = receive_data_value[6];
          dataDecodeDTCsBuf[3] = receive_data_value[7];
        }
      }
    }
    if (receive_data_value[0] == 0x21)
    {      
      for (uint8_t j = 1; j < 8; j++)
      {
        dataDecodeDTCsBuf[3 + j] = receive_data_value[j];
        if ((3 + j) == (numOfDTCs*2 - 1))
        {
          FulldataDecodeDTCsBuf = true;
          break;
        }
      }
    }
    if (receive_data_value[0] == 0x22)
    {      
      for (uint8_t j = 1; j < 8; j++)
      {
        dataDecodeDTCsBuf[10 + j] = receive_data_value[j];
        if ((10 + j) == (numOfDTCs*2 - 1))
        {
          FulldataDecodeDTCsBuf = true;
          break;
        }
      }
    }
    if (receive_data_value[0] == 0x23)
    {      
      for (uint8_t j = 1; j < 8; j++)
      {
        dataDecodeDTCsBuf[17 + j] = receive_data_value[j];
        if ((17 + j) == (numOfDTCs*2 - 1))
        {
          FulldataDecodeDTCsBuf = true;
          break;
        }
      }
    }
    if (receive_data_value[0] == 0x24)
    {      
      for (uint8_t j = 1; j < 8; j++)
      {
        dataDecodeDTCsBuf[24 + j] = receive_data_value[j];
        if ((24 + j) == (numOfDTCs*2 - 1))
        {
          FulldataDecodeDTCsBuf = true;
          break;
        }
      }
    }
    if (FulldataDecodeDTCsBuf)
    {
      decode_DTCs();
    }
  }

  if ((V_old[0] == 0)&&(V[4] == 1))
  {
    ReadPendingDTCs();
  }
  if ((V_old[1] == 0)&&(V[5] == 1))
  {
    ReadStoredDTCs();
  }
  if ((V_old[2] == 0)&&(V[6] == 1))
    ClearDTCs();

  V_old[0] = V[4];
  V_old[1] = V[5];
  V_old[2] = V[6];
  
  if ((V[4] == 0)&&(V[5] == 0))
  {
    sDTCs[0] = empty_String;
    sDTCs[1] = empty_String;
    sDTCs[2] = empty_String;
    sDTCs[3] = empty_String;
    sDTCs[4] = empty_String;
    sDTCs[5] = empty_String;
  }
  if (Serial.available())
  {
    read_buf = Serial.read();
    if (read_buf == '3')
      ReadStoredDTCs();
    if (read_buf == '4')
      ClearDTCs();
    if (read_buf == '7')
      ReadPendingDTCs();
  }
}

void onReceived(char variableType, uint8_t variableIndex, String valueAsText)
{     
  if (variableType == 'V')
  {
    float value = valueAsText.toFloat();       
    if (variableIndex < V_memory_count)
      V[variableIndex] = value;             
  }    
}

String onRequested(char variableType, uint8_t variableIndex)
{     
  if (variableType == 'V')
  {
    if (variableIndex < V_memory_count) 
      return  String(V[variableIndex]); 
    else if (variableIndex == 7)
      return sDTCs[0];
    else if (variableIndex == 8)
      return sDTCs[1];
    else if (variableIndex == 9)
      return sDTCs[2];
    else if (variableIndex == 10)
      return sDTCs[3];
    else if (variableIndex == 11)
      return sDTCs[4];
    else if (variableIndex == 12)
      return sDTCs[5];
  }
  return "";
}

void virtuinoRun()
{
  while (espSerial.available())
  {
    char tempChar = espSerial.read();
    if (tempChar == CM_START_CHAR)
    {              
      virtuino.readBuffer = CM_START_CHAR;     
      virtuino.readBuffer += espSerial.readStringUntil(CM_END_CHAR);
      virtuino.readBuffer += CM_END_CHAR;
      String* response = virtuino.getResponse();
      espSerial.print(*response);
      break; 
     }
  }
}
 
void vDelay(int delayInMillis)
{
  long t = millis() + delayInMillis;
  while (millis() < t)
    virtuinoRun();
}
