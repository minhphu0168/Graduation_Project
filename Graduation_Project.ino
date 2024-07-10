#include <SPI.h>
#include <stdlib.h>
#include <string.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "mcp2515_can.h"
      
#define CAN_2515
const int SPI_CS_PIN = 10;
const int CAN_INT_PIN = 2;
mcp2515_can CAN_bus(SPI_CS_PIN);

#define EngineSpeedPID        0x0C
#define VehicleSpeedPID       0x0D
#define CoolantTempPID        0x05
#define PedalPositionPID      0x49

#define nodeID                0x7DF

SoftwareSerial softUART (4,5); 

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
String buff;

void set_mask_filter()
{
  CAN_bus.init_Mask(0, 0, 0x7FC);   // 0x7E8
  CAN_bus.init_Mask(1, 0, 0x7FC);   // 0x7E8
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
  CAN_bus.sendMsgBuf(nodeID, 0, 8, data_readStored_DTCs);
}

void ClearDTCs()
{
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
    softUART.print("No DTCs.");
    softUART.print(",");
  }
    
  else
  {
    for (uint8_t j = 0; j < numOfDTCs; j++)
    {
      for (uint8_t i = 0; i < 5; i++)
      {
        Serial.print((char)DTCs[j][i]);
        softUART.print((char)DTCs[j][i]);
      }
      Serial.println();
    }
    softUART.print(",");
  }
}

void ReadPendingDTCs()
{
  CAN_bus.sendMsgBuf(nodeID, 0, 8, data_readPending_DTCs);
}

void setup()
{
  pinMode(13, OUTPUT);
  Serial.begin(115200);
  softUART.begin(9600);

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
  now_time = millis();  
  if ((now_time - old_time) >= 250)
  {
    old_time = now_time;
    digitalWrite(13, !digitalRead(13));
    Query(query_PID_list[query_pos]);
    query_pos = (++query_pos)%4;

    // softUART.print(CoolantTemp);
    // softUART.print(",");
    // softUART.print(EngineSpeed);
    // softUART.print(",");
    // softUART.print(VehicleSpeed);
    // softUART.print(",");
    // softUART.print(Pedal);
    // softUART.print(",");
  }

  if (CAN_MSGAVAIL == CAN_bus.checkReceive())
  {
    receive_id = CAN_bus.getCanId();
    CAN_bus.readMsgBuf(&data_len, receive_data_value);    
    if ((receive_data_value[1]) == 0x41)
    {
      switch (receive_data_value[2])
      {
        case 0x05:  CoolantTemp = receive_data_value[3] - 40;
                    Serial.print("CoolantTemp: ");
                    Serial.println(CoolantTemp);
                    softUART.print(CoolantTemp);
                    softUART.print(",");
                    break;
        case 0x0C:  EngineSpeed = (float)(receive_data_value[3]*235 + receive_data_value[4])/4.0;
                    Serial.print("EngineSpeed: ");
                    Serial.println(EngineSpeed);
                    softUART.print(EngineSpeed);
                    softUART.print(",");
                    break;
        case 0x0D:  VehicleSpeed = receive_data_value[3];
                    Serial.print("VehicleSpeed: ");
                    Serial.println(VehicleSpeed);
                    softUART.print(VehicleSpeed);
                    softUART.print(",");
                    break;
        case 0x49:  Pedal = (float)receive_data_value[3]*100/255.0;
                    Serial.print("Pedal: ");
                    Serial.println(Pedal);
                    softUART.print(Pedal);
                    softUART.print(",");
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

  if (softUART.available())
  {
    buff = softUART.readString ();

    if(buff == "CLEAR")
    {
      ClearDTCs();
    }
  }
}
