/**************************************************************************
    CNCmodule - Configurable Arduino Loconet Module
    Copyright (C) 2018 Daniel Guisado Serra

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ------------------------------------------------------------------------
 AUTHOR : Dani Guisado - http://www.clubncaldes.com - dguisado@gmail.com
 ------------------------------------------------------------------------
 DESCRIPTION:
    This software emulates the functionality of a GCA50 board from Peter
    Giling (Giling Computer Applications). This is a Loconet Interface
    with 16 I/O that can be individually configured as Input (block sensors)
    or Outputs (switches, lights,...).
    Configuration is done through SV Loconet protocol and can be configured
    from Rocrail (Programming->GCA->GCA50).
 ------------------------------------------------------------------------
 PIN ASSIGNMENT:
   0,1 -> Serial, used to debug and Loconet Monitor (uncomment DEBUG)
   2,3,4,5,6 -> Configurable I/O from 1 to 5
   7 -> Loconet TX (connected to GCA185 shield)
   8 -> Loconet RX (connected to GCA185 shield)
   9,10,11,12,13 -> Configurable I/O from 6 to 10
   A0,A1,A2,A3,A4,A5-> Configurable I/O from 11 to 16
 ------------------------------------------------------------------------
 CREDITS: 
 * Based on MRRwA Loconet libraries for Arduino - http://mrrwa.org/ and 
   the Loconet Monitor example.
 * Inspired in GCA50 board from Peter Giling - http://www.phgiling.net/
 * Idea also inspired in LocoShield from SPCoast - http://www.scuba.net/
 * Thanks also to Rocrail group - http://www.rocrail.org
*************************************************************************/

#include <LocoNet.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define VERSION 300
#define EPROM_SIZE 259

//Uncomment this line to debug through the serial monitor
#define DEBUG

//Custom defines
#define mDEACTIVATED 0
#define mINPUT 1
#define mOUTPUT 2
#define mSERVO 3
#define mSENSOR 0
#define mSWITCH 1
#define mPUSHBTN 2
#define mCONTINUOUS 0
#define mPULSE 1


namespace {
//#define VIDA_LOCOSHIELD_NANO 1

//Arduino pin assignment to each of the 16 outputs
#ifdef VIDA_LOCOSHIELD_NANO
uint8_t pinMap[16]={11,10,9,6,5,4,3,2,15,14,19,18,17,16,13,12};
#else
uint8_t pinMap[16]={2,3,4,5,6,9,10,11,12,13,14,15,16,17,18,19};
#endif
}

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)
#define SERVO_LAPSE 20  //millis between servo movements

//Timers for each input in case of using "block" configuration instead of "input" configuration
//input defined as "block" will keep the signal high at least 2 seconds
unsigned long inpTimer[16];

//3 bytes defining a pin behavior ( http://wiki.rocrail.net/doku.php?id=loconet-io-en )
typedef struct 
{
  uint16_t addr;
  uint8_t cnfg;
  uint8_t value1;
  uint8_t value2;
  uint8_t value3;
  uint8_t value4;
  uint8_t value5;
} PIN_CFG;

//Memory map exchanged with SV read and write commands ( http://wiki.rocrail.net/doku.php?id=lnsv-en )
typedef struct
{
  uint8_t vrsion;
  uint8_t addr_low;
  uint8_t addr_high;
  PIN_CFG pincfg[32];
} SV_TABLE;

//Union to access the data with the struct or by index
typedef union {
  SV_TABLE svt;
  uint8_t data[EPROM_SIZE];
} SV_DATA;

SV_DATA svtable;
lnMsg *LnPacket;

  
void setup()
{
  int n;
  uint16_t myAddr;
  
  // First initialize the LocoNet interface
  LocoNet.init(7);

  // Configure the serial port for 57600 baud
  #ifdef DEBUG
  Serial.begin(9600);
  Serial.print("CNCmodule v.");Serial.println(VERSION);
  #endif 

  //Load config from EEPROM
  for (n=0;n<EPROM_SIZE;n++)
    svtable.data[n]=EEPROM.read(n);

  //Check for a valid config
  if (svtable.svt.vrsion!=VERSION || svtable.svt.addr_low<1 || svtable.svt.addr_low>240 || svtable.svt.addr_high<1 || svtable.svt.addr_high>100 )
  {
    svtable.svt.vrsion=VERSION;
    svtable.svt.addr_low=81;
    svtable.svt.addr_high=1;
    EEPROM.write(0,VERSION);
    EEPROM.write(1, svtable.svt.addr_low);
    EEPROM.write(2, svtable.svt.addr_high);
    //TODO write the rest of config with default values
  }

  //Attacch Servos
  servo.begin();
  servo.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  
  //Configure I/O of onboard pins (0-15 arduino pins, 16-31 pwm servo shield)
  #ifdef DEBUG
  Serial.println("Initializing pins...");
  #endif 

  for (n=0;n<32;n++)
  {
    if (n<16)
      inpTimer[n]=0; //timer initialization
      
    if (svtable.svt.pincfg[n].cnfg==mOUTPUT) 
    {
      #ifdef DEBUG
      Serial.print("Pin ");Serial.print(pinMap[n]); Serial.print(" Output "); Serial.print(n); Serial.print(" Address "); Serial.print(svtable.svt.pincfg[n].addr); Serial.println(" as OUTPUT");
      #endif 
      if (n<16)
        pinMode(pinMap[n],OUTPUT);
        
      changeOutput(n, LOW);
    }        
    else if (svtable.svt.pincfg[n].cnfg==mINPUT && n<16) 
    {
      #ifdef DEBUG
      Serial.print("Pin ");Serial.print(pinMap[n]); Serial.print(" Input "); Serial.print(n); Serial.print(" Address "); Serial.print(svtable.svt.pincfg[n].addr); Serial.println(" as INPUT_PULLUP");
      #endif
      pinMode(pinMap[n],INPUT_PULLUP);
      svtable.svt.pincfg[n].value3=digitalRead(pinMap[n]);
    }
    else if (svtable.svt.pincfg[n].cnfg==mSERVO)
    {
      //Set current position to pos 2
      svtable.svt.pincfg[n].value5=svtable.svt.pincfg[n].value2;
      #ifdef DEBUG    
      Serial.print("Servo "); Serial.print(n-16); Serial.print(" to Pos1 "); Serial.println(svtable.svt.pincfg[n].value1);
      #endif
      //Move servo to pos1
      moveServo(n-16, svtable.svt.pincfg[n].value1);
      delay(300);
    }
  }
  
  Serial.print("Module ");Serial.print(svtable.svt.addr_low);Serial.print("/");Serial.println(svtable.svt.addr_high);
}

void loop()
{  
  int n;
  bool hasChanged;
  int currentState;
  
  // Check for any received LocoNet packets
  LnPacket = LocoNet.receive() ;
  if( LnPacket )
  {
    #ifdef DEBUG 
    // First print out the packet in HEX
    Serial.print("RX: ");
    uint8_t msgLen = getLnMsgSize(LnPacket); 
    for (uint8_t x = 0; x < msgLen; x++)
    {
      uint8_t val = LnPacket->data[x];
      // Print a leading 0 if less than 16 to make 2 HEX digits
      if(val < 16)
        Serial.print('0');        
      Serial.print(val, HEX);
      Serial.print(' ');
    }
    Serial.println();
    #endif  

    // If this packet was not a Switch or Sensor Message checks por PEER packet
    if(!LocoNet.processSwitchSensorMessage(LnPacket))
    {             
      processPeerPacket();
    }
  }
  
  // Check inputs to inform 
  for (n=0; n<16; n++)
  {
    if (svtable.svt.pincfg[n].cnfg==1)   //Setup as an Input
    {
      //Read current state of the pin
      currentState=digitalRead(pinMap[n]);

      //If state has not changed, update timer to use in case of delayed deactivation (sensor) and continue
      if (currentState==svtable.svt.pincfg[n].value3)
      {
        inpTimer[n]=millis();
        continue;
      }
      //If puch button and changed to not pressed (HIGH) do nothing, there is no change of state
      if (svtable.svt.pincfg[n].value1==2 && currentState==HIGH)
        continue;
        
      //check if is a BLOCK DETECTOR for delayed deactivation (as we use pullup resistor, deactivation is HIGH)
      if (svtable.svt.pincfg[n].value1==0 && currentState==HIGH)
      {
        if ((millis()-inpTimer[n])<1000)
          continue;
      }
      
      //------- From here we have a change of state ---------------
      
      //If push button invert state, if not take the current state of the pin
      if (svtable.svt.pincfg[n].value1==2)
        svtable.svt.pincfg[n].value3=!svtable.svt.pincfg[n].value3;
      else
        svtable.svt.pincfg[n].value3=currentState;  
      
      #ifdef DEBUG
      Serial.print("INPUT ");Serial.print(n);
      Serial.print(" IN PIN "); Serial.print(pinMap[n]);
      Serial.print(" CHANGED, INFORM "); Serial.println(svtable.svt.pincfg[n].addr);
      #endif
      //OPC_INPUT_REP LocoNet.send(OPC_INPUT_REP, svtable.svt.pincfg[n].value1, svtable.svt.pincfg[n].value2);
      sendOPC_INPUT_REP(svtable.svt.pincfg[n].addr, svtable.svt.pincfg[n].value3);
    }
  }
    
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Sensor messages
void notifySensor( uint16_t Address, uint8_t State )
{
  #ifdef DEBUG
  Serial.print("Sensor: ");
  Serial.print(Address, DEC);
  Serial.print(" - ");
  Serial.println( State ? "Active" : "Inactive" );
  #endif
}

  // This call-back function is called from LocoNet.processSwitchSensorMessage
  // for all Switch Request messages
void notifySwitchRequest( uint16_t Address, uint8_t Output, uint8_t Direction )
{
  int n;
  uint16_t myAddr;
  
  //Direction must be changed to 0 or 1, not 0 or 32
  Direction ? Direction=1 : Direction=0;
  
  #ifdef DEBUG
  Serial.print("Switch Request: ");
  Serial.print(Address, DEC);
  Serial.print(':');
  Serial.print(Direction ? "Closed/Green" : "Thrown/Red");
  Serial.print(" - ");
  Serial.println(Output ? "On" : "Off");
  #endif
  
  //Check if the Address is assigned, configured as output and same Direction
  for (n=0; n<32; n++)
  {
    //Check pint has the requested address and is not an input
    if (svtable.svt.pincfg[n].addr != Address-1 || svtable.svt.pincfg[n].cnfg==mINPUT) 
      continue;
 
    //If it is an output
    if (svtable.svt.pincfg[n].cnfg==mOUTPUT)
    {
      //If operation is normal and green On, operation green and green on, operation red and red on => activates
      if ((svtable.svt.pincfg[n].value4==0 && Direction && Output) || (svtable.svt.pincfg[n].value4==1 && Direction && Output) || (svtable.svt.pincfg[n].value4==2 && !Direction && Output))
        changeOutput(n, HIGH);

      //If operation is normal and red On, operation green and green off, operation red and red off => deactivates
      if ((svtable.svt.pincfg[n].value4==0 && !Direction && Output) || (svtable.svt.pincfg[n].value4==1 && Direction && !Output) || (svtable.svt.pincfg[n].value4==2 && !Direction && !Output))
        changeOutput(n, LOW);
    }

    //If it is a servo (pins 16 to 31)
    if (n>15 && n<32 && svtable.svt.pincfg[n].cnfg==mSERVO)
    {
      //If operation is normal and green On, operation green and green on, operation red and red on => servo to pos1
      if ((svtable.svt.pincfg[n].value4==0 && Direction && Output) || (svtable.svt.pincfg[n].value4==1 && Direction && Output) || (svtable.svt.pincfg[n].value4==2 && !Direction && Output))
        moveServo(n-16, svtable.svt.pincfg[n].value1);

      //If operation is normal and red On, operation green and green off, operation red and red off => servo to pos2
      if ((svtable.svt.pincfg[n].value4==0 && !Direction && Output) || (svtable.svt.pincfg[n].value4==1 && Direction && !Output) || (svtable.svt.pincfg[n].value4==2 && !Direction && !Output))
        moveServo(n-16, svtable.svt.pincfg[n].value2);        
    }
  }
}

  // This call-back function is called from LocoNet.processSwitchSensorMessage
  // for all Switch Report messages
void notifySwitchReport( uint16_t Address, uint8_t Output, uint8_t Direction )
{
  #ifdef DEBUG
  Serial.print("Switch Report: ");
  Serial.print(Address, DEC);
  Serial.print(':');
  Serial.print(Direction ? "Closed" : "Thrown");
  Serial.print(" - ");
  Serial.println(Output ? "On" : "Off");
  #endif
}

  // This call-back function is called from LocoNet.processSwitchSensorMessage
  // for all Switch State messages
void notifySwitchState( uint16_t Address, uint8_t Output, uint8_t Direction )
{
  #ifdef DEBUG
  Serial.print("Switch State: ");
  Serial.print(Address, DEC);
  Serial.print(':');
  Serial.print(Direction ? "Closed" : "Thrown");
  Serial.print(" - ");
  Serial.println(Output ? "On" : "Off");
  #endif
}

boolean processPeerPacket()
{
  //Check is a OPC_PEER_XFER message
  if (LnPacket->px.command != OPC_PEER_XFER) return(false);

  //Check is my destination
  if ((LnPacket->px.dst_l!=0 || LnPacket->px.d5!=0) &&
      (LnPacket->px.dst_l!=0x7f || LnPacket->px.d5!=svtable.svt.addr_high) &&
      (LnPacket->px.dst_l!=svtable.svt.addr_low || LnPacket->px.d5!=svtable.svt.addr_high))
  {
    #ifdef DEBUG
    Serial.println("OPC_PEER_XFER not for me!");
    Serial.print("LnPacket->px.dst_l: ");Serial.print(LnPacket->px.dst_l);Serial.print(" Addr low: ");Serial.println(svtable.svt.addr_low);
    Serial.print("LnPacket->px.d5: ");Serial.print(LnPacket->px.d5);Serial.print(" Addr high: ");Serial.println(svtable.svt.addr_high);
    Serial.print("LnPacket->px.dst_h: ");Serial.print(LnPacket->px.dst_h);Serial.print(" Addr high: ");Serial.println(svtable.svt.addr_high);
    Serial.print("LnPacket->px.d1: ");Serial.println(LnPacket->px.d1);
    Serial.print("LnPacket->px.d2: ");Serial.println(LnPacket->px.d2);
    #endif
    return(false);
  }
    

  //Set high bits in right position
  bitWrite(LnPacket->px.d1,7,bitRead(LnPacket->px.pxct1,0));
  bitWrite(LnPacket->px.d2,7,bitRead(LnPacket->px.pxct1,1));
  bitWrite(LnPacket->px.d3,7,bitRead(LnPacket->px.pxct1,2));
  bitWrite(LnPacket->px.d4,7,bitRead(LnPacket->px.pxct1,3));
  
  bitWrite(LnPacket->px.d5,7,bitRead(LnPacket->px.pxct2,0));
  bitWrite(LnPacket->px.d6,7,bitRead(LnPacket->px.pxct2,1));
  bitWrite(LnPacket->px.d7,7,bitRead(LnPacket->px.pxct2,2));
  bitWrite(LnPacket->px.d8,7,bitRead(LnPacket->px.pxct2,3));

  //OPC_PEER_XFER D1 -> Command (1 SV write, 2 SV read)
  //OPC_PEER_XFER D2 -> Register to read or write
  if (LnPacket->px.d1==2)
  {
    #ifdef DEBUG
    Serial.print("READ ");Serial.print(LnPacket->px.d2);Serial.print(" ");Serial.print(LnPacket->px.d2+1);Serial.print(" ");Serial.println(LnPacket->px.d2+2);
    #endif
    sendPeerPacket(svtable.data[LnPacket->px.d2], svtable.data[LnPacket->px.d2+1], svtable.data[LnPacket->px.d2+2]);
    return (true);
  }
  
  //Write command
  if (LnPacket->px.d1==1)
  {
    //SV 0 contains the program version (write SV0 == RESET? )
    if (LnPacket->px.d2>0)
    {
      //Store data
      svtable.data[LnPacket->px.d2]=LnPacket->px.d4;
      EEPROM.write(LnPacket->px.d2,LnPacket->px.d4);
      
      #ifdef DEBUG
      Serial.print("ESCRITURA "); Serial.print(LnPacket->px.d2); Serial.print(" <== ");
      Serial.print(LnPacket->px.d4); Serial.print(" | ");
      Serial.print(LnPacket->px.d4, HEX); Serial.print(" | ");
      Serial.println(LnPacket->px.d4, BIN);
      #endif
    }

    //Answer packet        
    sendPeerPacket(0x00, 0x00, LnPacket->px.d4);
    #ifdef DEBUG
    Serial.println(">> OPC_PEER_XFER answer sent");
    #endif
    return (true);
  }
  
  return (false);
  
}

void sendPeerPacket(uint8_t p0, uint8_t p1, uint8_t p2)
{
  lnMsg txPacket;

  txPacket.px.command=OPC_PEER_XFER;
  txPacket.px.mesg_size=0x10;
  txPacket.px.src=svtable.svt.addr_low;
  txPacket.px.dst_l=LnPacket->px.src;
  txPacket.px.dst_h=LnPacket->px.dst_h; 
  txPacket.px.pxct1=0x00;
  txPacket.px.d1=LnPacket->px.d1;  //Original command
  txPacket.px.d2=LnPacket->px.d2;  //SV requested
  txPacket.px.d3=svtable.svt.vrsion;
  txPacket.px.d4=0x00;
  txPacket.px.pxct2=0x00;
  txPacket.px.d5=svtable.svt.addr_high; //SOURCE high address
  txPacket.px.d6=p0;
  txPacket.px.d7=p1;
  txPacket.px.d8=p2;

  //Set high bits in right position  
  bitWrite(txPacket.px.pxct1,0,bitRead(txPacket.px.d1,7));
  bitClear(txPacket.px.d1,7);
  bitWrite(txPacket.px.pxct1,1,bitRead(txPacket.px.d2,7));
  bitClear(txPacket.px.d2,7);
  bitWrite(txPacket.px.pxct1,2,bitRead(txPacket.px.d3,7));
  bitClear(txPacket.px.d3,7);
  bitWrite(txPacket.px.pxct1,3,bitRead(txPacket.px.d4,7));
  bitClear(txPacket.px.d4,7);
  bitWrite(txPacket.px.pxct2,0,bitRead(txPacket.px.d5,7));
  bitClear(txPacket.px.d5,7);
  bitWrite(txPacket.px.pxct2,1,bitRead(txPacket.px.d6,7));
  bitClear(txPacket.px.d6,7);
  bitWrite(txPacket.px.pxct2,2,bitRead(txPacket.px.d7,7));
  bitClear(txPacket.px.d7,7);
  bitWrite(txPacket.px.pxct2,3,bitRead(txPacket.px.d8,7));
  bitClear(txPacket.px.d8,7);
   
  LocoNet.send(&txPacket);
  
  #ifdef DEBUG
  Serial.println("OPC_PEER_XFER Packet sent!");
  #endif
}

// Function to change an output
// pOutNum is a value between 0 and 31
//   0 to 15 are mapped to the right pin
//   16 to 31 use the servo shield
// pState is HIGH or LOW
//   libraty implement the logic normal or inverted and intensity of the output
void changeOutput(uint8_t pOutNum, uint8_t pState)
{
  //Make sure pin is configured as output
  if (svtable.svt.pincfg[pOutNum].cnfg!=2)
    return;

  //Pulse and LOW does nothing, deactivation is not by software
  if (svtable.svt.pincfg[pOutNum].value1==1 && pState==LOW)
    return;
    
  //Outputs 16 to 31 using the servo shield
  if (pOutNum>16)
  {
    //If high and normal OR low and inverted, change to high
    if ((pState==HIGH && svtable.svt.pincfg[pOutNum].value2==0) || (pState==LOW && svtable.svt.pincfg[pOutNum].value2==1))
      servo.setPWM(pOutNum-16, 0,map(svtable.svt.pincfg[pOutNum].value3,0, 255, 0, 4096));
    else
      servo.setPWM(pOutNum-16, 4096, 0);

    //Check if we have to deactivate it automatically because it is a pulse
    if (pState==HIGH && svtable.svt.pincfg[pOutNum].value1==1)
    {
      delay(150);
      if (svtable.svt.pincfg[pOutNum].value2==0)
        servo.setPWM(pOutNum-16, 4096, 0);
      else
        servo.setPWM(pOutNum-16, 0, map(svtable.svt.pincfg[pOutNum].value3,0, 255, 0, 4096));

      return;
    }
  }

  //-- Normal pins
  
  //If high and normal OR low and inverted, change to high
    if ((pState==HIGH && svtable.svt.pincfg[pOutNum].value2==0) || (pState==LOW && svtable.svt.pincfg[pOutNum].value2==1))
      analogWrite(pinMap[pOutNum],svtable.svt.pincfg[pOutNum].value3);
    else  
      analogWrite(pinMap[pOutNum],LOW);

    //Check if we have to deactivate it automatically because it is a pulse
    if (pState==HIGH && svtable.svt.pincfg[pOutNum].value1==1)
    {
      delay(150);
      if (svtable.svt.pincfg[pOutNum].value2==0)
        analogWrite(pinMap[pOutNum],LOW);
      else
        analogWrite(pinMap[pOutNum],svtable.svt.pincfg[pOutNum].value3);       
  }
}

// moves a servo according to the servoDestPos[#servo]
// and the configured speed
void moveServo(int pNumServo, int pDestPos)
{
  int grades;
  int steps;
  bool cambiado=false;
  int midgrades;
  int initPos=0;
  int finPos=0;
  
  //if servo already in desired position exit
  if (pDestPos==svtable.svt.pincfg[pNumServo].value5) return;
  
  //read configuration servo speed 0 - 5
  steps=10-svtable.svt.pincfg[pNumServo].value3;

  //To move servos with the shield ressolution
  initPos=map(svtable.svt.pincfg[pNumServo].value5, 0, 360, SERVOMIN, SERVOMAX);
  finPos=map(pDestPos, 0, 360, SERVOMIN, SERVOMAX);
  midgrades=abs(finPos+initPos)/2;
    
  #ifdef DEBUG    
  Serial.print("MOVING SERVO ");Serial.println(pNumServo);
  Serial.print("From     : ");Serial.println(svtable.svt.pincfg[pNumServo].value5);
  Serial.print("MidGrades: ");Serial.println(midgrades);
  Serial.print("To       : ");Serial.println(pDestPos);
  Serial.print("Speed    : ");Serial.println(steps);
  #endif     
  
  if (initPos<finPos)
  {
    // increment grades    
    for (grades=initPos;grades<=finPos;grades++)
    {
      servo.setPWM(pNumServo, 0, grades);
      delay(SERVO_LAPSE*steps);

      //Relay polarization      
      if (!cambiado && grades>midgrades)
      {
        if (svtable.svt.pincfg[pNumServo].value4==1)
        {
          servo.setPWM(pNumServo+8, 0, 4096);
          #ifdef DEBUG    
          Serial.print("FROG ON ");Serial.println(pNumServo+8);
          #endif     
        }
        else if (svtable.svt.pincfg[pNumServo].value4==2)
        {
          //digitalWrite(PIN_RELAY+pNumServo, HIGH);
          servo.setPWM(pNumServo+8, 4096, 0);
          #ifdef DEBUG    
          Serial.print("FROG OFF ");Serial.println(pNumServo+8);
          #endif     
        } 
        cambiado=true;  
      }        
    }
    sendOPC_INPUT_REP(svtable.svt.pincfg[pNumServo].addr, 1);       
  }
  else
  {
    // decrement grades
    for (grades=initPos;grades>=finPos;grades--)
    {
      servo.setPWM(pNumServo, 0, grades);
      delay(SERVO_LAPSE*steps);

      //Relay polarization      
      if (!cambiado && grades>midgrades)
      {
        if (svtable.svt.pincfg[pNumServo].value4==1)
        {
          servo.setPWM(pNumServo+8, 0, 4096);
          #ifdef DEBUG    
          Serial.print("FROG ON ");Serial.println(pNumServo+8);
          #endif     
        }
        else if (svtable.svt.pincfg[pNumServo].value4==2)
        {
          //digitalWrite(PIN_RELAY+pNumServo, HIGH);
          servo.setPWM(pNumServo+8, 4096, 0);
          #ifdef DEBUG    
          Serial.print("FROG OFF ");Serial.println(pNumServo+8);
          #endif     
        } 
        cambiado=true;  
      }        
    }
    sendOPC_INPUT_REP(svtable.svt.pincfg[pNumServo].addr, 0);         
  }
  svtable.svt.pincfg[pNumServo].value5=pDestPos;
}

void sendOPC_INPUT_REP(int address, byte on) {
        lnMsg SendPacket;
        
        SendPacket.data[ 0 ] = OPC_INPUT_REP;  
        SendPacket.data[ 1 ] = address & 0x7F;  // turnout address
        int in2 = B01000000;
        if (on)  in2 |= B00010000;
        in2 |= (address >> 7) & 0x0F;
        SendPacket.data[ 2 ] = in2;            // sw2 contains direction, on/off and hi nibble of address

        LocoNet.send( &SendPacket ) ;
}


