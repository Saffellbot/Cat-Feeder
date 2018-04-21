#include <SPIFlash.h>



//#include <RFM69.h>
//#include <RFM69registers.h>
//#include <RFM69_ATC.h>
//#include <RFM69_OTA.h>

//Includes//

#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69, radio library.
#include <SPI.h>  //For comms with radio.
//#include <LowPower.h> //get library from: https://github.com/lowpowerlab/lowpower, for sleeping micro.
#include <RFM69_ATC.h>//get it here: https://www.github.com/lowpowerlab/, radio library with power level control.
#include <LDR10k.h>  //10k nominal resistance photoresistor library.
#include <Thermistor10k.h>  //10k nominal resistance photoresistorl ibrary.
#include <EEPROM.h>

//Radio Settings//

#define NETWORKID     100  //the same on all nodes that talk to each other
#define NODEID        7  
#define GATEWAYID     1
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "16CHARACTERS1234" //exactly the same 16 characters/bytes on all nodes!
//#define IS_RFM69HW    //uncomment only for RFM69HW! Remove/comment if you have RFM69W!

//Connections//

const int LEDPIN=9;

//Setpoints//

struct setpointsObject
{
unsigned long feedPeriod=30000;
unsigned long feedDuration=1000;
};

const int eePromAddress=0;

//DC Motor//

const int MOTORPIN=5;

//Servo//

//Light//

const int LDRPIN=A5;
const int LDRRESISTOR=9800;
float ldrRaw=0.0;
float ldrFiltered=0.0;
float ldrFilterFactor=0.7;

//Temperature//

const int THERMISTORPIN=A3;
const int THERMISTORRESISTOR=9800;
float thermistorRaw=0.0;
float thermistorFiltered=0.0;
float thermistorFilterFactor=0.3;

//Incoming Comms//

unsigned long recieveBuffer;  //Buffer to send messages to base station.
unsigned long exponent;

//Outgoing Comms//

char transmitBuffer[30];  //Buffer to send messages to base station.
int sendSize=0;  //Used for radio transmissions.

const char SENSORNAME1[3]={"LI"};
const byte PRECISION1 = 3;  //Used for dtostrf() later.
char floatBuffer1[PRECISION1+4];

const char SENSORNAME2[3]={"TE"};
const byte PRECISION2 = 2;  //Used for dtostrf() later.
char floatBuffer2[PRECISION2+4];

//const char SENSORNAME3[3]={""};
//const byte PRECISION3 = 2;  //Used for dtostrf() later.
//char floatBuffer3[PRECISION3+4];

const char SENSORNAME4[3]="TL";
int sensorValue4=0;

const char SENSORNAME5[3]="SS";
int sensorValue5=0;

const char SENSORNAME6[3]="RT";
int sensorValue6=0;

//Timing Control//

unsigned long environmentalTimer=0;
int environmentalCounter=0;
const unsigned int ENVIRONMENTALCALLTIME=60000;
unsigned long feedTimer=0;

//Objects//

setpointsObject setpoints;
LDR10k ldr(LDRPIN, LDRRESISTOR);
thermistor10k thermistor(THERMISTORPIN, THERMISTORRESISTOR);
RFM69_ATC radio;

//Prototype Functions//
void feedKitties();
void environmentals();
void radioToNumber();
void readTemperature();
void readLight();
void transmitLevel();
void signalStrength();
void enclosureTemperature();

void setup()
{
	Serial.begin(9600);
	
	radio.initialize(FREQUENCY,NODEID,NETWORKID); //Startup radio
  radio.encrypt(ENCRYPTKEY);
  radio.enableAutoPower(-60);
	
	ldrFiltered=ldr.readLDR();
	thermistorFiltered=thermistor.readThermistor();
	
	
	pinMode(LEDPIN, OUTPUT);
	pinMode(MOTORPIN, OUTPUT);
	
	EEPROM.get(eePromAddress, setpoints);  //Make sure to put these in eeprom the first time.

 Serial.print("FEED PERIOD: ");
 Serial.println(setpoints.feedPeriod);
 Serial.print("FEED DURATION: ");
 Serial.println(setpoints.feedDuration);
}

void loop()
{
  //Serial.println(setpoints.feedDuration);
  //Serial.println(setpoints.feedPeriod);
  //delay(1000);
	if (radio.receiveDone())
	{
		if (radio.DATA[0]=='F' && radio.DATA[1]=='P')
		{
      radioToNumber();
      if (recieveBuffer!=0 && recieveBuffer!=4006342006)
      {
			  setpoints.feedPeriod=recieveBuffer;
        Serial.print("FEED PERIOD: ");
        Serial.println(setpoints.feedPeriod);
			  EEPROM.put(eePromAddress, setpoints);
      }
		}
		
		if (radio.DATA[0]=='F' && radio.DATA[1]=='D')
		{
      radioToNumber();
      if (recieveBuffer!=0  && recieveBuffer!=4006342006)
      {
        setpoints.feedDuration=recieveBuffer;
        Serial.print("FEEDDURATION: ");
        Serial.println(setpoints.feedDuration);
			  EEPROM.put(eePromAddress, setpoints);
      }
		}
		
		if (radio.DATA[0]=='F' && radio.DATA[1]=='K')
		{
			feedKitties();
		}
		
		if (radio.ACKRequested())
    {
      radio.sendACK();
    }
	}

	if ((millis()-feedTimer)>setpoints.feedPeriod)
	{
		feedKitties();
		feedTimer=millis();
	}

	if ((millis()-environmentalTimer)>ENVIRONMENTALCALLTIME)
  {
   	environmentals();
   	environmentalTimer=millis();
  }
}

void feedKitties()
{
	//servo to position 1
	digitalWrite(MOTORPIN, HIGH);
  digitalWrite(LEDPIN, HIGH);
	delay(setpoints.feedDuration);
	digitalWrite(MOTORPIN, LOW);
  digitalWrite(LEDPIN, LOW);
  Serial.println("FEEDIN TIME");
  Serial.print("FEED PERIOD: ");
  Serial.println(setpoints.feedPeriod);
  Serial.print("FEEDDURATION: ");
  Serial.println(setpoints.feedDuration);
	//servo to position 2
	//dispense food
	//servo to position 3
	//dispense food
	
	return;
}


void environmentals()
{
  //Serial.print("Environmental Counter: ");
  //Serial.println(environmentalCounter);
	if (environmentalCounter==0 || environmentalCounter==5)
	{
		readTemperature();
	}
	
	if (environmentalCounter==1 || environmentalCounter==6)
	{
		readLight();
	}
	
	if (environmentalCounter==2)
	{
		transmitLevel();
	}
	
	if (environmentalCounter==3)
	{
		signalStrength();
	}

  if (environmentalCounter==4)
 	{
    enclosureTemperature();
  }

  environmentalCounter++;
  
	if (environmentalCounter==9)
	{
		environmentalCounter=0;
	}

	return;	
}

void radioToNumber()
{
  int bufferLength=radio.DATALEN;
  bufferLength--;
  for (int i=bufferLength; i>=3; i--)
      {
        if(i==bufferLength)
        {
          recieveBuffer=0;
          exponent=1;
        }
        else
        {
          exponent=exponent*10;
        }
        recieveBuffer=recieveBuffer+((radio.DATA[i]-'0')*exponent);
      }
}

void readTemperature()
{
  thermistorRaw=thermistor.readThermistor();
  thermistorFiltered=(thermistorFilterFactor*thermistorRaw)+((1-thermistorFilterFactor)*thermistorFiltered);
  Serial.print("Temperature: ");
  Serial.println(thermistorFiltered);
  dtostrf(thermistorFiltered, PRECISION2+3, PRECISION2, floatBuffer2);  //Converts battery voltage to a string to be sent out for debugging.
  sprintf(transmitBuffer, "$%d@%s#%s*",NODEID,SENSORNAME2,floatBuffer2);
  sendSize = strlen(transmitBuffer);
  radio.sendWithRetry(GATEWAYID, transmitBuffer, sendSize);
    
  return;
}

void readLight()
{
  ldrRaw=ldr.readLDR();
  ldrFiltered=(ldrFilterFactor*ldrRaw)+((1-ldrFilterFactor)*ldrFiltered);
  Serial.print("Light: ");
  Serial.println(ldrFiltered);
  dtostrf(ldrFiltered, PRECISION1+3, PRECISION1, floatBuffer1);  //Converts battery voltage to a string to be sent out for debugging.
  sprintf(transmitBuffer, "$%d@%s#%s*",NODEID,SENSORNAME1,floatBuffer1);
  sendSize = strlen(transmitBuffer);
  radio.sendWithRetry(GATEWAYID, transmitBuffer, sendSize);
    
  return;
}

void transmitLevel()
{
  
  sensorValue4=radio._transmitLevel;
  Serial.print("Transmit Level: ");
  Serial.println(sensorValue4);
  sprintf(transmitBuffer, "$%d@%s#%d*",NODEID,SENSORNAME4,sensorValue4);
  sendSize = strlen(transmitBuffer);
  radio.sendWithRetry(GATEWAYID, transmitBuffer, sendSize);
  sensorValue5=radio.RSSI;
    
  return;
}

void signalStrength()
{
  sprintf(transmitBuffer, "$%d@%s#%d*",NODEID,SENSORNAME5,sensorValue5);
  sendSize = strlen(transmitBuffer);
  radio.sendWithRetry(GATEWAYID, transmitBuffer, sendSize);
    
  return;
}

void enclosureTemperature()
{
  sensorValue6=radio.readTemperature(0)*1.8+32;
  sprintf(transmitBuffer, "$%d@%s#%d*",NODEID,SENSORNAME6,sensorValue6);
  sendSize = strlen(transmitBuffer);
  radio.sendWithRetry(GATEWAYID, transmitBuffer, sendSize);
    
  return;
}
