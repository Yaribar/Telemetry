/*
    Name:       Telemetry
    Author:     YARIBAR
*/

#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_RX_BUFFER 512
#define RXD2 16 //Hardware Serial
#define TXD2 17 //Hardware Serial
#define SAMPLING_PERIOD 100
#define PPR 1

#include <Arduino.h>
#include <TinyGsmClient.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <Kalman_Filter.h>
#include <BasicLinearAlgebra.h>
#include "MPU6050_tockn.h"
#include <Adafruit_ADS1X15.h>
#include <SPI.h>

const uint8_t channelRight = 33;
const uint8_t channelLeft = 32;
static volatile int16_t ISRCounterRight = 0;
static volatile int16_t ISRCounterLeft = 0;
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
static uint8_t pulseCounterRight; 
static uint8_t pulseCounterLeft; 
static unsigned long currentRevRight; 
static unsigned long currentRevLeft; 
static unsigned long previousRevRight; 
static unsigned long previousRevLeft; 
float RPMR=0.0,RPML=0.0;


//************************
//**  GPRS CREDENTIALS  **
//************************

const char apn[] = "internet.itelcel.com";
const char user[] = "webgprs";
const char pass[] = "webgprs2002";

//**************************************
//*********** MQTT CONFIG **************
//**************************************

const char *mqtt_server = "c-iot.ml";
const uint16_t mqtt_port = 1883;
const char *mqtt_user = "ebccm";
const char *mqtt_pass = "ebccm_dash";

TinyGsm modem(Serial2);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

//**************************************
//***********  GLOBALS  ****************
//**************************************

char msg[25];
long count = 0;

//**************************************
//************* FUNCTION ***************
//**************************************

void setupModem();
void reconnect();
void getSpeed(int sampling_time);
//void callback(char *topic, byte *payload, unsigned int length);

MPU6050 mpu6050(Wire);
float accX, accY, accZ;

ulong start_time_imu;
ulong start_time;
ulong start_time_speed;

//****************************************
//**************** ADS *******************
//****************************************

Adafruit_ADS1115 ads1;
//Adafruit_ADS1115 ads2;

int16_t adc1[4];
int16_t adc2[4];

float voltageFR=0.0;
float voltageFL=0.0;
float voltageBR=0.0;
float voltageBL=0.0;

float current=0.0;

//****************************************
//**************** ISR *******************
//****************************************

void ISRencoderRight(){
  portENTER_CRITICAL_ISR(&spinlock);
  ISRCounterRight++;
  portEXIT_CRITICAL_ISR(&spinlock);
}

void ISRencoderLeft(){
  portENTER_CRITICAL_ISR(&spinlock);
  ISRCounterLeft++;
  portEXIT_CRITICAL_ISR(&spinlock);
}


void setup()
{
	
	Serial.begin(115200);
  	Serial2.begin(115200, SERIAL_8N1, RXD2,TXD2); //Baud rate

    pinMode(channelRight, INPUT_PULLUP); 
    pinMode(channelLeft, INPUT_PULLUP);  
    attachInterrupt(digitalPinToInterrupt(channelRight), ISRencoderRight, CHANGE);
    attachInterrupt(digitalPinToInterrupt(channelLeft), ISRencoderLeft, CHANGE);

	setupModem(); 
  	mqtt.setServer(mqtt_server, mqtt_port); // MQTT Broker setup

	Wire.begin();
	mpu6050.begin();
	mpu6050.calcGyroOffsets(true);
	delay(1000);

    ads1.setGain(GAIN_TWOTHIRDS); // +/- 6.144V  1 bit = 0.1875mV (default)
    ads1.begin(0x48);
    delay(100);

}

//**************************************
//************** MAIN ******************
//**************************************

void loop()
{

  if(!mqtt.connected()){//Check if the board is connected to the server
      reconnect();
  }

	if (mqtt.connected())
	{

		if (millis() - start_time_imu > SAMPLING_PERIOD)
		{
			mpu6050.update();
			accX = mpu6050.getAccX();
			accY = mpu6050.getAccY();
			accZ = mpu6050.getAccZ();
			start_time_imu = millis();
		}

        getSpeed(100);

		if (millis() - start_time > SAMPLING_PERIOD)
		{   

            adc1[0] = ads1.readADC_SingleEnded(0);
            voltageFR=adc1[0]*3.3/26400.0;
            adc1[1] = ads1.readADC_SingleEnded(0);
            voltageFL=adc1[1]*3.3/26400.0;
            //adc1[2] = ads1.readADC_SingleEnded(0);
            //voltageBR=adc1[2]*3.3/26400.0;
            //adc1[3] = ads1.readADC_SingleEnded(0);
            //voltageBR=adc1[3]*3.3/26400.0;

            adc1[2] = ads1.readADC_SingleEnded(0);
            current=(adc1[2]*3.3*100)/26400.0;
            
			String str1 = String(voltageFR);
			str1.toCharArray(msg, 35);
			mqtt.publish("dashboard/voltage", msg);
            Serial.printf("Adc: %d\n",adc1[0]);
            
            Serial.printf("Voltage: %f\n",voltageFR);
            Serial.printf("Current: %f\n",current);
            Serial.printf("rmp Right: %f\n",RPMR);
            Serial.printf("rmp Left: %f\n",RPML);
            Serial.printf("accY: %f\n",accY);

			start_time = millis();
		}
	}
	mqtt.loop();
}

//*****************************
//********  GET SPEED  ********
//*****************************

void getSpeed(int sampling_time){
    if(ISRCounterRight){
        pulseCounterRight++; 
        portENTER_CRITICAL_ISR(&spinlock);
        ISRCounterRight--;
        portEXIT_CRITICAL_ISR(&spinlock);
    }

    if(ISRCounterLeft){
        pulseCounterLeft++; 
        portENTER_CRITICAL_ISR(&spinlock);
        ISRCounterLeft--;
        portEXIT_CRITICAL_ISR(&spinlock);
    }

    if(pulseCounterRight>=PPR){
        currentRevRight++;
        pulseCounterRight=0;
    } 

    if(pulseCounterLeft>=PPR){
        currentRevLeft++;
        pulseCounterRight=0;
    } 

    if (millis() - start_time_speed > sampling_time)
		{   
            start_time_speed=millis();

            RPMR = (currentRevRight - previousRevRight)*float(sampling_time);
            previousRevRight = currentRevRight;

            RPML = (currentRevLeft - previousRevLeft)*float(sampling_time);
            previousRevLeft = currentRevLeft;

        }
}

//*****************************
//****** WIFI CONNECTION ******
//*****************************

void setupModem(){
    delay(10);
    Serial.println("Initializing modem...");
    modem.init();

    Serial.print("Waiting for network...");
    if (!modem.waitForNetwork()) {
        Serial.println("fail");
        while (true);
    }
    Serial.println(" OK");

    Serial.print("Connecting to ");
    Serial.print(apn);
    if (!modem.gprsConnect(apn, user, pass)) {
        Serial.println(" fail");
        while (true);
    }
    Serial.println(" OK");
}

//*****************************
//***    CONEXION MQTT      ***
//*****************************

void reconnect(){
    while(!mqtt.connected()){
        Serial.println("");
        Serial.print("Trying connection MQTT...");

        String clientId = "esp32_";
        clientId += String(random(0xffff),HEX);

        if(mqtt.connect(clientId.c_str(),mqtt_user,mqtt_pass)){//connects to MQTT
            Serial.println("Connected!");

            char topic[25];
            String topic_aux = "dashboard/#";
            topic_aux.toCharArray(topic,25);
            mqtt.subscribe(topic);//subscribe to topic command

        }else{
            Serial.print("fail :( with error -> ");
            Serial.print(mqtt.state());
            Serial.println(" Try again in 5 seconds");

            delay(5000);
        }    
    }
}
