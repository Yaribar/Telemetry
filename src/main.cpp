#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_RX_BUFFER 512
#define RXD2 16 //Hardware Serial
#define TXD2 17 //Hardware Serial
#define SAMPLING_PERIOD 100

#include <Arduino.h>
#include <TinyGsmClient.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <Kalman_Filter.h>
#include <BasicLinearAlgebra.h>
#include "MPU6050_tockn.h"
#include <Adafruit_ADS1X15.h>
#include <SPI.h>

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
//void callback(char *topic, byte *payload, unsigned int length);

MPU6050 mpu6050(Wire);
float accX, accY, accZ;

ulong start_time_imu;
ulong start_time;
ulong start_time_lap;

//****************************************
//**************** ADS *******************
//****************************************

Adafruit_ADS1115 ads;
//const float multiplier = 0.1875F;
int16_t adc0;
float voltage=0.0;

void setup()
{
	
	Serial.begin(115200);
  	Serial2.begin(115200, SERIAL_8N1, RXD2,TXD2); //Baud rate

	setupModem(); 
  	mqtt.setServer(mqtt_server, mqtt_port); // MQTT Broker setup

	Wire.begin();
	mpu6050.begin();
	mpu6050.calcGyroOffsets(true);
	delay(1000);

    ads.setGain(GAIN_TWO);  //    +/- 4.096V  1 bit = 0.125mV
    ads.begin(0x49);
    delay(100);

}

//******************************************************************************************************************
//*********** MAIN *************************************************************************************************
//******************************************************************************************************************

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

		if (millis() - start_time > SAMPLING_PERIOD)
		{
            adc0 = ads.readADC_SingleEnded(0);
            voltage=adc0*3.3/52800.0;
            
			String str1 = String(voltage);
			str1.toCharArray(msg, 35);
			mqtt.publish("dashboard/voltage", msg);
            Serial.printf("Adc: %d\n",adc0);
            Serial.printf("Voltage: %f\n",voltage);
			start_time = millis();
		}
	}
	mqtt.loop();
}

//*****************************
//******  CONEXION WIFI  ******
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
