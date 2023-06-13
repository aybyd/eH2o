#include <Arduino.h>
#include <SimpleTimer.h>
#include "DS18S20.h"
#include "DFRobot_EC10.h"
#include <DFRobot_PH.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

#define DS18S20_PIN 28
#define EC10_PIN 10
#define PH_PIN 8
#define DO_PIN 9
#define WATER_INTAKE_PIN 22
#define WATER_OUT_PIN 23
#define PUMP_DURATION 15000
#define VREF 5000     //VREF (mv)
#define ADC_RES 1024  //ADC Resolution
//Amonia Sensor
#define RL 47         //The value of resistor RL is 47K
#define m -0.263      //Enter calculated Slope
#define b 0.42        //Enter calculated intercept
#define Ro 20         //Enter found Ro value
#define MQ_sensor A7  //BH3 Sensor is connected to A7

//Single-point calibration Mode=0
//Two-point calibration Mode=1
#define TWO_POINT_CALIBRATION 0
//Single point calibration needs to be filled CAL1_V and CAL1_T
#define CAL1_V (483)    //mv
#define CAL1_T (26.19)  //℃
//Two-point calibration needs to be filled CAL2_V and CAL2_T
//CAL1 High temperature point, CAL2 Low temperature point
#define CAL2_V (1300)  //mv
#define CAL2_T (15)    //℃
SoftwareSerial sw_master(21, 20);
SimpleTimer taskTimer(1800000);  // 30 minutes
SimpleTimer taskTimer1(600000);     // 10 minute/s

DFRobot_PH ph;
DFRobot_EC10 ec;
DS18S20 temp;

float ec_voltage, ph_voltage, phValue, ecValue, curr_temp = 25, VRL, Rs, ratio;
String hw_id = "98CDAC25ACA2";

const uint16_t DO_Table[41] = {
  14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
  11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
  9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
  7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410
};

uint8_t Temperature;
uint16_t ADC_Raw;
uint16_t ADC_Voltage;
uint16_t DO;

bool FLAG_FORCE_READING = false;
bool FLAG_GET_READINGS = false;
bool FLAG_WATER_INTAKE = false;

struct Data {
  float temperature;
  float pH;
  float eC;
  float DO;
  float NH3;
} data;

void setup() {
  Serial.begin(9600);
  sw_master.begin(9600);
  Serial.println("Sensors initializing......");
  delay(3000);
  pinMode(WATER_INTAKE_PIN, OUTPUT);
  pinMode(WATER_OUT_PIN, OUTPUT);
  delay(300);
  digitalWrite(WATER_INTAKE_PIN, HIGH);
  digitalWrite(WATER_OUT_PIN, HIGH);
  //activatePump();

}



void loop() {
  if (taskTimer1.isReady()) {  // Check if ready a  timer
    Serial.println("Task called here!");
    activatePump();
    taskTimer1.reset();  // Reset a  timer
  }
}

void getTemperature() {
  temp.read(DS18S20_PIN);
  char c_temp[7];
  curr_temp = temp.temperature;
}


void getData(Data* data) {
  digitalWrite(WATER_INTAKE_PIN, HIGH);
  unsigned long startTime = millis();
  while (millis() - startTime < 60000) {  //time interval: 1s
    getTemperature();
    ph_voltage = analogRead(PH_PIN) / 1024.0 * 5000;  // read the ph voltage
    phValue = ph.readPH(ph_voltage, curr_temp);       // convert voltage to pH with temperature compensation
                                                      //Serial.print("pH:");
                                                      //Serial.print(phValue, 2);
    ADC_Raw = analogRead(DO_PIN);
    ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;
    //Serial.println("DO:\t" + String(readDO(ADC_Voltage, curr_temp) / 1000) + "\t");

    ec_voltage = analogRead(EC10_PIN) / 1024.0 * 5000;
    ecValue = ec.readEC(ec_voltage, curr_temp);  // convert voltage to EC with temperature compensation
    //Serial.print(", EC:");
    //Serial.print(ecValue, 2);
    //Serial.println("ms/cm");

    VRL = analogRead(MQ_sensor) * (5.0 / 1023.0);   //Measure the voltage drop and convert to 0-5V
    Rs = ((5.0 * RL) / VRL) - RL;                   //Use formula to get Rs value
    ratio = Rs / Ro;                                // find ratio Rs/Ro
    float ppm = pow(10, ((log10(ratio) - b) / m));  //use formula to calculate ppm

    data->temperature = curr_temp;
    data->pH = phValue;
    data->eC = ecValue;
    data->DO = readDO(ADC_Voltage, curr_temp) / 1000;
    data->NH3 = ppm;
    //printData(data);
  }
  Serial.println("This is final reading!");
  printData(data);
}

void printData(Data* data) {

  StaticJsonDocument<200> json_data;
  //JsonObject& json_data = jsonBuffer.createObject();
  json_data["log_temp"] = data->temperature;
  json_data["log_ph"] = data->pH;
  json_data["log_ec"] = data->eC;
  json_data["log_do"] = data->DO;
  json_data["log_amonia"] = data->NH3;
  json_data["hw_id"] = hw_id;  
  //Serial.println();
  serializeJson(json_data, Serial);
  serializeJson(json_data, sw_master);
  sw_master.print("\n");
  json_data.clear();
  dumpWater();
}

int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c) {
#if TWO_POINT_CALIBRATION == 00
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}

void activatePump() {
  digitalWrite(WATER_INTAKE_PIN, LOW);  
  delay(PUMP_DURATION);
  getData(&data);  
}

void dumpWater() {
  digitalWrite(WATER_OUT_PIN, LOW);
  delay(7000);
  digitalWrite(WATER_OUT_PIN, HIGH);
}