#ifndef SECURITY_PERSONALIZER

#define SEND_MESSAGE_RETRY_COUNT 10

// Enable and select radio type attached
#define MY_RADIO_RF24
#define MY_BAUD_RATE 9600

// Signing
// #define MY_SIGNING_SOFT

// #define MY_PASSIVE_NODE
#define MY_PARENT_NODE_ID 0
// #define MY_PARENT_NODE_IS_STATIC
#define MY_NODE_ID                     16

#define MY_RF24_CHANNEL                111
#define MY_RF24_DATARATE               RF24_2MBPS
#define MY_RF24_PA_LEVEL               RF24_PA_HIGH

// DEBUG
// #define MY_DISABLED_SERIAL
#define MY_MAIN_DEBUG
#define MY_DEBUG
// #define MY_DEBUG_VERBOSE_RF24
// #define MY_DEBUG_VERBOSE_CORE
// #define MY_DEBUG_VERBOSE_HARDWARE
// #define MY_DEBUG_VERBOSE_TRANSPORT
// #define MY_DEBUG_VERBOSE_TRANSPORT_HAL
// #define MY_DEBUG_VERBOSE_SIGNING

// #define MY_SLEEP_TRANSPORT_RECONNECT_TIMEOUT_MS 1000
#define MY_SLEEP_HANDLER

#include "pins.h"

#define CHILD_TEMP_ID 1
#define CHILD_HUM_ID 2

#define CHILD_PMS_SP1_ID 3
#define CHILD_PMS_SP2_5_ID 4
#define CHILD_PMS_SP10_ID 5
#define CHILD_PMS_AE1_ID 6
#define CHILD_PMS_AE2_5_ID 7
#define CHILD_PMS_AE10_ID 8
#define CHILD_BATTERY_mV_ID 10
#define CHILD_BATTERY_ADC_ID 9

#define CHILD_PRESSURE_ID 11

#define CHILD_PMS_ERROR 255
#define CHILD_SHT_ERROR 254

#define BATTERY_MIN_mV 1800
#define BATTERY_MAX_mV 4500

#include <MySensors.h>

#ifdef SHT31
#include <SHT31.h>
#endif // SHT31
#ifdef BME280
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#endif // BME280

#include <Wire.h>

#include <PMS.h>
#include <SoftwareSerial.h>

#if defined(MY_MAIN_DEBUG)
#define MAIN_DEBUG(x,...) hwDebugPrint(x, ##__VA_ARGS__)
#else
#define MAIN_DEBUG(x,...)
#endif

#define SEALEVELPRESSURE_HPA (1013.25)

#define PMS_MODUS 3

uint8_t loop_counter = 0;
const uint32_t SLEEP_TIME = (uint32_t) 10 * 60 * 1000;  // sleep time between reads (seconds * 1000 milliseconds)
// const uint32_t SLEEP_TIME = (uint32_t) 30 * 1000;  // sleep time between reads (seconds * 1000 milliseconds)

/*******************************************************************
 * NodeId/1/1/0/0 = TEMP
 * NodeId/2/1/0/1 = HUM
 *
 * NodeId/3/0/0/23 = ?
 * NodeId/3/0/0/48 = PMS SP 1
 *
 * NodeId/4/0/0/23 = ?
 * NodeId/4/0/0/48 = PMS SP 2.5
 *
 * NodeId/5/0/0/23 = ?
 * NodeId/5/0/0/48 = PMS SP 10
 *
 * NodeId/6/0/0/23 = ?
 * NodeId/6/0/0/48 = PMS AE 1
 *
 * NodeId/7/0/0/23 = ?
 * NodeId/7/0/0/48 = PMS AE 2.5
 *
 * NodeId/8/0/0/23 = ?
 * NodeId/8/0/0/48 = PMS AE 10
 *
 * NodeId/255/0/0/17 = MySensors Version
 * NodeId/255/3/0/0 = Battery
 * NodeId/255/3/0/6 = ?
 * NodeId/255/3/0/11 = Device name
 * NodeId/255/3/0/12 = Device version
 * NodeId/255/3/0/32 = ?
*******************************************************************/

MyMessage msgTemperature(CHILD_TEMP_ID, V_TEMP);
MyMessage msgHumidity(CHILD_HUM_ID, V_HUM);

MyMessage msgPmsSp1(CHILD_PMS_SP1_ID, V_CUSTOM);
MyMessage msgPmsSp2_5(CHILD_PMS_SP2_5_ID, V_CUSTOM);
MyMessage msgPmsSp10(CHILD_PMS_SP10_ID, V_CUSTOM);
MyMessage msgPmsAe1(CHILD_PMS_AE1_ID, V_CUSTOM);
MyMessage msgPmsAe2_5(CHILD_PMS_AE2_5_ID, V_CUSTOM);
MyMessage msgPmsAe10(CHILD_PMS_AE10_ID, V_CUSTOM);
MyMessage msgBattery_mV(CHILD_BATTERY_mV_ID, V_CUSTOM);
MyMessage msgBatteryAdc(CHILD_BATTERY_ADC_ID, V_CUSTOM);
MyMessage msgPressure(CHILD_PRESSURE_ID, V_PRESSURE);

MyMessage msgPmsError(CHILD_PMS_ERROR, V_CUSTOM);
MyMessage msgShtError(CHILD_SHT_ERROR, V_CUSTOM);

#ifdef SHT31
SHT31 sht;
#endif //SHT31
#ifdef BME280
Adafruit_BME280 bme; // I2C
#endif // BME280
bool shtSuccessful = false;

static const uint32_t PMS_READ_DELAY = 30000;
SoftwareSerial pmsSerial(MY_PMS_TX_PIN, MY_PMS_RX_PIN);
PMS pms(pmsSerial);
PMS::DATA pmsData;
bool pmsSuccessful = false;

uint16_t battery_mV;
uint8_t batteryPcnt;

void initializeNrf();
void enableBoostConverter();
void disableBoostConverterIfEnoughPower();
void readSensors();
void readBatteryLevel();
void readTemperatureHumidityPressure();
void sendData();
void retryingSend(MyMessage &message);
void waitingPresent(const uint8_t childSensorId, const mysensors_sensor_t sensorType, const char *description);

void setup() {
   pinMode(PIN_PC2, INPUT_PULLUP);
   pinMode(PIN_PD2, INPUT_PULLUP);
   pinMode(PIN_PD3, INPUT_PULLUP);
   pinMode(PIN_PC1, INPUT_PULLUP);

   pinMode(MY_SENSORS_POWER_PIN, OUTPUT);
   #ifdef MY_MAIN_DEBUG
   Serial.begin(MY_BAUD_RATE);
   #endif

   pinMode(MY_PMS_TX_PIN, INPUT);
   pinMode(MY_PMS_RX_PIN, OUTPUT);

   pinMode(LED_PIN, OUTPUT);
   pinMode(BAT_REF_PIN, INPUT);
   pinMode(CONVERTER_EN_PIN, OUTPUT);

   // pinMode(ON_INDICATOR, OUTPUT);
}

void presentation() {
   // Send the sketch version information to the gateway and Controller
	sendSketchInfo("Weather Station", "0.95");

	// Register all sensors to gw (they will be created as child devices)
#if defined(SHT31) || defined(BME280)
	waitingPresent(CHILD_TEMP_ID, S_TEMP, "Temperature °C");
	waitingPresent(CHILD_HUM_ID, S_HUM, "Humidity %");
#endif
#ifdef BME280
	waitingPresent(CHILD_PRESSURE_ID, S_BARO, "Pressure hPa");
#endif

	waitingPresent(CHILD_PMS_SP1_ID, S_CUSTOM, "SP 1.0 ug/m^3");
	waitingPresent(CHILD_PMS_SP2_5_ID, S_CUSTOM, "SP 2.5, ug/m^3");
	waitingPresent(CHILD_PMS_SP10_ID, S_CUSTOM, "SP 10 ug/m^3");
	waitingPresent(CHILD_PMS_AE1_ID, S_CUSTOM, "AE 1.0 ug/m^3");
	waitingPresent(CHILD_PMS_AE2_5_ID, S_CUSTOM, "AE 2.5 ug/m^3");
	waitingPresent(CHILD_PMS_AE10_ID, S_CUSTOM, "AE 10 ug/m^3");
	waitingPresent(CHILD_BATTERY_mV_ID, S_CUSTOM, "Battery mV");

	waitingPresent(CHILD_PMS_ERROR, S_CUSTOM, "Particle sensor error");
	waitingPresent(CHILD_SHT_ERROR, S_CUSTOM, "Temperature sensor error");
}

void waitingPresent(const uint8_t childSensorId, const mysensors_sensor_t sensorType, const char *description) {
   present(childSensorId, sensorType, description);
   delay(100);
}

void loop() {
   //digitalWrite(ON_INDICATOR, HIGH);

   enableBoostConverter();
   readSensors();

   sendData();

   disableBoostConverterIfEnoughPower();

   // digitalWrite(ON_INDICATOR, LOW);

	sleep(SLEEP_TIME);
   loop_counter++;
}

void sleepHandler(bool sleep) {
	if (sleep) {
      hwSPI.end();

      digitalWrite(MY_RF24_CE_PIN, LOW);
      digitalWrite(MY_RF24_CS_PIN, LOW);
      digitalWrite(MY_SCK, LOW);
      pinMode(MY_MO, OUTPUT);
      digitalWrite(MY_MO, LOW);
   } else {
      hwSPI.begin();
   }
}

void initializeNrf() {
   MAIN_DEBUG(PSTR("MAIN:INIT:RADIO\n"));
   // Initialise transport layer
   transportInitialise();

   // Register transport=ready callback
   transportRegisterReadyCallback(_callbackTransportReady);

   // wait until transport is ready
   (void)transportWaitUntilReady(MY_TRANSPORT_WAIT_READY_MS);
}

void sendData() {
   digitalWrite(LED_PIN, HIGH);
   MAIN_DEBUG(PSTR("MAIN:LED:ON\n"));

   initializeNrf();

   sendBatteryLevel(batteryPcnt);

   retryingSend(msgBattery_mV);
   retryingSend(msgBatteryAdc);

   if (shtSuccessful) {
      retryingSend(msgTemperature);
      retryingSend(msgHumidity);
      retryingSend(msgPressure);
   } else {
      retryingSend(msgShtError);
   }

   if (pmsSuccessful) {
      retryingSend(msgPmsSp1);
      retryingSend(msgPmsSp2_5);
      retryingSend(msgPmsSp10);
      retryingSend(msgPmsAe1);
      retryingSend(msgPmsAe2_5);
      retryingSend(msgPmsAe10);
   } else {
      retryingSend(msgPmsError);
   }

   digitalWrite(LED_PIN, LOW);
   MAIN_DEBUG(PSTR("MAIN:LED:OFF\n"));
}

void retryingSend(MyMessage &message) {
   uint8_t retry = 0;
   while (!send(message) && retry++ <= SEND_MESSAGE_RETRY_COUNT) {
      delay(100);
   }
   delay(100);
}

void enableBoostConverter() {
   digitalWrite(CONVERTER_EN_PIN, HIGH);
   MAIN_DEBUG(PSTR("MAIN:CONV:ON\n"));
}

void disableBoostConverterIfEnoughPower() {
   if (battery_mV > 2800) {
      digitalWrite(CONVERTER_EN_PIN, LOW);
      MAIN_DEBUG(PSTR("MAIN:CONV:OFF\n"));
   } else {
      MAIN_DEBUG(PSTR("!MAIN:CONV:OFF\n"));
   }
}

void readBatteryLevel() {
   analogReference(INTERNAL);
   delay(500);

   uint16_t batteryRef = analogRead(BAT_REF_PIN);
   for (uint8_t i = 0; i < 10; i++) {
      batteryRef = analogRead(BAT_REF_PIN);
   }
   msgBatteryAdc.set(batteryRef);

   battery_mV = ((uint64_t)batteryRef * 1100 * 127) / (1023 * 27);
   msgBattery_mV.set(battery_mV);

   batteryPcnt = ((uint64_t)(battery_mV - BATTERY_MIN_mV) * 100) / (BATTERY_MAX_mV - BATTERY_MIN_mV);
   MAIN_DEBUG(PSTR("MAIN:VOLT:%" PRIi16 "\n"), battery_mV);
}

void readSensors() {
   MAIN_DEBUG(PSTR("MAIN:SENS:ON\n"));
   digitalWrite(MY_SENSORS_POWER_PIN, HIGH);

   bool shouldReadPms = (loop_counter % PMS_MODUS) == 0;

   if (shouldReadPms) {
      pmsSerial.begin(PMS::BAUD_RATE);
      pms.passiveMode();
      pms.wakeUp();
   }

   uint32_t start = millis();

   readBatteryLevel();

   readTemperatureHumidityPressure();

   if (shouldReadPms) {
      int32_t diff = PMS_READ_DELAY - (millis() - start);
      if (diff > 0) {
         MAIN_DEBUG(PSTR("MAIN:WAIT:%" PRIi32 "ms\n"), diff);

         pmsSerial.end();
         sleep(diff);
         pmsSerial.begin(PMS::BAUD_RATE);
      }

      pms.requestRead();
      if (pms.readUntil(pmsData)) {
         msgPmsSp1.set(pmsData.PM_SP_UG_1_0);
         msgPmsSp2_5.set(pmsData.PM_SP_UG_2_5);
         msgPmsSp10.set(pmsData.PM_SP_UG_10_0);
         msgPmsAe1.set(pmsData.PM_AE_UG_1_0);
         msgPmsAe2_5.set(pmsData.PM_AE_UG_2_5);
         msgPmsAe10.set(pmsData.PM_AE_UG_10_0);

         MAIN_DEBUG(PSTR("MAIN:PMS:\n   SP_1:%" PRIu16 "\n   SP_2_5:%" PRIu16 "\n   SP_10:%" PRIu16 "\n   AE_1:%" PRIu16 "\n   AE_2_5:%" PRIu16 "\n   AE_10:%" PRIu16 "\n"), pmsData.PM_SP_UG_1_0, pmsData.PM_SP_UG_2_5, pmsData.PM_SP_UG_10_0, pmsData.PM_AE_UG_1_0, pmsData.PM_AE_UG_2_5, pmsData.PM_AE_UG_10_0);
         pmsSuccessful = true;
      } else {
         msgPmsError.set(true);
         MAIN_DEBUG(PSTR("!MAIN:PMS:ERR\n"));
         pmsSuccessful = false;
      }

      pms.sleep();
      pmsSerial.end();
   }

   MAIN_DEBUG(PSTR("MAIN:SENS:OFF\n"));
   digitalWrite(MY_SENSORS_POWER_PIN, LOW);
}

void readTemperatureHumidityPressure() {
   Wire.begin();
   Wire.setClock(10000UL);
#ifdef SHT31
   sht.begin(SHT_DEFAULT_ADDRESS);

   if (sht.read()) {
	   msgTemperature.set(sht.getTemperature(), 2);
	   msgHumidity.set(sht.getHumidity(), 2);
      msgPressure.set(SEALEVELPRESSURE_HPA, 2);

      MAIN_DEBUG(PSTR("MAIN:TEMP:%" PRIi16 ".\n"), (int16_t)sht.getTemperature());
      MAIN_DEBUG(PSTR("MAIN:HUM:%" PRIi16 "\n"), (int16_t)sht.getHumidity());
      shtSuccessful = true;
   } else {
      MAIN_DEBUG(PSTR("!MAIN:TEMP:ERR:%" PRIi16 "\n"), sht.getError());
      msgShtError.set(sht.getError());
      shtSuccessful = false;
   }
#endif // SHT31
#ifdef BME280
   unsigned status;
   // status = bme.begin();
   // You can also pass in a Wire library object like &Wire2
   status = bme.begin(0x76, &Wire);
   bme.init();
   if (status) {
      msgTemperature.set(bme.readTemperature(), 2);
      msgHumidity.set(bme.readHumidity(), 2);
      msgPressure.set(bme.readPressure() / 100.0F, 2);

      MAIN_DEBUG(PSTR("MAIN:BME:%" PRIu32 ".\n"), bme.sensorID());
      MAIN_DEBUG(PSTR("MAIN:TEMP:%" PRIi16 ".\n"), (int16_t)bme.readTemperature());
      MAIN_DEBUG(PSTR("MAIN:HUM:%" PRIi16 "\n"), (int16_t)bme.readHumidity());
      MAIN_DEBUG(PSTR("MAIN:PRES:%" PRIi16 "\n"), (int16_t)bme.readPressure());
      shtSuccessful = true;
   } else {
      MAIN_DEBUG(PSTR("MAIN:BME:%" PRIu32 ".\n"), bme.sensorID());
      MAIN_DEBUG(PSTR("!MAIN:BME.\n"));
   }

#endif // BME280
   Wire.end();
}

#endif