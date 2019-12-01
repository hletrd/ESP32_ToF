//ToF related
#include "SlowSoftI2CMaster.h"
#include "VL53L0X.h"
VL53L0X sensor[4];
int distance[4];
int sensorCount = 4;

//Sleep related
RTC_DATA_ATTR int bootCount = 0;

//BLE related. Just broadcasting collected data via BLE.
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
float txValue = 0;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define charlen 36
char sendvalue[charlen];

class bleServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Connected!");
    //BLEDevice::startAdvertising();
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Disconnected!");
    BLEDevice::startAdvertising();
  }
};

class bleCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.print("Received:");

        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]);
        }
        Serial.println();
      }
    }
};

void setup() {
  Serial.begin(115200);

  Serial.println("Device init");
  delay(3000);

  //Each I2C pins for each ToF sensors

  sensor[0].setI2CPin(33, 32);
  sensor[1].setI2CPin(26, 25);
  sensor[2].setI2CPin(14, 27);
  sensor[3].setI2CPin(2, 15);
  
  for (int i = 0; i < sensorCount; i++) {
    sensor[i].init();
    sensor[i].setTimeout(50);
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.println(" init");
  }
  
  BLEDevice::init("ESP32 ToF");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new bleServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pRxCharacteristic->setCallbacks(new bleCharacteristicCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  
  Serial.println("BLE advertising");
}



void loop() {
  for (int i = 0; i < sensorCount; i++) {
    distance[i] = sensor[i].readRangeSingleMillimeters();
    //delay(300);
    if (distance[i] == 65535) distance[i] = -1; //failed to measure distance
    if (distance[i] >= 8190) distance[i] = 0;
    //distance[i] = random(100, 1000);
  }
  
  //if (deviceConnected) {
    String txString = String("");
    for (int i = 0; i < sensorCount; i++) {
      txString += distance[i];
      txString += ",";
    }

    for (int i = 0; i < charlen; i++) {
      sendvalue[i] = 0;
    }
    
    txString.toCharArray(sendvalue, txString.length());
    pTxCharacteristic->setValue(sendvalue);
    pTxCharacteristic->notify();
    
    Serial.print("Sent:");
    Serial.print(sendvalue);
    Serial.println();
  //}
/*  if (!deviceConnected && oldDeviceConnected) {
    delay(100);
    pServer->startAdvertising();
    Serial.println("BLE advertising");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }*/
  //delay(0);
}
