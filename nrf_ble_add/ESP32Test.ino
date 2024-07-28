#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAddress.h>

#include "irk.h"

#define IRK_LIST_NUMBER 2
char * IrkListName[IRK_LIST_NUMBER] = {"A", "B"};
uint8_t irk[IRK_LIST_NUMBER][ESP_BT_OCTET16_LEN]= 
{
    {0x92,0xE1,0x70,0x7B,0x84,0xDC,0x21,0x4D,0xA6,0x33,0xDC,0x3A,0x3A,0xB2,0x08,0x3F},
    {0x2E,0xB7,0xB3,0xD4,0xDC,0x5C,0x16,0x73,0xA7,0x9B,0x75,0x0E,0xEC,0xEB,0x60,0x2D}
};

#define LED_PIN 2
#define SCAN_TIME 5 // Thời gian quét BLE (giây)

BLEScan* pBLEScan;

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 BLE Scanner");
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW); // Đảm bảo đèn tắt ban đầu

    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
    pBLEScan->setActiveScan(true);
}

void loop() {
    BLEScanResults foundDevices = pBLEScan->start(SCAN_TIME, false);
    int count = foundDevices.getCount();
    bool deviceFound = false;

    for (int i = 0; i < count; i++) {
        BLEAdvertisedDevice advertisedDevice = foundDevices.getDevice(i);
        std::string strAddress = advertisedDevice.getAddress().toString();
        
        // Kiểm tra nếu địa chỉ MAC là địa chỉ ngẫu nhiên
        if (advertisedDevice.getAddressType() == BLE_ADDR_TYPE_RANDOM) {
            uint8_t AdMac[6];
            sscanf(strAddress.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &AdMac[5], &AdMac[4], &AdMac[3], &AdMac[2], &AdMac[1], &AdMac[0]);
            
            for (byte j = 0; j < IRK_LIST_NUMBER; j++) {
                if (btm_ble_addr_resolvable(AdMac, irk[j])) {
                    Serial.printf("MacAdd= %s Belongs to: %s\n", strAddress.c_str(), IrkListName[j]);
                    deviceFound = true;
                    break;
                }
            }
        }
    }

    if (deviceFound) {
        digitalWrite(LED_PIN, HIGH); // Bật đèn khi phát hiện thiết bị
    } else {
        digitalWrite(LED_PIN, LOW); // Tắt đèn nếu không phát hiện thiết bị
    }

    delay(1000); // Chờ trước khi quét lại
}
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAddress.h>

#include "irk.h"

#define IRK_LIST_NUMBER 2
char * IrkListName[IRK_LIST_NUMBER] = {"A", "B"};
uint8_t irk[IRK_LIST_NUMBER][ESP_BT_OCTET16_LEN]= 
{
    {0x92,0xE1,0x70,0x7B,0x84,0xDC,0x21,0x4D,0xA6,0x33,0xDC,0x3A,0x3A,0xB2,0x08,0x3F},
    {0x2E,0xB7,0xB3,0xD4,0xDC,0x5C,0x16,0x73,0xA7,0x9B,0x75,0x0E,0xEC,0xEB,0x60,0x2D}
};

#define LED_PIN 2
#define SCAN_TIME 5 // Thời gian quét BLE (giây)

BLEScan* pBLEScan;

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 BLE Scanner");
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW); // Đảm bảo đèn tắt ban đầu

    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
    pBLEScan->setActiveScan(true);
}

void loop() {
    BLEScanResults foundDevices = pBLEScan->start(SCAN_TIME, false);
    int count = foundDevices.getCount();
    bool deviceFound = false;

    for (int i = 0; i < count; i++) {
        BLEAdvertisedDevice advertisedDevice = foundDevices.getDevice(i);
        std::string strAddress = advertisedDevice.getAddress().toString();
        
        uint8_t AdMac[6];
        sscanf(strAddress.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &AdMac[5], &AdMac[4], &AdMac[3], &AdMac[2], &AdMac[1], &AdMac[0]);
        
        // Kiểm tra xem địa chỉ có phải là địa chỉ ngẫu nhiên không
        if ((AdMac[0] & 0xC0) == 0x40) {
            for (byte j = 0; j < IRK_LIST_NUMBER; j++) {
                if (btm_ble_addr_resolvable(AdMac, irk[j])) {
                    Serial.printf("MacAdd= %s Belongs to: %s\n", strAddress.c_str(), IrkListName[j]);
                    deviceFound = true;
                    break;
                }
            }
        }
    }

    if (deviceFound) {
        digitalWrite(LED_PIN, HIGH); // Bật đèn khi phát hiện thiết bị
    } else {
        digitalWrite(LED_PIN, LOW); // Tắt đèn nếu không phát hiện thiết bị
    }

    delay(1000); // Chờ trước khi quét lại
}
