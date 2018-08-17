// Simple test of USB Host printer
//
// This example is in the public domain

#include "USBHost_t36.h"
#include "USBPrinter_t36.h"

USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
USBHIDParser hid1(myusb);
USBHIDParser hid2(myusb);
USBHIDParser hid3(myusb);
USBPrinter uprinter(myusb);

USBDriver *drivers[] = {&hub1, &hub2, &hid1, &hid2, &hid3, &uprinter};
#define CNT_DEVICES (sizeof(drivers)/sizeof(drivers[0]))
const char * driver_names[CNT_DEVICES] = {"Hub1", "Hub2",  "HID1", "HID2", "HID3", "RECEIPT1" };
bool driver_active[CNT_DEVICES] = {false, false, false, false, false, false};

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(2, OUTPUT);
  for (int i = 0; i < 5; i++) {
    digitalWrite(2, HIGH);
    delayMicroseconds(50);
    digitalWrite(2, LOW);
    delayMicroseconds(50);
  }
  while (!Serial && (millis() < 5000)) ; // wait for Arduino Serial Monitor
  Serial.println("\n\nUSB Host Testing - Printer");
  myusb.begin();
  Serial1.begin(115200);  // We will echo stuff Through Serial1...
}

void loop()
{
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  myusb.Task();
  // Print out information about different devices.
  for (uint8_t i = 0; i < CNT_DEVICES; i++) {
    if (*drivers[i] != driver_active[i]) {
      if (driver_active[i]) {
        Serial.printf("*** Device %s - disconnected ***\n", driver_names[i]);
        driver_active[i] = false;
      } else {
        Serial.printf("*** Device %s %x:%x - connected ***\n", driver_names[i], drivers[i]->idVendor(), drivers[i]->idProduct());
        driver_active[i] = true;

        const uint8_t *psz = drivers[i]->manufacturer();
        if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
        psz = drivers[i]->product();
        if (psz && *psz) Serial.printf("  product: %s\n", psz);
        psz = drivers[i]->serialNumber();
        if (psz && *psz) Serial.printf("  Serial: %s\n", psz);

        // If this is a new Printer device.
        if (drivers[i] == &uprinter) {
          // Lets try first outputting something to our UPrinter to see if it will go out...
          uprinter.begin();
        }
      }
    }
  }

  if (Serial.available()) {
//    Serial.println("Serial Available");
    while (Serial.available()) {
      uprinter.write(Serial.read());
    }
  }

  while (Serial1.available()) {
//    Serial.println("Serial1 Available");
    Serial1.write(Serial1.read());
  }

  while (uprinter.available()) {
//    Serial.println("uprinter Available");
    Serial.write(uprinter.read());
  }
}
