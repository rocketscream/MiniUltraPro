/*******************************************************************************
   Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   This example sends a valid LoRaWAN packet with payload "Hello,
   world!", using frequency and encryption settings matching those of
   the The Things Network.

   This uses OTAA (Over-the-air activation), where where a DevEUI and
   application key is configured, which are used in an over-the-air
   activation procedure where a DevAddr and session keys are
   assigned/generated for use with all further communication.

   Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
   g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
   violated by this sketch when left running for longer)!

   To use this sketch, first register your application and device with
   the things network, to set or generate an AppEUI, DevEUI and AppKey.
   Multiple devices can use the same AppEUI, but each device has its own
   DevEUI and AppKey.

   Do not forget to define the radio type correctly in config.h.

 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <RTCZero.h>
#include <SerialFlash.h>
#define EUI64_CHIP_ADDRESS 0x50
#define EUI64_MAC_ADDRESS 0xF8
#define EUI64_MAC_LENGTH 0x08
#define MAX_DATA_SIZE 2
#define Serial SerialUSB

RTCZero rtc;

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
u1_t DEVEUI[EUI64_MAC_LENGTH];
void os_getDevEui (u1_t* buf) {
  memcpy(buf, DEVEUI, EUI64_MAC_LENGTH);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

static uint8_t data[MAX_DATA_SIZE];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 5,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 3,
  .dio = {2, 6, LMIC_UNUSED_PIN},
};

void onEvent (ev_t ev)
{
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));

      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Ensure all debugging messages are sent before sleep
      Serial.flush();

      // Sleep for a period of TX_INTERVAL using single shot alarm
      rtc.setAlarmEpoch(rtc.getEpoch() + TX_INTERVAL);
      rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
      rtc.attachInterrupt(alarmMatch);
      // USB port consumes extra current
      USBDevice.detach();
      // Enter sleep mode
      rtc.standbyMode();
      // Reinitialize USB for debugging
      USBDevice.init();
      USBDevice.attach();

      // Schedule next transmission to be immediately after this
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(1), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void do_send(osjob_t* j)
{
  unsigned char counter;
  float batteryVoltage;
  int adcReading;
  int voltage;

  digitalWrite(LED_BUILTIN, HIGH);

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  {
    // ***** Battery monitor connection
    //
    // VBAT-----1M-----3M3-----GND
    //              | 
    //              ---0.1uF---GND
    //              |
    //              A5   

    adcReading = analogRead(A5);
    // Discard inaccurate 1st reading
    adcReading = 0;
    // Perform averaging
    for (counter = 10; counter > 0; counter--)
    {
      adcReading += analogRead(A5);
    }
    adcReading = adcReading / 10;
    // Convert to volts
    batteryVoltage = adcReading * (4.3 / 1023.0);

    Serial.print(F("Battery: "));
    Serial.print(batteryVoltage);
    Serial.println(F(" V"));

    // Pack float into int with 2 decimal point resolution
    voltage = batteryVoltage * 100;
    data[0] = voltage >> 8;
    data[1] = voltage;
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, data, sizeof(data), 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.

  digitalWrite(LED_BUILTIN, LOW);
}

void setDevEui(unsigned char* buf)
{
  Wire.begin();
  Wire.beginTransmission(EUI64_CHIP_ADDRESS);
  Wire.write(EUI64_MAC_ADDRESS);
  Wire.endTransmission();
  Wire.requestFrom(EUI64_CHIP_ADDRESS, EUI64_MAC_LENGTH);

  // Format needs to be little endian (LSB...MSB)
  while (Wire.available())
  {
    *buf-- = Wire.read();
  }
}

void setup()
{
  int count;
  unsigned char pinNumber;

  // ***** Put unused pins into known state *****
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);

  // D7-D13, A0(D14)-A5(D19), SDA(D20), SCL(D21), MISO(D22)
  for (pinNumber = 7; pinNumber <= 22; pinNumber++)
  {
    pinMode(pinNumber, INPUT_PULLUP);
  }
  // RX_LED (D25) & TX_LED (D26) (both LED not mounted on Mini Ultra Pro)
  pinMode(25, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);
  // D30 (RX) & D31 (TX) of Serial
  pinMode(30, INPUT_PULLUP);
  pinMode(31, INPUT_PULLUP);
  // D34-D38 (EBDG Interface)
  for (pinNumber = 34; pinNumber <= 38; pinNumber++)
  {
    pinMode(pinNumber, INPUT_PULLUP);
  }
  // ***** End of unused pins state initialization *****

  pinMode(LED_BUILTIN, OUTPUT);

  setDevEui(&DEVEUI[EUI64_MAC_LENGTH - 1]);
  while (!Serial && millis() < 10000);
  Serial.begin(115200);
  Serial.println(F("Starting"));
  Serial.print(F("DEVEUI: "));

  for (count = EUI64_MAC_LENGTH; count > 0; count--)
  {
    Serial.print("0x");
    if (DEVEUI[count - 1] <= 0x0F) Serial.print("0");
    Serial.print(DEVEUI[count - 1], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Initialize serial flash
  SerialFlash.begin(4);
  // Put serial flash in sleep
  SerialFlash.sleep();

  // Initialize RTC
  rtc.begin();
  // Use RTC as a second timer instead of calendar
  rtc.setEpoch(0);

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}

void alarmMatch()
{

}
