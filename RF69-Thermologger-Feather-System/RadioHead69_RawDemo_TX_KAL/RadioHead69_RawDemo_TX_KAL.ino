// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem 
// configuration

/*Joseph de la Viesca
 * Comments for working with this code going forward:
 * Currently, this code is able to send data in the form of a character array
 * to the RX node, which is sent in the form of a character array. The send function
 * requires a packet parameter of type (unsigned char*), so at the moment data can
 * only be sent in the form of a character array. The code to implement the thermouple
 * data reading and storing into the radio packet has also not been implemented yet.
 * When done so, the read temperature from the thermocouple should be stored in the
 * data.temp variable so that it can be formatted into the character array and sent.
 * 
 * The data structure consists of three components:
 *   uint16_t nodeID: stores the ID of the message
 *   unsigned long uptime: stores the uptime of the system at the moment it is sent
 *   float temp: stores the read temperature
 * 
 * NOTE: When formatting data into a string using sprintf, make sure you are printing
 * into a character array with memory allocated and NOT just a character array pointer.
 * 
 * NECESSARY PIN CONNECTIONS FOR THERMOCOUPLE:
 * 3V on Feather -> Vin on MAX31855
 * GND on Feather -> GND on MAX31855
 * SDA on Feather -> CLK on MAX31855
 * Digital I/O Pin 12 on Feather -> CS on MAX31855
 * SCL on Feather -> DO on MAX31855
 * 
 */

#include <SPI.h>
#include <RH_RF69.h>
#include <string.h>
#include <stdio.h>
//#include <"Adafruit_MAX31855.h">

/***********MAX31855 Thermocouple Setup***********/
//Define your data out, clock, and chip select pins
//#define DO 20 
//#define CLK 21
//#define CS 12

//const int chipSelect = 12;
//File root;
//String fName;
//bool error = false;

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

/*
#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     7
  #define RFM69_RST     4
  #define LED           13
#endif
*/
#if defined(ADAFRUIT_FEATHER_M0) // Feather M0 w/Radio
  #define RFM69_CS      10  //8
  #define RFM69_INT     6   //3
  #define RFM69_RST     11  //4
  #define LED           13
#endif
/*
#if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM69_INT     3  // 
  #define RFM69_CS      4  //
  #define RFM69_RST     2  // "A"
  #define LED           13
#endif

#if defined(ESP8266)    // ESP8266 feather w/wing
  #define RFM69_CS      2    // "E"
  #define RFM69_IRQ     15   // "B"
  #define RFM69_RST     16   // "D"
  #define LED           0
#endif

#if defined(ESP32)    // ESP32 feather w/wing
  #define RFM69_RST     13   // same as LED
  #define RFM69_CS      33   // "B"
  #define RFM69_INT     27   // "A"
  #define LED           13
#endif
*/

//The following was entered by KAL per instruction on https://learn.adafruit.com/radio-featherwing/wiring for the M0 feathers
//#define RFM69_CS      10   // "B"
//#define RFM69_RST     11   // "A"
//#define RFM69_IRQ     6    // "D"
//#define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )


/* Teensy 3.x w/wing
#define RFM69_RST     9   // "A"
#define RFM69_CS      10   // "B"
#define RFM69_IRQ     4    // "C"
#define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )
*/
 
/* WICED Feather w/wing 
#define RFM69_RST     PA4     // "A"
#define RFM69_CS      PB4     // "B"
#define RFM69_IRQ     PA15    // "C"
#define RFM69_IRQN    RFM69_IRQ
*/

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission

//Data structure to store 
typedef struct RFData{
    uint16_t nodeID;
    unsigned long uptime;
    float temp;
};

RFData data; //Declared only once outside loop

void setup() 
{
  /***MAX31855 Thermocouple Setup***/
  
  /***RF69 Feather Wing Setup***/
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer
  Serial.println("This is a test....");  //KAL added

  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 TX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}



void loop() {
  delay(1000);  // Wait 1 second between transmits, could also 'sleep' here!

  //KAL added
  Serial.println ("Still a test...");
  Serial.print("RST = ");
  Serial.println(RFM69_RST);
  Serial.print("CS = ");
  Serial.println(RFM69_CS);
  Serial.print("INT = ");
  Serial.println(RFM69_INT);

  //Until Thermocouple works, use these placeholders
  data.nodeID = 1;
  data.uptime = millis();
  data.temp = 91.20;

  //Declaration of character array to be sent
  char radiopacket[20];

  //Printing the data into the radio packet char array in the following format:
  //Node ID#, Uptime,
  sprintf(radiopacket, "%d,%d,%0.2f",data.nodeID, data.uptime, data.temp);
  
  //char radiopacket[20] = "Hello World #";
  //itoa(packetnum++, data.nodeID+13, 10);
  Serial.print("Sending packet #"); Serial.println(data.nodeID);

  //FIX: Updating Node ID
  data.nodeID++;
  
  // Send a message!
  rf69.send((unsigned char*)&radiopacket, sizeof(radiopacket));
  rf69.waitPacketSent();

  // Now wait for a reply
  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN] = "Hello World";
  uint8_t len = sizeof(buf);

  if (rf69.waitAvailableTimeout(500))  { 
    // Should be a reply message for us now   
    if (rf69.recv(buf, &len)) {
      Serial.print("Got a reply: ");
      Serial.println((char*)buf);
      Blink(LED, 50, 3); //blink LED 3 times, 50ms between blinks
    } else {
      Serial.println("Receive failed");
    }
  } else {
    Serial.println("No reply, is another RFM69 listening?");
  }

}

void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}
