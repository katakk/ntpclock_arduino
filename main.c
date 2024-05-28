#include <SPI.h>
#include <avr/wdt.h>

#include <MsTimer2.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
const byte interruptPin = 2;
const int latchPin = 7; //w
// 11 , 10 ng
const int dataPin = 5; // br
const int clockPin = 6; // pu
// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEA
};
byte RefID[4] = {0};
byte emp = 0b00000000;
byte dot = 0b00000001;
byte dec[] = {
  0b11111100, // 0
  0b01100000, // 1
  0b11011010, // 2
  0b11110010, // 3
  0b01100110, // 4
  0b10110110, // 5
  0b10111110, // 6
  0b11100000, // 7
  0b11111110, // 8
  0b11110110, // 9
};
unsigned int localPort = 8888; // local port to listen for UDP packets
char timeServer[] = "192.168.78.255";
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[256]; //buffer to hold incoming and outgoing packets
// A UDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

unsigned long b = 0;
unsigned long t = 0;
unsigned long nt = 0;
unsigned long bm = 0;
unsigned long epoch = 0;

void software_reset() {
  asm volatile ("  jmp 0");
}

void clear()
{
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, emp);
  shiftOut(dataPin, clockPin, LSBFIRST, emp);
  shiftOut(dataPin, clockPin, LSBFIRST, emp);
  shiftOut(dataPin, clockPin, LSBFIRST, emp);
  shiftOut(dataPin, clockPin, LSBFIRST, emp);
  shiftOut(dataPin, clockPin, LSBFIRST, emp);
  shiftOut(dataPin, clockPin, LSBFIRST, emp);
  shiftOut(dataPin, clockPin, LSBFIRST, emp);
  shiftOut(dataPin, clockPin, LSBFIRST, emp);
  shiftOut(dataPin, clockPin, LSBFIRST, emp);
  shiftOut(dataPin, clockPin, LSBFIRST, emp);
  shiftOut(dataPin, clockPin, LSBFIRST, emp);
  digitalWrite(latchPin, HIGH);
}

void visible(unsigned long bm, unsigned long b, unsigned long nt)
{
  unsigned long tm = millis() - bm;
  tm /= 1000; //s
  tm += epoch;

  int h = (((((tm % 86400L) / 3600)) + 9 ) % 24);
  if ( h < 5 ) h += 24;
  int m = (tm % 3600) / 60;
  int s = tm % 60;

  t  = nt + micros() - b;
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, ((h / 10) % 10) ? dec[(h / 10) % 10] : emp);
  shiftOut(dataPin, clockPin, LSBFIRST, dec[(h / 1) % 10] | dot);
  shiftOut(dataPin, clockPin, LSBFIRST, dec[(m / 10) % 10]);
  shiftOut(dataPin, clockPin, LSBFIRST, dec[(m / 1) % 10] | dot);
  shiftOut(dataPin, clockPin, LSBFIRST, dec[(s / 10) % 10]);
  shiftOut(dataPin, clockPin, LSBFIRST, dec[(s / 1) % 10] | dot);
  shiftOut(dataPin, clockPin, LSBFIRST, dec[(t / 100000) % 10]);
  shiftOut(dataPin, clockPin, LSBFIRST, dec[(t / 10000) % 10]);
  shiftOut(dataPin, clockPin, LSBFIRST, dec[(t / 1000) % 10]);
  shiftOut(dataPin, clockPin, LSBFIRST, dec[(t / 100) % 10]);
  shiftOut(dataPin, clockPin, LSBFIRST, dec[(t / 10) % 10]);
  shiftOut(dataPin, clockPin, LSBFIRST, dec[(t / 1) % 10]);
  digitalWrite(latchPin, HIGH);
}

void display()
{
  epoch ? visible(bm, b, nt) : clear();
}

void sendNTPpacket(char* address)
{
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011; // LI, Version, Mode
  packetBuffer[1] = 0; // Stratum, or type of clock
  packetBuffer[2] = 6; // Polling Interval
  packetBuffer[3] = 0xEC; // Peer Clock Precision
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

void ntp() {

  b  = micros();
  bm  = millis();


  Udp.read(packetBuffer, NTP_PACKET_SIZE);
  /*
  if (
    packetBuffer[0] == 0xE3 &&
    packetBuffer[1] == 0 &&
    packetBuffer[2] == 6 &&
    packetBuffer[3] == 0xEC ) return;
*/
// ntp.recvtime =! NULL
  if (packetBuffer[36] == 0 && 
  packetBuffer[37] == 0 && 
  packetBuffer[38] == 0 && 
  packetBuffer[39] == 0 && 
  packetBuffer[40] == 0 && 
  packetBuffer[41] == 0 && 
  packetBuffer[42] == 0 && 
  packetBuffer[43] == 0 ) return;

/*
if(
  packetBuffer[4] == 0 &&
  packetBuffer[5] == 0 &&
  packetBuffer[6] == 0 &&
  packetBuffer[7] == 0)
  return;
  */
if(
  RefID[0] == 0 &&
  RefID[1] == 0 &&
  RefID[2] == 0 &&
  RefID[3] == 0)
  memcpy(RefID, packetBuffer +12, 4);


  if(memcmp(RefID, packetBuffer +12, 4))
  {
      int i;  
    
    for (i = 12; i < 12+4; i++) Serial.print(packetBuffer[i], HEX);
    Serial.print(" is not " ); 
    
    for (i = 0; i < 4; i++) Serial.print(RefID[i], HEX);
    
    Serial.println(" " );
    return;
  }

  
    int i;
   // for (i = 12; i < 16; i++) Serial.print(packetBuffer[i], HEX);
    
    i = 0;
    /*
    for (; i < 4; i++) Serial.print(packetBuffer[i], HEX);
    Serial.print(" RD" );
    for (; i < 8; i++) Serial.print(packetBuffer[i], HEX);
    Serial.print(" RD" );
    for (; i < 12; i++) Serial.print(packetBuffer[i], HEX);
    Serial.print(" ID" );
    for (; i < 16; i++) Serial.print(packetBuffer[i], HEX);
    */
    /*
    Serial.print(" RT" );
    for (; i < 24; i++) Serial.print(packetBuffer[i], HEX);
    Serial.print(" OT" );
    for (; i < 32; i++) Serial.print(packetBuffer[i], HEX);
    Serial.print(" RT" );
    for (; i < 40; i++) Serial.print(packetBuffer[i], HEX);
    Serial.print(" TT" );
    for (; i < 48; i++) Serial.print(packetBuffer[i], HEX);
    */
    
   // Serial.println(" " );

  unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
  unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
  unsigned long secsSince1900 = highWord << 16 | lowWord;
  const unsigned long seventyYears = 2208988800UL;
  epoch = secsSince1900 - seventyYears;

  highWord = word(packetBuffer[36], packetBuffer[37]);
  lowWord = word(packetBuffer[38], packetBuffer[39]);
  nt = highWord << 16 | lowWord;


  

}



void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  // start Ethernet and UDP
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // no point in carrying on, so do nothing forevermore:
    delay(3000);
    software_reset();
  }

  Udp.begin(localPort);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  clear();
}

int n = 0;
int reboot = 0;
void loop() {
   int len;
  long timing = millis() + 1000;
  sendNTPpacket(timeServer); // send an NTP packet to a time server
  while (millis() < timing)
  {
    display();
    delay(30);
  }
  reboot++;
  if (reboot > 1800 ) software_reset();

  //Serial.print(len, DEC);
    len =Udp.parsePacket();
  if (len == NTP_PACKET_SIZE) ntp();
  Ethernet.maintain();
}




