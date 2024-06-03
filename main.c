#include <SPI.h>
#include <avr/wdt.h>

#include <MsTimer2.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

const byte interruptPin = 2;
const int latchPin = 7; // w
// 11 , 10 ng
const int dataPin = 5;  // br
const int clockPin = 6; // pu
// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield

byte mac[] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEA};
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
byte packetBuffer[256];         // buffer to hold incoming and outgoing packets
// A UDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

unsigned long recved_micro = 0;
unsigned long recved_time = 0;
unsigned long epoch = 0;
int reboot = 0;

void software_reset()
{
  asm volatile("  jmp 0");
}

void hms(unsigned long tm, int *h, int *m, int *s)
{
  *h = (((((tm % 86400L) / 3600)) + 9) % 24);
  if (*h < 5)
    *h += 24;
  *m = (tm % 3600) / 60;
  *s = tm % 60;
}

void disp_hms(int h, int m, int s, unsigned long t)
{
  Serial.print(h, DEC);
  Serial.print(":");
  Serial.print(m, DEC);
  Serial.print(":");
  Serial.print(s, DEC);
  Serial.print(".");
  Serial.print(t, DEC);
  Serial.println(" ");
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

void visible_hms(int h, int m, int s, unsigned long t)
{
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

void visible(unsigned long nt)
{
  unsigned long now = millis();
  unsigned long micro = micros();
  unsigned long tm = now - recved_time;
  unsigned long t = nt + micro - recved_micro;
  tm /= 1000; // s
  tm += epoch;
  int h;
  int m;
  int s;
  hms(tm, &h, &m, &s);
  visible_hms(h, m, s, t);
}
void display(unsigned long nt)
{
  int n;

  for (n = 0; n < 30; n++)
  {
    unsigned long timing = millis() + 1000;
    while (millis() < timing)
    {
      epoch ? visible(nt) : clear();
      delay(30);
    }
  }
}

void sendNTPpacket(char *address)
{
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011; // LI, Version, Mode
  packetBuffer[1] = 0;          // Stratum, or type of clock
  packetBuffer[2] = 6;          // Polling Interval
  packetBuffer[3] = 0xEC;       // Peer Clock Precision
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  Udp.beginPacket(address, 123); // NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

void ntp(unsigned long *nt)
{
  byte *recvtime = packetBuffer + 36;
  byte *refid = packetBuffer + 12;

  Udp.read(packetBuffer, NTP_PACKET_SIZE);

  // ntp.recvtime =! NULL
  if (recvtime[0] == 0 &&
      recvtime[1] == 0 &&
      recvtime[2] == 0 &&
      recvtime[3] == 0 &&
      recvtime[4] == 0 &&
      recvtime[5] == 0 &&
      recvtime[6] == 0 &&
      recvtime[7] == 0)
    return;

  if (
      RefID[0] == 0 &&
      RefID[1] == 0 &&
      RefID[2] == 0 &&
      RefID[3] == 0)
    memcpy(RefID, refid, 4);

  if (memcmp(RefID, refid, 4))
  {
    int i;
    /*
        for (i = 12; i < 12 + 4; i++)
          Serial.print(packetBuffer[i], HEX);
        Serial.print(" is not ");

        for (i = 0; i < 4; i++)
          Serial.print(RefID[i], HEX);

        Serial.println(" ");
    */
    return;
  }

  if (0)
  {
    int i;
    i = 0;
    for (; i < 4; i++)
      Serial.print(packetBuffer[i], HEX);
    Serial.print(" RD");
    for (; i < 8; i++)
      Serial.print(packetBuffer[i], HEX);
    Serial.print(" RD");
    for (; i < 12; i++)
      Serial.print(packetBuffer[i], HEX);
    Serial.print(" ID");
    for (; i < 16; i++)
      Serial.print(packetBuffer[i], HEX);

    Serial.print(" RT");
    for (; i < 24; i++)
      Serial.print(packetBuffer[i], HEX);
    Serial.print(" OT");
    for (; i < 32; i++)
      Serial.print(packetBuffer[i], HEX);
    Serial.print(" RT");
    for (; i < 40; i++)
      Serial.print(packetBuffer[i], HEX);
    Serial.print(" TT");
    for (; i < 48; i++)
      Serial.print(packetBuffer[i], HEX);

    Serial.println(" ");
  }

  do
  {
    unsigned long h = makeWord(recvtime[4], recvtime[5]);
    unsigned long l = makeWord(recvtime[6], recvtime[7]);
    unsigned long secsSince1900 = h << 16 | l;
    const unsigned long seventyYears = 2208988800UL;
    epoch = secsSince1900 - seventyYears;
  } while (0);

  do
  {
    unsigned long h = makeWord(recvtime[0], recvtime[1]);
    unsigned long l = makeWord(recvtime[2], recvtime[3]);
    *nt = h << 16 | l;
  } while (0);

  recved_time = millis();
  recved_micro = micros();

  {
    int h;
    int m;
    int s;
    hms(epoch, &h, &m, &s);
    disp_hms(h, m, s, *nt);
  }
}

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  // start Ethernet and UDP
  if (Ethernet.begin(mac) == 0)
  {
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

void __update(unsigned long *nt)
{
  int len;
  sendNTPpacket(timeServer);
  len = Udp.parsePacket();
  if (len == NTP_PACKET_SIZE)
  {
    ntp(nt);
  }
  Ethernet.maintain();
}

void update(unsigned long *nt)
{
  int n;
  for (n = 0; n < 30; n++)
  {
    __update(nt);
    if (epoch)
    {
      break;
    }
    delay(30);
  }
}

void check()
{

  if (!epoch)
  {
    reboot++;
  }

  if (reboot > 18)
  {
    software_reset();
  }

  epoch = 0;
}

void loop()
{
  unsigned long nt;
  update(&nt);
  display(nt);
  check();
}
