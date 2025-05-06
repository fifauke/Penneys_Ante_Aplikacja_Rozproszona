#include <SPI.h>
#include <ZsutEthernet.h>
#include <ZsutEthernetUdp.h>
#include <ZsutFeatures.h>

#pragma pack(push, 1)
typedef struct {
    uint8_t  ver_type;
    uint8_t  flags;
    uint16_t seqNum;
    uint16_t clientID;
    uint16_t payloadLength;
} ALP_Header;
#pragma pack(pop)

#define MSGT_ACK       0
#define MSGT_REGISTER  1
#define MSGT_START     2
#define MSGT_TOSS      3
#define MSGT_REPORT    4
#define MSGT_ENDGAME   5

#define FLAG_ACK_REQUEST  (1 << 0)
#define FLAG_ACK_RESPONSE (1 << 1)

#define PIN_EDGE ZSUT_PIN_D2
#define PIN_BIT  ZSUT_PIN_D3

#define PATTERN_SIZE_IN_BYTES 1

byte mac[] = { 0xDE, 0xAD, 0xDE, 0xEF, 0xFE, 0xED };
ZsutIPAddress serverIP(192, 168, 56, 107);
unsigned int serverPort = 12345;
unsigned int localPort = 54321;

ZsutEthernetUDP Udp;

static uint16_t g_myClientID = 0;  // otrzymywany po rejestracji
static uint16_t g_seqNum    = 1;

const int PATTERN_LENGTH = 5;
static int myPattern[5];


static int tossHistory[PATTERN_LENGTH];
static int tossCount = 0;

bool gameActive = false;

bool g_reported = false;


// Funkcje pomocnicze

uint16_t htons(uint16_t value) {
  return (value << 8) | (value >> 8); // Zamiana kolejności bajtów w 16-bitowej liczbie
}

uint32_t htonl(uint32_t value) {
  return ((value & 0xFF) << 24) |
         ((value & 0xFF00) << 8) |
         ((value & 0xFF0000) >> 8) |
         ((value >> 24) & 0xFF); // Zamiana kolejności bajtów w 32-bitowej liczbie
}

uint16_t ntohs(uint16_t value) {
  return (value << 8) | (value >> 8); // Zamiana kolejności bajtów w 16-bitowej liczbie
}

uint32_t ntohl(uint32_t value) {
  return ((value & 0xFF) << 24) |
         ((value & 0xFF00) << 8) |
         ((value & 0xFF0000) >> 8) |
         ((value >> 24) & 0xFF); // Zamiana kolejności bajtów w 32-bitowej liczbie
}

// Funkcja pomocnicza do przekonwertowania historii na uint8
uint8_t convertBinaryArrayToUint8(const int pattern[], size_t size) {
    uint8_t result = 0;

    for (size_t i = 0; i < size; ++i) {
        result = (result << 1) | (pattern[i] & 0x01);
    }

    return result;
}

void readPatternFromGPIO() {
  Serial.println("Odczyt patternu z GPIO...");
  ZsutPinMode(PIN_EDGE, INPUT);
  ZsutPinMode(PIN_BIT, INPUT);

  for (int i = 0; i < PATTERN_LENGTH; i++) {

    Serial.print("Wartosc D2 (stan): ");
    Serial.println((ZsutDigitalRead() >> 2) & 0b1);
    
    Serial.print("Wartosc D3 (bit): ");
    Serial.println((ZsutDigitalRead() >> 3) & 0b1);

    // czekaj na zbocze
    while (((ZsutDigitalRead() >> 2) & 0b1) == 0) {
    }

    // odczytaj bit
    myPattern[i] = (ZsutDigitalRead() >> 3) & 0b1;
    Serial.print("Odczytano bit: ");
    Serial.println(myPattern[i]);
    // czekaj aż edge zaniknie, by nie zliczać wielokrotnie
    while (((ZsutDigitalRead() >> 2) & 0b1) == 1) {
    }
  }
  Serial.print("Pattern odczytany: ");
  for (int i = 0; i < PATTERN_LENGTH; i++) {
      Serial.print(myPattern[i]);
  }
  Serial.println();
}

void sendRegister() {

  uint8_t convertedPattern = convertBinaryArrayToUint8(myPattern, PATTERN_LENGTH);  // htons nie jest potrzebny bo przesylamy tylko jeden bajt

  uint8_t buffer[sizeof(ALP_Header) + sizeof(convertedPattern)];
  memset(buffer, 0, sizeof(buffer));

  ALP_Header* hdr = (ALP_Header*)buffer;
  // ver=1, REGISTER=1
  hdr->ver_type = ((1 & 0x0F) << 4) | (MSGT_REGISTER & 0x0F);
  hdr->flags = FLAG_ACK_REQUEST;
  hdr->seqNum = htons(g_seqNum++);
  hdr->clientID = htons(0);
  hdr->payloadLength = htons(sizeof(convertedPattern));

  uint8_t* payload = buffer + sizeof(ALP_Header);
  memcpy(payload, &convertedPattern, sizeof(convertedPattern));

  Udp.beginPacket(serverIP, serverPort);
  Udp.write(buffer, sizeof(buffer));
  Udp.endPacket();
}

void sendReport(uint32_t tossCount) {
  uint8_t buffer[sizeof(ALP_Header) + 8];
  memset(buffer, 0, sizeof(buffer));

  ALP_Header* hdr = (ALP_Header*)buffer;
  hdr->ver_type = ((1 & 0x0F) << 4) | (MSGT_REPORT & 0x0F);
  hdr->flags = FLAG_ACK_REQUEST;
  hdr->seqNum = htons(g_seqNum++);
  hdr->clientID = htons(g_myClientID);
  hdr->payloadLength = htons(8);

  uint8_t* payload = buffer + sizeof(ALP_Header);
  uint32_t game_id_n = htonl(0);
  memcpy(payload, &game_id_n, 4);
  uint32_t toss_count_n = htonl(tossCount);
  memcpy(payload+4, &toss_count_n, 4);

  Serial.println("Wysylam pakiet typu report");

  Udp.beginPacket(serverIP, serverPort);
  Udp.write(buffer, sizeof(ALP_Header) + 8);
  Udp.endPacket();
}

bool checkPatternOccurred() {
  for (int i = 0; i < PATTERN_LENGTH; i++) {
    if (tossHistory[i] != myPattern[i]) {
      return false;
    }
  }
  return true;
}

void addBitToHistory(int bitVal) {
  // przesuwamy w lewo
  for (int i = 0; i < PATTERN_LENGTH-1; i++) {
    tossHistory[i] = tossHistory[i+1];
  }
  tossHistory[PATTERN_LENGTH-1] = bitVal;
}

void setup() {
  Serial.begin(9600);
  while(!Serial) { ; }

  ZsutEthernet.begin(mac);
  Udp.begin(localPort);

  readPatternFromGPIO();

  sendRegister();
  Serial.println("Wyslano REGISTER do serwera.");
  
  for (int i = 0; i < PATTERN_LENGTH; i++) {
    tossHistory[i] = -1;
  }
}

void loop() {
  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    uint8_t buffer[1500];
    int len = Udp.read(buffer, 1500);
    if (len >= (int)sizeof(ALP_Header)) {
      ALP_Header* hdr = (ALP_Header*)buffer;
      uint16_t payLen = ntohs(hdr->payloadLength);
      uint8_t msgType = (hdr->ver_type & 0x0F);

      if (hdr->flags & FLAG_ACK_REQUEST) {
        
        uint8_t ackBuf[sizeof(ALP_Header)];
        memset(ackBuf, 0, sizeof(ackBuf));
        ALP_Header* ahdr = (ALP_Header*)ackBuf;
        ahdr->ver_type = ((1 & 0x0F) << 4) | (MSGT_ACK & 0x0F);
        ahdr->flags = FLAG_ACK_RESPONSE;
        ahdr->seqNum = hdr->seqNum;
        ahdr->clientID = htons(g_myClientID);
        ahdr->payloadLength = htons(0);

        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write(ackBuf, sizeof(ackBuf));
        Udp.endPacket();
      }

      switch (msgType) {
        case MSGT_ACK: {
          uint16_t cid = ntohs(hdr->clientID);
          if (g_myClientID == 0 && cid != 0) {
            g_myClientID = cid;
            Serial.print("Otrzymano ACK z nowym clientID=");
            Serial.println(g_myClientID);
          }
        } break;

        case MSGT_START: {
          gameActive = true;
          tossCount = 0;
          g_reported = false;
          
          for (int i=0; i<PATTERN_LENGTH; i++){
            tossHistory[i] = -1;
          }
          Serial.println("Nowa gra START");
        } break;

        case MSGT_TOSS: {
          if (!gameActive) {
            break;
          }
          if (payLen >= 1) {
            int bitVal = buffer[sizeof(ALP_Header)];
            addBitToHistory(bitVal);
            tossCount++;
            
            if (checkPatternOccurred()) {
    
              if (!g_reported) {
                sendReport(tossCount);
                g_reported = true;
                Serial.print(">>> Wzorzec znaleziony przy tossCount=");
                Serial.println(tossCount);
              }
            }
          }
        } break;

        case MSGT_ENDGAME: {
          gameActive = false;
          Serial.println("Gra zakonczona (ENDGAME).");
        } break;
      }
    }
  }

  delay(50);
}