/* ------------------- Library yang Digunakan ------------------*/
#include <ESP8266WiFi.h>
#include <MQTTClient.h>
#include <RFID.h>
#include <SPI.h>
#include <stdlib.h>


/* ---------------------- Koneksi Sensor RFID ke NodeMCU ----------------------
  MOSI  : D7
  MISO  : D6
  SCK   : D5
  SS/SDA: D0
  RST   : D10
  GND   : GND
  3.3V  : 3.3V

*/

/* ------------------- Koneksi Sensor MARVELMIND ke NodeMCu ------------------

  USART2_TX : RX
  Ground    : GND
*/

/* -------------------- Koneksi Sensor Enkoder ke NodeMCu -------------------
  Read pulse per second as train move along to calculate speed
  Pin layout should be as follows (on NodeMCU):
  Vin : HV2 (Logic Converter)
  -   : LV2 (Logic Converter) : 3.3V (NodeMCU)
  GND : GND
  DO  : D8 (Interrupt)
*/

/* ================================================================
   ===                    MQTT INITIAL SETUP                    ===
   ================================================================ */
const char ssid[] = "Redmi";                        //WiFi SSID
const char pass[] = "rushdialus";                   //WiFi password
const char host[] = "192.168.43.248 ";              //broker host
int port = 1882;                                    //port
const char clientID[] = "0004";                     //ID klien 
const char username[] = "0002";                     //MQTT username 
const char password[] = "test123";                  //MQTT password 
const String subs[] = {String("SinyalKontrol1")};   //daftar topik yang disubscribe 

//variabel global tambahan
unsigned long lastMillis = 0;
String data;
unsigned long timer = 0;
unsigned long startMillis;
float dt = 0;
int sinyalkontrol, direction, speed;

WiFiClient net;
MQTTClient client;

/* ================================================================
   ===                        VOID MQTT                         ===
   ================================================================ */

//fungsi untuk melakukan koneksi ke WiFi
void connectWiFi() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nWiFi connected!");
  Serial.println(WiFi.localIP());
}

//fungsi untuk melakukan koneksi ke MQTT
void connectMQTT() {
   Serial.print("\nconnecting...");
    while (!client.connect(clientID, username, password)) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nMQTT connected!");
}

//fungsi untuk melakukan subscribe pada broker MQTT
void subsMQTT() {
  for (int i = 0; i < 1; i++) {
    client.subscribe("SinyalKontrol1", 0);
    Serial.println("subscribe to " + subs[i] + " completed");
  }
}

//fungsi yang dipanggil saat menerima pesan masuk
void messageReceived(String &topic, String &payload) {
  if (topic == subs[0]) {
    sinyalkontrol = payload.toInt();  // nilai sinyal kontrol berdasarkan perhitungan Matlab

    if (sinyalkontrol < 0) {
      direction = 1;
    } else {
      direction = 0;
    }

    analogWrite(D1, payload.toInt()); // Perintah ke aktuator (motor)
    digitalWrite(D3, direction);
    startMillis = millis();
  }
}

/* ================================================================
   ===                        RFID SETUP                        ===
   ================================================================ */

#define SS_PIN 16
#define RST_PIN 10

RFID rfid(SS_PIN, RST_PIN);
int serNum[5];

//------ID Tag RFID yang digunakan-----//
int cards[][5] = {
  {43, 173, 11, 59, 182}, 
  {42, 186, 103, 202, 61}, 
  {122, 12, 249, 160, 47}, 
  {11, 199, 75, 58, 189}, 
  {128, 149, 248, 167, 74},
  {96, 190, 96, 168, 22},
  {240, 45, 250, 167, 128},
  {112, 153, 24, 167, 86},
  {16, 223, 255, 164, 148},
  {144, 181, 254, 167, 124},
  {112, 111, 155, 94, 218},
  {208, 56, 149, 94, 35},
  {101, 252, 24, 31, 158},
  {176, 172, 122, 162, 196}
};
float x_rfid, y_rfid;
String idrfid;
bool access = false;
int stateRFID = 0;


/* ================================================================
   ===                     MARVELMIND SETUP                     ===
   ================================================================ */

int hedgehogx = 0;
int hedgehogy = 0;
int hedgehog_x, hedgehog_y; // koordinat x dan y marvelmind
int hedgehog_z;             // koordinat z marvelmind
int hedgehog_pos_updated;   // variabel penanda adanya update data
bool high_resolution_mode;

#define HEDGEHOG_BUF_SIZE 40
#define HEDGEHOG_CM_DATA_SIZE 0x10
#define HEDGEHOG_MM_DATA_SIZE 0x16
byte hedgehog_serial_buf[HEDGEHOG_BUF_SIZE];
byte hedgehog_serial_buf_ofs;

#define POSITION_DATAGRAM_ID 0x0001
#define POSITION_DATAGRAM_HIGHRES_ID 0x0011
unsigned int hedgehog_data_id;
unsigned int hedgehog_address;

typedef union {
  byte b[1];
  unsigned int w;
} uni_8x1_8;
typedef union {
  byte b[2];
  unsigned int w;
  int wi;
} uni_8x2_16;
typedef union {
  byte b[4];
  float f;
  unsigned long v32;
  long vi32;
} uni_8x4_32;

// Marvelmind hedgehog support initialize
void setup_hedgehog()
{
  hedgehog_serial_buf_ofs = 0;
  hedgehog_pos_updated = 0;
}

// Marvelmind hedgehog service loop
void loop_hedgehog() {
  int incoming_byte;
  int total_received_in_loop;
  int packet_received;
  bool good_byte;
  byte packet_size;
  uni_8x1_8 un8;
  uni_8x2_16 un16;
  uni_8x4_32 un32;

  total_received_in_loop = 0;
  packet_received = 0;

  while (Serial.available() > 0)
  {
    if (hedgehog_serial_buf_ofs >= HEDGEHOG_BUF_SIZE)
    {
      hedgehog_serial_buf_ofs = 0;            // restart bufer fill
      break;                                  // buffer overflow
    }
    total_received_in_loop++;
    if (total_received_in_loop > 100) break; // too much data without required header

    incoming_byte = Serial.read();
    good_byte = false;
    switch (hedgehog_serial_buf_ofs)
    {
      case 0:
        {
          good_byte = (incoming_byte = 0xff);
          break;
        }
      case 1:
        {
          good_byte = (incoming_byte = 0x47);
          break;
        }
      case 2:
        {
          good_byte = true;
          break;
        }
      case 3:
        {
          hedgehog_data_id = (((unsigned int) incoming_byte) << 8) + hedgehog_serial_buf[2];
          good_byte =   (hedgehog_data_id == POSITION_DATAGRAM_ID) ||
                        (hedgehog_data_id == POSITION_DATAGRAM_HIGHRES_ID);
          break;
        }
      case 4:
        {
          switch (hedgehog_data_id)
          {
            case POSITION_DATAGRAM_ID:
              {
                good_byte = (incoming_byte == HEDGEHOG_CM_DATA_SIZE);
                break;
              }
            case POSITION_DATAGRAM_HIGHRES_ID:
              {
                good_byte = (incoming_byte == HEDGEHOG_MM_DATA_SIZE);
                break;
              }
          }
          break;
        }
      default:
        {
          good_byte = true;
          break;
        }
    }

    if (!good_byte)
    {
      hedgehog_serial_buf_ofs = 0;          // restart bufer fill
      continue;
    }
    hedgehog_serial_buf[hedgehog_serial_buf_ofs++] = incoming_byte;
    if (hedgehog_serial_buf_ofs > 5)
    {
      packet_size =  7 + hedgehog_serial_buf[4];
      if (hedgehog_serial_buf_ofs == packet_size)
      { // received packet with required header
        packet_received = 1;
        hedgehog_serial_buf_ofs = 0;        // restart bufer fill
        break;
      }
    }
  }

  if (packet_received)
  {
    hedgehog_set_crc16(&hedgehog_serial_buf[0], packet_size);// calculate CRC checksum of packet
    if ((hedgehog_serial_buf[packet_size] == 0) && (hedgehog_serial_buf[packet_size + 1] == 0))
    { // checksum success
      switch (hedgehog_data_id)
      {
        case POSITION_DATAGRAM_ID:
          {
            // Koordinat of beacon bergerak (X,Y), cm ==> mm
            un16.b[0] = hedgehog_serial_buf[9];
            un16.b[1] = hedgehog_serial_buf[10];
            hedgehog_x = 10 * long(un16.wi);

            un16.b[0] = hedgehog_serial_buf[11];
            un16.b[1] = hedgehog_serial_buf[12];
            hedgehog_y = 10 * long(un16.wi);

            // Ketinggian beacon bergerak, cm==>mm (FW V3.97+)
            un16.b[0] = hedgehog_serial_buf[13];
            un16.b[1] = hedgehog_serial_buf[14];
            hedgehog_z = 10 * long(un16.wi);

            un8.b[0] = hedgehog_serial_buf[22];
            hedgehog_address = un8.w;

            hedgehog_pos_updated = 1;         // Penanda jika ada update data
            high_resolution_mode = false;
            break;
          }

          
        // Opsi penentuan posisi sensor marvelmind dengan resolusi tinggi
        case POSITION_DATAGRAM_HIGHRES_ID:
          {
            // koordinat beacon bergerak (X,Y), mm
            un32.b[0] = hedgehog_serial_buf[9];
            un32.b[1] = hedgehog_serial_buf[10];
            un32.b[2] = hedgehog_serial_buf[11];
            un32.b[3] = hedgehog_serial_buf[12];
            hedgehog_x = un32.vi32;

            un32.b[0] = hedgehog_serial_buf[13];
            un32.b[1] = hedgehog_serial_buf[14];
            un32.b[2] = hedgehog_serial_buf[15];
            un32.b[3] = hedgehog_serial_buf[16];
            hedgehog_y = un32.vi32;

            // ketinggian beacon bergerak, mm
            un32.b[0] = hedgehog_serial_buf[17];
            un32.b[1] = hedgehog_serial_buf[18];
            un32.b[2] = hedgehog_serial_buf[19];
            un32.b[3] = hedgehog_serial_buf[20];
            hedgehog_z = un32.vi32;

            un8.b[0] = hedgehog_serial_buf[22];
            hedgehog_address = un8.w;

            hedgehog_pos_updated = 1; //  Penanda jika ada update data
            high_resolution_mode = true;
            break;
          }
      }
    }
  }
}

// Calculate CRC-16 of hedgehog packet
void hedgehog_set_crc16(byte *buf, byte size)
{ uni_8x2_16 sum;
  byte shift_cnt;
  byte byte_cnt;

  sum.w = 0xffffU;

  for (byte_cnt = size; byte_cnt > 0; byte_cnt--)
  {
    sum.w = (unsigned int) ((sum.w / 256U) * 256U + ((sum.w % 256U) ^ (buf[size - byte_cnt])));

    for (shift_cnt = 0; shift_cnt < 8; shift_cnt++)
    {
      if ((sum.w & 0x1) == 1) sum.w = (unsigned int)((sum.w >> 1) ^ 0xa001U);
      else sum.w >>= 1;
    }
  }

  buf[size] = sum.b[0];
  buf[size + 1] = sum.b[1]; // little endian
}// hedgehog_set_crc16


/* ================================================================
   ===                      ENCODER SETUP                       ===
   ================================================================ */
int encoder_pin1 = D2;
int encoder_pin2 = D8;
unsigned int rpm1 = 0;
unsigned int rpm2 = 0;
float velocity = 0;
volatile byte pulses1 = 0;
volatile byte pulses2 = 0;
unsigned long timeold1 = 0;
unsigned long timeold2 = 0;
unsigned int pulsesperturn = 20;
const int wheel_diameter = 17;
static volatile unsigned long debounce = 0;


//Menghitung pulsa yang dibaca enkoder//
void counter1() {
  if (  digitalRead (encoder_pin1) && (micros() - debounce > 500) && digitalRead (encoder_pin1) ) {
    debounce = micros();
    pulses1++;
  }
  else ;
}


/* ================================================================
   ===                      INITIAL SETUP                       ===
   ================================================================ */

void setup() {

  /*---------------------Connecting to WiFi----------------------*/
  Serial.begin(115200);
  WiFi.begin(ssid, pass);
  client.begin(host, port, net);
  client.onMessage(messageReceived);
  delay(10);
  connectWiFi();


  /* ---------------- RFID INITIAL SETUP --------------- */

  SPI.begin();
  rfid.init();

  /* -------------- MARVELMIND INITIAL SETUP ------------ */

  setup_hedgehog();
  /* --------------------- ENCODER SETUP ------------------- */
  attachInterrupt(encoder_pin1, counter1, RISING);
  //attachInterrupt(encoder_pin2, counter2, RISING);
  pulses1 = 0;
  rpm1 = 0;
  timeold1 = 0;


  /* --------------------- MQTT SETUP ------------------- */

  connectMQTT();
  subsMQTT();
  lastMillis = 0;


  // --- Membaca nilai awal sensor Marvelmind ---
  while (hedgehogx == 0) {
    loop_hedgehog();
    if (hedgehog_pos_updated == true)
    {
      if (hedgehog_address == 5) {
        hedgehogx = hedgehog_x;
        hedgehogy = hedgehog_y;
      }
      hedgehog_pos_updated = false;
    }
  }

  timeold2 = 250;
}

/* ================================================================
   ===                    MAIN PROGRAM LOOP                     ===
   ================================================================ */
long lastTimer = 0;

void loop() {
  /* -------------------- Memulai MQTT -------------------- */

  client.loop();
  if (WiFi.status() != WL_CONNECTED) {    //Menyambungkan kembali ke jaringan WiFi jika koneksi terputus
    connectWiFi();
  }

  if (!client.connected()) {              //Menyambungkan kembali ke broker MQTT jika koneksi terputus
    client.begin(host, port, net);
    client.onMessage(messageReceived);
    connectMQTT();
    subsMQTT();
  }

  /* ---------------- Pembacaan Sensor RFID ---------------- */

  if (rfid.isCard()) {

    if (rfid.readCardSerial()) {

      idrfid = String(rfid.serNum[0]) + String(rfid.serNum[1]) + String(rfid.serNum[2]) + String(rfid.serNum[3]) + String(rfid.serNum[4]);

      for (int x = 0; x < sizeof(cards); x++) {
        for (int i = 0; i < sizeof(rfid.serNum); i++ ) {
          if (rfid.serNum[i] != cards[x][i]) {
            access = false;
            break;
          } else {
            access = true;
          }
        }
        if (access) break;
      }
    }

    if ((access) && (idrfid == "431731159182")) {
      x_rfid = 373; y_rfid = 769;
      stateRFID = 11;
    } else if ((access) && (idrfid == "111997558189")) {
      x_rfid = 4; y_rfid = 4;
      stateRFID = 12;
    } else if ((access) && (idrfid == "12814924816774")) {
      x_rfid = 1378; y_rfid = 737;
      stateRFID = 7;
    } else if ((access) && (idrfid == "961909616822")) {
      x_rfid = 32; y_rfid = 447;
      stateRFID = 3;
    } else if ((access) && (idrfid == "24045250167128")) {
      x_rfid = 593; y_rfid = 70;
      stateRFID = 4;
    } else if ((access) && (idrfid == "1121532416786")) {
      x_rfid = 345; y_rfid = 44;
      stateRFID = 9;
    } else if ((access) && (idrfid == "16223255164148")) {
      x_rfid = 1342; y_rfid = 83;
      stateRFID = 10;
    } else if ((access) && (idrfid == "144181254167124")) {
      x_rfid = 10; y_rfid = 10;
      stateRFID = 5;
    } else if ((access) && (idrfid == "11211115594218")) {
      x_rfid = 1067; y_rfid = 75;
      stateRFID = 1;
    } else if ((access) && (idrfid == "208561499435")) {
      x_rfid = 1679; y_rfid = 382;
      stateRFID = 6;
    }  else if ((access) && (idrfid == "176172122162196")) {
      x_rfid = 1679; y_rfid = 382;
      stateRFID = 2;
    }  else if ((access) && (idrfid == "1012522431158")) {
      x_rfid = 1679; y_rfid = 382;
      stateRFID = 8;
    }
  } else {
    x_rfid = 0; y_rfid = 0;
  }

  rfid.halt();

  /* ---------------- Pembacaan Sensor Marvelmind ---------------- */

  loop_hedgehog();

  if (hedgehog_pos_updated == true)
  {
    if (hedgehog_address == 5) {
      hedgehogx = hedgehog_x;
      hedgehogy = hedgehog_y;
    }
    hedgehog_pos_updated = false;
  }

  /* ---------------- Pembacaan Sensor Enkoder ---------------- */
  if (millis() - timeold1 >= 300) {
    noInterrupts();
    rpm1 = (60 * 1000 / pulsesperturn ) / (millis() - timeold1) * pulses1;
    velocity = rpm1 * 3.1416 * wheel_diameter / 60000;
    timeold1 = millis();
    pulses1 = 0;
    interrupts();
  }

  /* ------------------ Pengiriman Data ------------------- */
  dt = (float)(micros() - timer) / 1000000;
  timer = micros();
  String data = String(hedgehogx) + ";" + String(hedgehogy) + ";" + String(stateRFID) + ";" + String(velocity) + ";" + String(dt) + ";" + String(sinyalkontrol);
  client.publish("datasensor1", data, 0, 0);
}
