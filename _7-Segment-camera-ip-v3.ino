
/*
  Basic ESP8266 MQTT 7 Segment driver 

*/
#include <stdio.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266httpUpdate.h>
#include <SevenSegment.h>
#include <ov528.h>

#define GREEN     14         // Control the Green ; write '1'
#define RED       16         // Control the Green ; write '1'
#define CLK       13          // This is the clock pin
#define DATA      12           // This is the data pin
#define MAXDIGITS 4       // number of display digits
#define MAXSSID   4
#define DIGIT_BUFFERSIZE 5 * MAXDIGITS +1  // set buffersize for 7 Segment show messages + msg cnt 
#define REBOOT_CNT_BUFFERSIZE 2  // set buffersize for reboot counter 

#define uchar unsigned char

// Some character data to be displayed
unsigned char data[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'a', 'B', 'b', 
                         'C', 'c', 'D', 'd', 'E', 'e', 'F', 'f', 'G', 'g', 'H', 'h', 'I', 'i', 
                         'J', 'j', 'L', 'l', 'N', 'n', 'O', 'o', 'P', 'p', 'Q', 'q', 'R', 'r',
                         'S' ,'s', 'T', 't', 'U', 'u', 'Y', 'y', '-', '?', '=', '"' };
                         
SevenSegment seg = SevenSegment(DATA,CLK,0 ,MAXDIGITS);    // Create the SevenSegment Object
EspClass esp;     // Create  control object to reset device 
WiFiClient espClient;
WiFiClient picClient;
PubSubClient client(espClient);

bool FW_Update( char* updatemsg);

#define HAVN

// Update these with values suitable for your network.
#ifdef IPHONE
const char* ssid = "Mikes 7";
const char* password = "Frdrborgvej48";
#endif

#ifdef HOME
const char* ssid = "Davinci";
const char* password = "xxxxx";
#endif
#ifdef HAVN_TEST
const char* ssid = "ByensBedsteHavn";
const char* password = "#Havn4000";
#endif
#ifdef HAVN
const char* ssid = "RoskildeHavn_5B";
const char* password = "";
#endif



const char *ssid_list[MAXSSID] = {
    "RoskildeHavn_5B",
    "RoskildeHavn_4B",
    "RoskildeHavn_1B"
    };

const char *ssid_psw[MAXSSID] = {
//    "xxxxxx",
    "",
    "",
    ""
    };

const char* mqtt_server = "mqtt.technovator.dk";
const char* esp_binary =  "/rh-version.htm";
const char* update_server = "ota.technovator.dk";
const char* pic_server = "ota.technovator.dk";  


const char* swvers = "RH2f$";


byte val;

char tmsg[200];
long lastMsg = 0;
// char msg[50];
static  char mynetinfo[200]; // IP, RSSI and short if 
static char myid[8];      // my listner id, part of MAC address 
unsigned char digits[DIGIT_BUFFERSIZE];  //  mcg cnt + 5 buffers to digit array used to display characters
static char rhid[10];     // RH id :   "RH-xxxx/" 
int msg_cnt = 0;
int msg_ndx = 1;
volatile  int reconnect_cnt = 0; // to use as reset/reboot counter connection to MQTT 
byte nx = 0x1; 
volatile bool state = false;
volatile bool snapshot = false;
volatile bool camera_present = false;
static byte reboot_cnt = 0;    // store msg cnt and msg in eeprom 


void setup() {
  memset(digits, 0, DIGIT_BUFFERSIZE); 
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  pinMode(GREEN, OUTPUT);     // Initialize the GREEN pin as an output
  pinMode(RED, OUTPUT);     // Initialize the RED pin as an output
  digitalWrite(GREEN,LOW);
  digitalWrite(RED,HIGH);
  EEPROM.begin(DIGIT_BUFFERSIZE + REBOOT_CNT_BUFFERSIZE);   // Init eeprom writing (in flash) + cnt
  // EEPROM.begin(REBOOT_CNT_BUFFERSIZE);   // Init eeprom writing 
  seg.clearDisplay(); 
  Serial.begin(57600);
  Serial.println("\nRH 7 Segement driver + MQTT - Havne projekt OTA + Camera");
  Serial.setDebugOutput(true);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  reboot_cnt = EEPROM.read(22);  // Get reboot count 
  reconnect(); 
  if (reconnect_cnt > 1 ) {
      String mik = "Resetting : reconnect Count: " + String(reconnect_cnt);
      snprintf (tmsg, 75, "%s", mik.c_str() );
      client.publish("RH-CTRL/", tmsg);
  }
  
  seg.displayChars((unsigned char*)  swvers);
  delay (4000);
  
  seg.displayChars((unsigned char*)  myid);
  delay (4000);

  if (!restore_msg())  // revert to default if no msg to show
  {
    digits[0] = '=';
    digits[1] = '=';
    digits[2] = '=';
    digits[3] = '=';
    msg_cnt = -1;
  } 
  seg.displayChars(digits);

  Serial.println("Camera Sync");
  if (camera_sync() ) {
     camera_init(); 
     camera_powerdown();
     camera_present = true;
  }
  else 
  {
     camera_present = false;
     Serial.println("No Camera response -   leaving setup");
  }
}



bool restore_msg()
{
  // Get message count from 
  int rv =  EEPROM.read(0);  // read msg count byte from Eeprom 
  if (rv > 0 && rv <= 5) {
    msg_cnt = rv;  // use the first byte at count of dsp messages
    msg_ndx = 1;  // Start at index 1
    for (int i = 0; i < DIGIT_BUFFERSIZE; i++) {
      digits[i] = EEPROM.read(i);  // copy 7 seg msg from flash
    }
    return true; 
  }
  else   
    return false;
}

void status(bool init)
{
  byte mac[6];
  if (init) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");        
    Serial.println(WiFi.localIP());
    WiFi.macAddress(mac); 
    Serial.print("MAC: "); Serial.print(mac[5],HEX); Serial.print(":");
    Serial.print(mac[4],HEX); Serial.print(":"); Serial.print(mac[3],HEX); Serial.print(":"); Serial.print(mac[2],HEX);
    Serial.print(":"); Serial.print(mac[1],HEX); Serial.print(":"); Serial.println(mac[0],HEX);  
    Serial.println(esp.getChipId(), HEX);
    sprintf((char*) myid, "%2X%2X",mac[5],mac[4]); 
    sprintf(rhid, "RH-%s/",myid);  
  }

  long rssi = WiFi.RSSI();
  Serial.print("RSSI:");
  Serial.println(rssi);
  IPAddress ip = WiFi.localIP();  
  sprintf((char*) mynetinfo, "%d.%d.%d.%d RSSI:%ld, id:%s, SW:%s Cam:%d, Boot:%d", ip[0],ip[1],ip[2],ip[3], WiFi.RSSI(), myid, swvers, camera_present,reboot_cnt ); 
  Serial.println((char*) mynetinfo);
  Serial.print("bootvers: ");Serial.println(esp.getBootVersion());
  Serial.print("getFlashChipRealSize(): ");Serial.println(esp.getFlashChipRealSize());
  Serial.print("getFlashChipSize(): ");Serial.println(esp.getFlashChipSize());
  Serial.print("getFreeSketchSpace(): ");Serial.println(esp.getFreeSketchSpace());
  Serial.print("getSketchSize(): ");Serial.println(esp.getSketchSize());
}



void setup_wifi() {
  int ssidndx = 0; 
  digits[0] = nx = 0x80;
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
//  Serial.println(ssid_list[ssidndx]);
 // WiFi.begin(ssid_list[ssidndx], ssid_psw[ssidndx]);
  Serial.println(ssid);  
  WiFi.begin(ssid, password);
  delay(1000);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(900);
    seg.segOutput(digits);
    Serial.print(".");
    if (nx == 1) {   // try 7 times, then switch to next ssid
       ssidndx =  ssidndx >= MAXSSID ? 0 :  ssidndx + 1;
       nx = 0x80;
       WiFi.disconnect(0);
 //      Serial.println(ssid_list[ssidndx]);
   //    WiFi.begin(ssid_list[ssidndx], ssid_psw[ssidndx]);
       WiFi.begin(ssid, password);
    }
    digits[ssidndx] = nx ;  //  rotate a bit around the  7segment.
    nx = nx >> 1; 
  }
  //digitalWrite(GREEN,LOW);
  digits[0] = 2; // '-';  // Wifi done
  seg.segOutput(digits);

  status(true);

}

void callback(char* topic, byte* payload, unsigned int length) {
  
//  Serial.print("Message arrived [");
//  Serial.print(topic);
//  Serial.print("] ");
//  for (int i = 0; i < length; i++) {
//    Serial.print((char)payload[i]);
//  }
//  Serial.println();

  sprintf(tmsg, "RH-%s-ACK", myid) ;  // Prepare response
  payload[length] = 0; // use for string operations
  
  if ( isdigit((char)payload[0] )) {
      memset(digits, 0, sizeof(digits));    // clear all digits to space
      memcpy(digits,payload,length);
      msg_cnt = (int) digits[0] - '0';  // turn into integer
      msg_ndx = (int) 1; 
      EEPROM.write(0,msg_cnt);    // store msg cnt and msg in eeprom 
      for (int i = 0; i < DIGIT_BUFFERSIZE; i++){
        EEPROM.write(i+1,digits[i+1]);  // copy 7 seg msg to  flash 
      }
      EEPROM.commit();  // transfer to eeprom
  } else if ( strcmp((char*)payload, "status") == 0 )  {
      status(false);   // refresh status 
      if (msg_cnt >= 0 ) {    // add msg to status info 
         char m[30]; memset(m, 0, sizeof(m)); 
         int l = msg_cnt == 0 ? 1: msg_cnt;  
         memcpy (m, &digits[1], 4 * l); // prepare 7 seg msg   
         sprintf(tmsg, "%s msg:(%s)", (char*) mynetinfo, m); // use org message. 
      } 
      else
         sprintf(tmsg, "%s-ACK: %s msg:(idle)", myid, (char*) mynetinfo);  // Prepare response
      Serial.println((char*) mynetinfo);
      EEPROM.write(22,0);    // reset reboot count in eeprom
      EEPROM.commit();  // transfer to eeprom
      reboot_cnt = 0; 

  } else if ( strcmp((char*)payload, "reboot") == 0 )  {
      sprintf(tmsg, "%s - rebooting", myid) ;  // Prepare response
      client.publish("RH-CTRL/",  (const char*) tmsg  );
      Serial.println("Rebooting");
      esp.reset(); 
  } else if ( strstr((char*)payload, "update") != NULL)  {  // Search for "update nnn.nnn.nnn.nnn /file;"
     Serial.println("Update request"); 
     FW_Update((char*) payload);
  } else if ( strstr((char*)payload, "snapshot") != NULL)  {  // take picture ;"
    if (camera_present) {  // Only serve camera if available 
       camera_sync();
       camera_init();
       get_picsrvip(payload);
       if (!picClient.connect(pic_server, 6000)) {
          Serial.print("PIC server connection failed:"); Serial.println(update_server);
       }
       camera_snapshot(); 
       snapshot = true;
       sprintf(tmsg, "%s-ACK (%s): - snapshot", myid,update_server ) ;
      
    }
    else
      sprintf(tmsg, "%s-ACK: - no camera available", myid) ;
  }
  client.publish("RH-CTRL/",  (const char*) tmsg  );
}

void get_picsrvip(byte * pl)
{
    char *pch =  strtok((char*)pl, " ");  // Ignore cmd id
    if (pch != NULL) {                 // search for pic server ip 
        pch = strtok(NULL, " ");        
        if (pch != NULL) { 
           if (strchr( (const char*) pch, '.') != NULL)  {  // check for valid ip address
            strcpy((char*)update_server, (const char*) pch);   // overrule default update server
           }
        }
     }
    Serial.println(update_server);
}



void reconnect() {
  nx = 0x1;

  if (client.connected())
    return;

  
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect( rhid) ) {
      Serial.println("connected");
      reconnect_cnt = 0; // reset counter.
      client.subscribe(rhid);
      client.publish("RH-CTRL/", mynetinfo);
      client.subscribe("RH-global/");  // for global sw updates. 
      // Once connected, publish an announcement...
      Serial.println("RH-CTRL/ connected");
      Serial.println(String (rhid) + String(" connected") );
      restore_msg();  // get old msg if any 
      return;
    } 
    else {
      nx = nx == 1 ? 0x80 : nx >> 1;
      digits[1] = nx ;  //  rotate a bit around the  7segment.
      seg.segOutput(digits);
       
      if (++reconnect_cnt > 5 ) 
      {
        Serial.println("Resetting device");
        EEPROM.write(22,++reboot_cnt);    // store msg cnt and msg in eeprom 
        EEPROM.commit();  // transfer to eeprom
        esp.reset(); // reset device. 
      }      
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.print(" reconnect cnt: "); Serial.print(reconnect_cnt);
      Serial.println(" try again in 2 seconds");
      // Wait 2 seconds before retrying
      for (int dlay = 0; dlay < 100; dlay++) { 
        yield();
        delay(20); 
      }
   }
  }
}


void mydelay(unsigned long sleep)
{
//  Serial.print("Waiting time to xxx sec. "); Serial.println(sleep);
  sleep = 1000*sleep;  // convert to mili secs. 
  unsigned now = millis();  
  while(now + sleep > millis() ) {
    yield();
    delay (100);
    if (!client.connected()) 
       reconnect();
    client.loop();
  }
 
}
 
int idlecnt = 0;
 
void loop() {

  if (Serial.available() > 0 ) {
    if (Serial.readBytes(tmsg, 4) > 3) { 
       Serial.println((char*) tmsg);
       memcpy(&digits[1],tmsg,4);
       msg_cnt = 1;
       msg_ndx = 1;
  //     seg.displayChars(&digits[msg_ndx]);
       client.publish("davinci/", (const char*) tmsg);
       Serial.println("inside-monitor");
    }
  }
  mydelay(1);    //  was client.loop();

  if (snapshot) {
    snapshot = false;
    byte dummy = camera_get_data(NULL); 
    picClient.stop();
    camera_powerdown();
  }

  // Show once
  if (msg_cnt == 1)
  { 
    seg.displayChars(&digits[msg_ndx]);
    msg_cnt = 0;
 //   mydelay(2);
  }
  else if (msg_cnt > 1)
  { 
    seg.displayChars(&digits[msg_ndx]);
    msg_ndx += 4;   // advance to next block
    if (msg_ndx > (msg_cnt * 4)) 
      msg_ndx = 1;    // reset to first block
 //   mydelay(2);  // 
  //  Serial.print(msg_ndx);Serial.print(" msg cnt ");Serial.println(msg_cnt);
  }

  if (msg_cnt == -1)
  { 
   uchar idle[] = {0,0,0,0 };
 //   if ( (millis() % 2100) == 0)  <---- replaced by mydelay
    {  
      idle[idlecnt++] = '-';
      seg.displayChars(idle);
      idlecnt = idlecnt > 3 ? 0: idlecnt;
      mydelay(1); // need delay to avoid multiple runs 
    }
  }
}



bool FW_Update( char* updatemsg)
{
  int n; 
  char *pch = strtok(updatemsg, " ");  // Ignore cmd id
     if (pch != NULL) {                 // search for update server ip 
        pch = strtok(NULL, " ");        
        if (pch != NULL) { 
           if (strchr( (const char*) pch, '.') != NULL)  {  // check for valid ip address
            strcpy((char*)update_server, (const char*) pch);   // overrule default update server
           }
        }
        pch = strtok(NULL, ";");        
        if (pch != NULL) {               
              strcpy((char*) esp_binary, (const char*) pch);   // overrule default binary file
           }
     }
     seg.displayChars((unsigned char*)"uuuu");
     yield();
     t_httpUpdate_return ret = ESPhttpUpdate.update( update_server, 80, esp_binary); 
     Serial.print("Update: ");
     Serial.println(ret);
     switch(ret) {

       case 7: // HTTP_UPDATE_SKETCH_UPDFAILED:
          Serial.println("HTTP_UPDATE_FAILED  sketch update failed "); 
           seg.displayChars((unsigned char*)"u001");
           client.publish("RH-CTRL/",  "Update failed - sketch update problem");
          break;
          
       case 9: // HTTP_UPDATE_MEMORYTOOSMALL:
          Serial.println("HTTP_UPDATE_FAILED  lack of memory "); 
           seg.displayChars((unsigned char*)"u002");
           client.publish("RH-CTRL/",  "Update failed - Lack of memory");
          break;
      
       case HTTP_UPDATE_FAILED:
      //    Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
          Serial.println("HTTP_UPDATE_FAILED");
           seg.displayChars((unsigned char*)"u003");
          break;

      case HTTP_UPDATE_NO_UPDATES:
          Serial.println("HTTP_UPDATE_NO_UPDATES");
           seg.displayChars((unsigned char*)"u004");
          break;

      case HTTP_UPDATE_OK:
          Serial.println("HTTP_UPDATE_OK");
          break;
      }
           
}


// ov528 protocol.
#define OV528_BAUD 115200 // 57600 115200
#define OV528_PKT_SZ     512
#define OV528_SIZE        OV528_SIZE_QVGA  //OV528_SIZE_VGA 
#define OV528_ADDR       (0<<5)

#define ACK_TIMEOUT 500
#define RETRY_LIMIT 200
#define TIMEOUT 500

void writeBytes(uint8_t buf[], uint16_t len) {
  Serial.write(buf, len);
}

inline void recv_clear(void) {
  Serial.flush();
}

uint16_t readBytes(uint8_t buf[], uint16_t len, uint16_t timeout_ms) {
  uint16_t i;
  uint8_t subms = 0;
  for (i = 0; i < len; i++) {
    while (Serial.available() == 0) {
      delayMicroseconds(10);
      if (++subms >= 100) {
        if (timeout_ms == 0) {
          return i;
        }
        subms = 0;
        timeout_ms--;
      }
    }
    buf[i] = Serial.read();
  }
  return i;
}

uint8_t chkAck(uint8_t cmd) {
  uint8_t buf[6];
  if (readBytes(buf, 6, ACK_TIMEOUT) != 6) {
    return 0;
  }
  if (buf[0] == 0xaa && buf[1] == (OV528_CMD_ACK | OV528_ADDR) && buf[2] == cmd && buf[4] == 0 && buf[5] == 0) {
    return 1;
  }
  return 0;
}


inline uint8_t send_cmd_with_ack(uint8_t cmd, uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4) {
  while (1) {
    recv_clear();
    sendCmd(cmd, p1, p2, p3, p4);
    if (chkAck(cmd)) {
      break;
    }
  }
  return 1;
}


void sendCmd(uint8_t cmd, uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4) {
  uint8_t buf[6] = { 0xaa };
  buf[1] = cmd | OV528_ADDR;
  buf[2] = p1;
  buf[3] = p2;
  buf[4] = p3;
  buf[5] = p4;
  writeBytes(buf, 6);
}

bool camera_sync() {
  uint8_t resp[6];
  recv_clear();
  bool stat = false;
  byte n=0;
  while (n++ < 5) {
    sendCmd(OV528_CMD_SYNC, 0, 0, 0, 0);
    if (!chkAck(OV528_CMD_SYNC)) {
      continue;
    }
    if (readBytes(resp, 6, TIMEOUT) != 6) continue;
    if (resp[0] == 0xaa && resp[1] == (OV528_CMD_SYNC | OV528_ADDR) && resp[2] == 0 && resp[3] == 0 && resp[4] == 0 && resp[5] == 0) {
      stat  = true;
      break;
    }
  }
 // Serial.println("Camera Sync done");
  sendCmd(OV528_CMD_ACK, OV528_CMD_SYNC, 0, 0, 0);
  return stat;
}

void camera_init(void) {
  send_cmd_with_ack(OV528_CMD_INIT, 0, 7, 3, 7); // OV528_SIZE);
//   Serial.println("Camera init done");
}

void camera_powerdown(void) {
  send_cmd_with_ack(OV528_CMD_PW_OFF, 0, 0, 0, 0);
}


void camera_snapshot(void) {
  send_cmd_with_ack(OV528_CMD_PKT_SZ, 0x08, OV528_PKT_SZ & 0xff, (OV528_PKT_SZ >> 8) & 0xff , 0);
  send_cmd_with_ack(OV528_CMD_SNAPSHOT, 0, 0, 0, 0);
}

byte camera_get_data(void *ctx /* SIZE_FUN size_fun, WRITE_FUN write_fun */ ) {
  char pubid[40]; 
  unsigned long data_size;
  byte  buf1[20];
  byte buf[OV528_PKT_SZ];

  sprintf(pubid,"%ssnapshot/", rhid); 

  while (1) {
    send_cmd_with_ack(OV528_CMD_GET_PIC, OV528_PIC_TYPE_SNAPSHOT, 0, 0, 0);

    if (readBytes(buf1, 6, 1000) != 6) {
      continue;
    }
    if (buf1[0] == 0xaa && buf1[1] == (OV528_CMD_DATA | OV528_ADDR) && buf1[2] == 0x01) {
      data_size = (buf1[3]) | (buf1[4] << 8) | ((uint32_t)buf1[5] << 16);
      break;
    }
  }

   // send small header with  client id  & size   
   sprintf(  (char*)  buf1, "%s size:%d\n", rhid, data_size); 
//    client.publish((const char*) pubid, (const char*) buf1 );
   picClient.write((const char*) buf1, strlen((char*) buf1) );

  uint16_t num_packet = (data_size + (OV528_PKT_SZ - 6 - 1)) / (OV528_PKT_SZ - 6);


//  size_fun(ctx, data_size);

  for (uint16_t i = 0; i < num_packet; i++) {
    uint8_t retry_cnt = 0;

retry:
    recv_clear();
    sendCmd(OV528_CMD_ACK, 0, 0,  i & 0xff, (i >> 8) & 0xff);

    // recv data : 0xaa, OV528_CMD_DATA, len16, data..., sum?
    uint16_t len = readBytes(buf, OV528_PKT_SZ, OV528_PKT_SZ);

    // checksum
    uint8_t sum = 0;
    for (uint16_t y = 0; y < len - 2; y++) {
      sum += buf[y];
    }
  //    sprintf(  (char*)  buf1, "block size:%d\n", len-4); 
  //    client.publish((const char*) rhid, (const char*) buf1 );

    // Ignore first 5 bytes / it seems to be a counting camera header
   // client.publish(pubid, &buf[4], len-6);
     picClient.write( &buf[4],len-6);
    
  //  Serial.print("Len;");
  //  Serial.println(len, HEX);
    if (sum != buf[len - 2]) {
      if (++retry_cnt < RETRY_LIMIT) {
        delay(100);
        goto retry;
      } else {
        sendCmd(OV528_CMD_ACK, 0, 0, 0xf0, 0xf0);
        return 0;
      }
    }

    // Send image data.
    //  write_fun(ctx, buf+4, len-6);
  }
  sendCmd(OV528_CMD_ACK, 0, 0, 0xf0, 0xf0);
  return 1;
}






