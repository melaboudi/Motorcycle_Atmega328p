#include "arduino.h"
#include "LowPower.h"

#include "PinChangeInterrupt.h"
//#define intPin A4
#define intPin 8
volatile bool flag=false;
bool received=false;

#include "Adafruit_FRAM_I2C.h"
Adafruit_FRAM_I2C fram     = Adafruit_FRAM_I2C();
uint16_t          framAddr = 0;
unsigned long framWritePosition = 0;
unsigned long framWritePositionDebug = 32000;
int insertCounter = 0;
String fixStatus = " ";
String gpsTime = " ";
String latitude = " ";
String longitude = " ";
String course = " ";
String pdop = " ";
uint32_t payLoadSize = 0;
String used_satellites = " ";
String viewed_satellites = " ";
String lastUnixTime = " ";
char Buffer[10] = {0};
String speed = " ";
#include "timestamp32bits.h"
String imei = " ";
int fixState = 0;
int gnsState = 0;
uint16_t gnsFailCounter = 0;
unsigned long previousMillisGps = 0;
const long intervalGps = 200000;
unsigned long currentMillis = 0;
uint16_t gpsFailCounter = 0;
bool started = true;
bool restarted = false;
uint16_t httpActionFail = 0;
uint16_t FirstStartCounter = 0;
uint16_t ReStartCounter=0;
uint16_t httpPostFail = 0;
uint16_t httpPostSuccess = 0;
bool onOff = true;
unsigned long t1 = 0; //le temps du dernier point inséré
unsigned long t2 = 0; //le temps du dernier point capté
uint16_t ti = 6; //le temps entre chaque insertion
unsigned long t3 = 0; //le temps du dernier envoie
unsigned long te = 45; //le temps entre les envoies
unsigned long unixTimeInt = 0;
uint16_t SizeRec = 66;
bool wakeUp=true;
bool OkToSend = true;
uint16_t maxTime = 0;
uint16_t Size = 0;
String batLev=" ";
volatile uint16_t wakeUpCounter = 0;
char newC[3]={0};
//6  45----7points 9secondsSend  ok
//6  41----7points 10secondsSend
//6  39----6points 9secondsSend
//6  50----8points 10secondsSend
//6  36----6points 9secondsSend
//6  34----6points 8secondsSend
//6  29----5points 8.5secondsSent
//6  25----4Points 8secondsSend
uint8_t limitToSend = 5;
void IntRoutine(void);
bool httpPostAll();
bool httpPostLimited();
bool httpPostWakeUp();
bool httpPostSleeping();
void httpPost1P();
void getWriteFromFramFromZero(uint16_t p1, uint16_t p2);
void decrementCounter(uint16_t value);
bool turnOnGns();
uint8_t getGnsStat();
bool getGpsData();
void sendAtCom(char *AtCom);
String returnImei();
uint8_t getGsmStat();
String batteryLevel();
String rssiLevel();
bool gprsOn();
void gprsOff();
bool getGprsState();
void flushSim();
void writeDataFram(char* dataFram);
void writeDataFramDebug(char* dataFram, long p1);
void powerUp();
void powerDown();
void blinkLED(int k);
void blinkLEDFast(int k);
void simHardReset();
void clearMemory(int size);
void clearMemoryDiff(int size, int size1);
void clearMemoryDebug(unsigned long size);
void insertMem();
void incrementCounter();
String complete(String s, int t);
int getCounter();
int getValue(uint16_t position, uint8_t largeur);
void incrementValue(uint16_t position, uint8_t largeur);
bool sendAtFram(long timeout, uint16_t pos1, uint16_t pos2, char* Rep, char* Error, int nbRep);
bool fireHttpAction(long timeout, char* Commande, char* Rep, char* Error, int nbRep);
void getWriteFromFram(uint16_t p1, uint16_t p2);
void trace(unsigned long unixTime, uint8_t type);
void clearValue();
bool insertGpsData();
void resetSS();
void cfunReset();


void setup() {
  delay(100);
  fram.begin();
  pinMode(3, OUTPUT);//VIO
  pinMode(A3, INPUT);//sim Power Status
  pinMode(0, INPUT);//SS RX
  pinMode(1, OUTPUT);//SS TX
  pinMode(6, OUTPUT);//sim Reset
  digitalWrite(6, HIGH);
  digitalWrite(3, HIGH);
  powerUp();
  Serial.begin(4800);
  turnOnGns();
  imei = returnImei();
  while (getGsmStat() != 1) {
    delay(500);
  }
  gprsOn();
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(intPin), IntRoutine, RISING);
}

void loop() {
  enablePinChangeInterrupt(digitalPinToPinChangeInterrupt(intPin));
  if(wakeUp){httpPostWakeUp();wakeUp=false;}
  if (!digitalRead(8)) {
    if (getGnsStat() == 1) {
      if (getGpsData()) {
        if ((t2 - t1) >= ti) {insertMem();t1 = t2;}
        if ((t1 - t3) >= te) {
          if (getCounter() <= 10) {if(!httpPostAll()){resetSS();}} else {if(!httpPostLimited()){resetSS();}}
        }
      }else {resetSS();delay(10000);} 
  } else {turnOnGns(); delay(1000);}
  } else {
      if(getCounter()==0){httpPost1P();}else {if(!httpPostAll()){resetSS();}}
    httpPostSleeping();powerDown(); 
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(intPin), IntRoutine, RISING);
    Serial.flush();
    while (wakeUpCounter < 3) {
    Wire.beginTransmission(8);
    Wire.write('f');
    Wire.endTransmission();
      LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_ON, TWI_OFF);
      wakeUpCounter++;
    }
    if (wakeUpCounter != 5) {
    powerUp();turnOnGns(); gprsOn(); 
    wakeUpCounter = 0;httpPostWakeUp();
    httpPost1P();
    } 
  else {
    Wire.beginTransmission(8);
    Wire.write('n');
    Wire.endTransmission();
    powerUp();turnOnGns();gprsOn();
    wakeUpCounter = 0;httpPostWakeUp();
    httpPost1P();
  }
  }
}
void IntRoutine(void) {
   wakeUpCounter = 4;
  Serial.flush();
  detachPinChangeInterrupt(digitalPinToPinChangeInterrupt(intPin));
}
bool httpPostAll() {
  bool OkToSend = true;
  if (sendAtFram(3000, 31254, 13, "OK", "ERROR", 5)) { //"AT+HTTPINIT"
    if (sendAtFram(3000, 31267, 19, "OK", "ERROR", 5)) { //"AT+HTTPPARA=\"CID\",1"
      if (sendAtFram(10000, 31609, 73, "OK", "ERROR", 5)) { //"AT+HTTPPARA=\"URL\",\"http://casa-interface.casabaia.ma/commandes.php\""
        Serial.setTimeout(30000);
        flushSim();
        Serial.print("AT+HTTPDATA=");
        delay(100);
        uint16_t nb = getCounter();
    uint16_t Size = (nb * (SizeRec + 1)) + (nb * 8) - 1 + 2;
        Serial.print(Size);
        Serial.print(",");
        uint32_t maxTime = 30000;
        Serial.println(maxTime);
        Serial.findUntil("DOWNLOAD", "ERROR");
        Serial.print("[");
        for (uint16_t i = 0; i < nb ; i++)
        {
          for (uint16_t j = SizeRec * i; j < (SizeRec * (i + 1)) ; j++)
          {
            if (j == (i * SizeRec)) {
              Serial.print("{\"P\":\"");
              delay(1);
            }
            uint8_t test = fram.read8(j);
            sprintf(Buffer, "%c", test);
            Serial.write(Buffer);
            delay(1);
          }
          Serial.print("\"}");
          delay(1);
          if (i < nb - 1) {
            Serial.write(",");
            delay(1);
          }
        }
        Serial.print("]");
        Serial.findUntil("OK", "OK");
      } else OkToSend = false;
    } else OkToSend = false;
  } else OkToSend = false;
  if (OkToSend) {
    if (fireHttpAction(30000, "AT+HTTPACTION=", ",200,", "ERROR", 1)) {httpPostFail = 0;httpPostSuccess++;
    clearMemory(getCounter() * 66); clearMemoryDebug(32003); 
    t3 = t1; return true;
    } else {  return false;}
  }else return false;
}
bool httpPostLimited() {
  bool OkToSend = true;
  if (sendAtFram(3000, 31254, 13, "OK", "ERROR", 5)) { //"AT+HTTPINIT"
    if (sendAtFram(3000, 31267, 19, "OK", "ERROR", 5)) { //"AT+HTTPPARA=\"CID\",1"
      if (sendAtFram(10000, 31609, 73, "OK", "ERROR", 5)) { //"AT+HTTPPARA=\"URL\",\"http://casa-interface.casabaia.ma/commandes.php\""
        Serial.setTimeout(30000);
        flushSim();
        Serial.print("AT+HTTPDATA=");
        delay(100);
        uint16_t Size = (limitToSend * (SizeRec + 1)) + (limitToSend * 8) - 1 + 2;
        Serial.print(Size);
        Serial.print(",");
        uint32_t maxTime = 30000;
        Serial.println(maxTime);
        Serial.findUntil("DOWNLOAD", "ERROR");
        Serial.print("[");
        for (uint16_t i = 0; i < limitToSend ; i++)
        {
          for (uint16_t j = SizeRec * i; j < (SizeRec * (i + 1)) ; j++)
          {
            if (j == (i * SizeRec)) {
              Serial.print("{\"P\":\"");
              delay(1);
            }
            uint8_t test = fram.read8(j);
            sprintf(Buffer, "%c", test);
            Serial.write(Buffer);
            delay(1);
          }
          Serial.print("\"}");
          delay(1);
          if (i < limitToSend - 1) {
            Serial.write(",");
            delay(1);
          }
        }
        Serial.print("]");
        Serial.findUntil("OK", "OK");
      } else OkToSend = false;
    } else OkToSend = false;
  } else OkToSend = false;
  if (OkToSend) {
    if (fireHttpAction(30000, "AT+HTTPACTION=", ",200,", "ERROR", 1)) {
        decrementCounter(limitToSend);
        getWriteFromFramFromZero(limitToSend * 66, getCounter() * 66);
    return true;
    } else {return false;}
  }else return false;
}
bool httpPostWakeUp() {
  bool OkToSend = true;
  if (sendAtFram(3000, 31254, 13, "OK", "ERROR", 5)) { //"AT+HTTPINIT"
    if (sendAtFram(3000, 31267, 19, "OK", "ERROR", 5)) { //"AT+HTTPPARA=\"CID\",1"
      if (sendAtFram(10000, 31609, 73, "OK", "ERROR", 5)) { //"AT+HTTPPARA=\"URL\",\"http://casa-interface.casabaia.ma/commandes.php\""
        Serial.setTimeout(20000);
        flushSim();
        Serial.print("AT+HTTPDATA=");
        delay(100);
        uint16_t Size = 26;
        Serial.print(Size);
        Serial.print(",");
        uint32_t maxTime = 20000;
        Serial.println(maxTime);
        Serial.findUntil("DOWNLOAD", "ERROR");
        //String dataToSend="";
    //sprintf(dataToSend.c_str(), "[{\"S\":\"%c1\"]}", imei.c_str());
    Serial.print("[{\"S\":\"");
    Serial.print(imei.c_str());
    Serial.print("1\"}]");
        Serial.findUntil("OK", "OK");
      } else OkToSend = false;
    } else OkToSend = false;
  } else OkToSend = false;
  if (OkToSend) {
    if (fireHttpAction(60000, "AT+HTTPACTION=", ",200,", "ERROR", 1)) {
      return true;
    } else {return false;}
  }else return false;
}
bool httpPostSleeping() {
  bool OkToSend = true;
  if (sendAtFram(3000, 31254, 13, "OK", "ERROR", 5)) { //"AT+HTTPINIT"
    if (sendAtFram(3000, 31267, 19, "OK", "ERROR", 5)) { //"AT+HTTPPARA=\"CID\",1"
      if (sendAtFram(10000, 31609, 73, "OK", "ERROR", 5)) { //"AT+HTTPPARA=\"URL\",\"http://casa-interface.casabaia.ma/commandes.php\""
        Serial.setTimeout(20000);
        flushSim();
        Serial.print("AT+HTTPDATA=");
        delay(100);
        uint16_t Size = 26;
        Serial.print(Size);
        Serial.print(",");
        uint32_t maxTime = 20000;
        Serial.println(maxTime);
        Serial.findUntil("DOWNLOAD", "ERROR");
        //String dataToSend="";
    //sprintf(dataToSend.c_str(), "[{\"S\":\"%c0\"}]", imei.c_str());
    Serial.print("[{\"S\":\"");
    Serial.print(imei.c_str());
    Serial.print("0\"}]");
        Serial.findUntil("OK", "OK");
      } else OkToSend = false;
    } else OkToSend = false;
  } else OkToSend = false;
  if (OkToSend) {
    if (fireHttpAction(20000, "AT+HTTPACTION=", ",200,", "ERROR", 1)) {
      return true;
    } else {
      return false;
    }
  }else return false;
}
void httpPost1P() {
  sendAtFram(3000, 31241, 13, "OK", "ERROR", 5);
  batLev=batteryLevel();
    if (getGnsStat() == 1) {
    if (getGpsData()) {
    OkToSend = true;
    if (sendAtFram(3000, 31254, 13, "OK", "ERROR", 5)) { //"AT+HTTPINIT"
    if (sendAtFram(3000, 31267, 19, "OK", "ERROR", 5)) { //"AT+HTTPPARA=\"CID\",1"
      if (sendAtFram(10000, 31609, 73, "OK", "ERROR", 5)) { //"AT+HTTPPARA=\"URL\",\"http://casa-interface.casabaia.ma/commandes.php\""
    Serial.setTimeout(30000);
    flushSim();
    Serial.print("AT+HTTPDATA=");
    delay(100);
    Size = 76;
    Serial.print(Size);
    Serial.print(",");
    maxTime = 30000;
    Serial.println(maxTime);
    Serial.findUntil("DOWNLOAD", "ERROR");
        Serial.print("[{\"P\":\"");
        //Serial.print("869170031849640");
    Serial.print(imei.c_str());                      //15
        //Serial.print("1");
    Serial.print(fixStatus.c_str());                     //1
        //Serial.print("2222222222");
    Serial.print(latitude.c_str());                      //10
        //Serial.print("33333333333");
    Serial.print(longitude.c_str());                     //11
        //Serial.print("444444");
    Serial.print(speed.c_str());                       //6
        //Serial.print("55");
    Serial.print(used_satellites.c_str());               //2
        //Serial.print("666666");
    Serial.print(course.c_str());                      //6
        //Serial.print("777");
    Serial.print(batLev.c_str());                        //3
        //Serial.print("8888888888");
    Serial.print(lastUnixTime.c_str());
        
    Serial.print("00\"}]");
        Serial.findUntil("OK", "OK");
          } else OkToSend = false;
        } else OkToSend = false;
      } else OkToSend = false;
      if (OkToSend) {fireHttpAction(60000, "AT+HTTPACTION=", ",200,", "ERROR", 1);}
    }
  }
}
void getWriteFromFramFromZero(uint16_t p1, uint16_t p2) {
  framWritePosition = 0;
  for (uint16_t a = p1; a < p1 + p2; a++)
  {
    uint8_t test = fram.read8(a);
    char Buffer[2] = {0};
    sprintf(Buffer, "%c", test);
    writeDataFram(Buffer);
  }
}
void decrementCounter(uint16_t value) {
  int countVal = getCounter();
  countVal -= value;
  writeDataFramDebug(complete(String(countVal), 3).c_str(), 32000);
}

bool turnOnGns() {
  while (!getGnsStat()) {
    sendAtFram(3000, 31000, 12, "OK", "ERROR", 5); //"AT+CGNSPWR=1"
    delay(100);
  } return true;
}
uint8_t getGnsStat() {
  flushSim();
  Serial.setTimeout(100);
  Serial.println("AT+CGNSPWR?");//"AT+CGNSPWR?"
  if (Serial.findUntil("1", "0")) {
    return 1;
  } else return 0;
}
bool getGpsData() {
  fixStatus = gpsTime = latitude = longitude = used_satellites = viewed_satellites = speed = " ";
  Serial.setTimeout(2000);
  flushSim();
  char gpsData[120] = {0};
  Serial.println("AT+CGNSINF");
  Serial.readBytesUntil('O', gpsData, 119);

  String gpsdatastr = String(gpsData);

  uint8_t ind1 = gpsdatastr.indexOf(',');
  //  String mode = gpsdatastr.substring(0, ind1);

  uint8_t ind2 = gpsdatastr.indexOf(',', ind1 + 1);
  fixStatus = gpsdatastr.substring(ind1 + 1, ind2);
  fixStatus = fixStatus.substring(0, 1);

  uint8_t ind3 = gpsdatastr.indexOf(',', ind2 + 1);
  String utctime = gpsdatastr.substring(ind2 + 1, ind3);
  timestamp32bits stamp = timestamp32bits();

  unixTimeInt = stamp.timestamp(
                  (utctime.substring(2, 4)).toInt(),
                  (utctime.substring(4, 6)).toInt(),
                  (utctime.substring(6, 8)).toInt(),
                  (utctime.substring(8, 10)).toInt(),
                  (utctime.substring(10, 12)).toInt(),
                  (utctime.substring(12, 14)).toInt());
  lastUnixTime = String(unixTimeInt);
  lastUnixTime = lastUnixTime.substring(0, 10);

  unsigned long gpsTimeInt = unixTimeInt - 315961182 ; //315964782 - 3600
  gpsTime = String(gpsTimeInt);
  t2 = gpsTimeInt;

  uint8_t ind4 = gpsdatastr.indexOf(',', ind3 + 1);
  latitude = gpsdatastr.substring(ind3 + 1, ind4);
  //latitude = latitude.substring(0, 9);
  while (strlen(latitude.c_str()) < 10) {
    latitude += '0';
  }

  uint8_t ind5 = gpsdatastr.indexOf(',', ind4 + 1);
  longitude = gpsdatastr.substring(ind4 + 1, ind5);
  while (strlen(longitude.c_str()) < 11) {
    longitude += '0';
  }
  uint8_t ind6 = gpsdatastr.indexOf(',', ind5 + 1);
  //  String altitude = gpsdatastr.substring(ind5 + 1, ind6);

  uint8_t ind7 = gpsdatastr.indexOf(',', ind6 + 1);
  speed = gpsdatastr.substring(ind6 + 1, ind7);
  speed = speed.substring(0, 6);
  while (strlen(speed.c_str()) < 6) {
    speed = '0' + speed;
  }

  uint8_t ind8 = gpsdatastr.indexOf(',', ind7 + 1);
  course = gpsdatastr.substring(ind7 + 1, ind8);
  course = course.substring(0, 6);
  while (strlen(course.c_str()) < 6) {
    course = '0' + course;
  }

  uint8_t ind9 = gpsdatastr.indexOf(',', ind8 + 1);
  //  String fixmode = gpsdatastr.substring(ind8 + 1, ind9);
  uint8_t ind10 = gpsdatastr.indexOf(',', ind9 + 1);
  //  String reserved1 = gpsdatastr.substring(ind9 + 1, ind10);
  uint8_t ind11 = gpsdatastr.indexOf(',', ind10 + 1);
  //  String HDOP = gpsdatastr.substring(ind10 + 1, ind11);

  uint8_t ind12 = gpsdatastr.indexOf(',', ind11 + 1);
  pdop = gpsdatastr.substring(ind11 + 1, ind12);
  pdop = pdop.substring(0, 4);
  while (strlen(pdop.c_str()) < 4) {
    pdop = '0' + pdop;
  }

  uint8_t ind13 = gpsdatastr.indexOf(',', ind12 + 1);
  //  String VDOP = gpsdatastr.substring(ind12 + 1, ind13);
  uint8_t ind14 = gpsdatastr.indexOf(',', ind13 + 1);
  //  String reserved2 = gpsdatastr.substring(ind13 + 1, ind14);
  uint8_t ind15 = gpsdatastr.indexOf(',', ind14 + 1);
  String viewed_satellites = gpsdatastr.substring(ind14 + 1, ind15);
  while (strlen(viewed_satellites.c_str()) < 2) {
    viewed_satellites = '0' + viewed_satellites;
  }
  uint8_t ind16 = gpsdatastr.indexOf(',', ind15 + 1);

  used_satellites = gpsdatastr.substring(ind15 + 1, ind16);
  used_satellites = used_satellites.substring(0, 2);
  while (strlen(used_satellites.c_str()) < 2) {
    used_satellites = '0' + used_satellites;
  }

  //  uint8_t ind17 = gpsdatastr.indexOf(',', ind16 + 1);
  ////  String reserved3 = gpsdatastr.substring(ind16 + 1, ind17);
  //  uint8_t ind18 = gpsdatastr.indexOf(',', ind17 + 1);
  ////  String N0max = gpsdatastr.substring(ind17 + 1, ind18);
  //  uint8_t ind19 = gpsdatastr.indexOf(',', ind18 + 1);
  ////  String HPA = gpsdatastr.substring(ind18 + 1, ind19);
  //  uint8_t ind20 = gpsdatastr.indexOf(',', ind19 + 1);
  ////  String VPA = gpsdatastr.substring(ind19);
  //////////////////////////////////////////////////////////////////
  if (onOff) {
    trace(unixTimeInt, 1);
    onOff = false;
  }
  if ((fixStatus.toInt() == 1) && (latitude.toInt() != 0) && (longitude.toInt() != 0)) {
    started = false;
  restarted=false;
    return true;
  } else return false;
}
void sendAtCom(char *AtCom) {
  flushSim();
  Serial.println(AtCom);
  String tempGSM = Serial.readString();
}
String returnImei() {
  flushSim();
  Serial.println("AT+GSN");
  String tempGSM = Serial.readString();
  uint8_t counter = 0;
  while (!strstr(tempGSM.c_str(), "OK")) {
    tempGSM = " ";
    flushSim();
    Serial.println("AT+GSN");
    tempGSM = Serial.readString();
    delay(2000);
    counter++;
    if (counter >= 6) {
      counter = 0;
      simHardReset();
    }
  }
  String imei1 = strstr(tempGSM.c_str(), "8");
  String imei2 = imei1.substring(0, 15);
  return imei2;
  // String imeiFake = "869170031699599";
  //return imeiFake;
}
uint8_t getGsmStat() {
  flushSim();
  Serial.println("AT+CREG?");
  String tempGSM = Serial.readString();
  int ind1 = tempGSM.indexOf(',');
  String gsmStat = tempGSM.substring(ind1 + 1, ind1 + 2);
  return gsmStat.toInt();
}
String batteryLevel() {
  flushSim();
  Serial.println("AT+CBC");
  String tempGSM = Serial.readString();
  int ind1 = tempGSM.indexOf(',');
  int ind2 = tempGSM.indexOf(',', ind1 + 1);
  //String chargeState = tempGSM.substring( 1, ind1 + 1);
  String batteryLevel = tempGSM.substring(ind1 + 1, ind2);

  uint16_t vbat = batteryLevel.toInt();

  if (vbat >= 100) {
    sprintf(batteryLevel.c_str(), "%d", vbat);
  }
  else if ((vbat < 100) && (vbat > 9)) {
    sprintf(batteryLevel.c_str(), "0%d", vbat);
  } else if (vbat < 10) {
    char charbat[3] = {0};
    itoa(vbat, charbat, 10);
    sprintf(batteryLevel.c_str(), "00%d", vbat);
  } else {
    strcpy(batteryLevel.c_str(), "000");
  }

  return batteryLevel;
}
String rssiLevel() {
  flushSim();
  Serial.println("AT+CSQ");
  String tempGSM = Serial.readString();
  int ind1 = tempGSM.indexOf(':');
  int ind2 = tempGSM.indexOf(',');
  String rssiLevel = tempGSM.substring(ind1 + 1, ind2);
  uint16_t n = rssiLevel.toInt();
  int r;
  if (n == 0) r = -115;
  if (n == 1) r = -111;
  if (n == 31) r = -52;
  if ((n >= 2) && (n <= 30)) {
    r = map(n, 2, 30, -110, -54);
  }
  int p = abs(r);
  char charRSSI[8] = {0};
  if (p >= 100) {
    sprintf(charRSSI, "-%d", p);
  } else sprintf(charRSSI, "-0%d", p);
  return String(charRSSI);
}
bool gprsOn() {
  sendAtFram(5000, 31140, 9, "OK", "ERROR", 5); //"AT+CFUN=1"
  sendAtFram(5000, 31184, 29, "OK", "OK", 5); //"AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""
  if (sendAtFram(5000, 31213, 12, "OK", "OK", 5)) { //"AT+SAPBR=1,1"
    if (sendAtFram(5000, 31225, 8, "OK", "ERROR", 5)) { //"AT+CIICR"
      if (sendAtFram(5000, 31233, 8, ">", "ERROR", 5)) { //"AT+CIFSR"
        return true;
      } else return false;
    } else return false;
  } else return false;
}
void gprsOff() {
  //if(getGprsState())
  { sendAtFram(5000, 31599, 10, "OK", "ERROR", 5); //"AT+CGATT=0"
  }
}
bool getGprsState() {//very slow!!
  Serial.setTimeout(10000);
  flushSim();
  Serial.write("AT+CGATT?");
  //Serial.println("");
  if (Serial.findUntil("1", "OK")) {
    return true;
  } else return false;
}
void flushSim() {
  uint16_t timeoutloop = 0;
  while (timeoutloop++ < 40) {
    while (Serial.available()) {
      Serial.read();
      timeoutloop = 0;  // If char was received reset the timer
    }
    delay(1);
  }
}
void writeDataFram(char* dataFram) {
  uint8_t dataFramSize = strlen(dataFram);
  for (unsigned long i = framWritePosition; i <= (dataFramSize + framWritePosition); i++)
  {
    fram.write8(i, dataFram[(i - framWritePosition)]);
  } framWritePosition += (dataFramSize) ;
}
void writeDataFramDebug(char* dataFram, long p1) {
  //for (unsigned long i = p1; i <= (p1 + strlen(dataFram)); i++)
  for (unsigned long i = p1; i < (p1 + strlen(dataFram)); i++)
  {
    fram.write8(i, dataFram[(i - p1)]);
  }
}
void powerUp() {
  while (analogRead(A3) < 200) {
    pinMode(5, OUTPUT);//PWR KEY
    digitalWrite(5, LOW);
    delay(2000);
    pinMode(5, INPUT_PULLUP);
    delay(100);
  }
}
void powerDown() {
  while (analogRead(A3) > 200) {
    pinMode(5, OUTPUT);//PWR KEY
    digitalWrite(5, LOW);
    delay(2000);
    pinMode(5, INPUT_PULLUP);
    delay(100);
  }
}
void blinkLED(int k) {
  for (int i = 0; i < k; i++) {
    digitalWrite(A0, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                       // wait for a second
    digitalWrite(A0, LOW);    // turn the LED off by making the voltage LOW
    delay(100);
  }
}
void blinkLEDFast(int k) {
  for (int i = 0; i < k; i++) {
    digitalWrite(A0, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(20);                       // wait for a second
    digitalWrite(A0, LOW);    // turn the LED off by making the voltage LOW
    delay(20);
  }
}
void simHardReset() {
  digitalWrite(6, HIGH);
  delay(10);
  digitalWrite(6, LOW);
  delay(200);
  digitalWrite(6, HIGH);
  powerUp();
  Serial.begin(4800);
}
void clearMemory(int size) {
  for (uint16_t a = 0; a < size; a++) {
    fram.write8(a, 0);
  }
  framWritePosition = 0;
}
void clearMemoryDiff(int size, int size1) {
  for (uint16_t a = size; a < size1; a++) {
    fram.write8(a, "0");
  }
}
void clearMemoryDebug(unsigned long size) {
  for (uint16_t a = 32000; a < size; a++) {
    fram.write8(a, "0");
  }
}
void insertMem() {
  framWritePosition = getCounter() * SizeRec;
  //getWriteFromFram(31041,13); //"<Track Imei=\""
  writeDataFram(imei.c_str());                    //15
  //getWriteFromFram(31054,26); //"\" Fc=\"WGS84\" FixPosition=\""
  writeDataFram(fixStatus.c_str());                 //1
  //getWriteFromFram(31080,7); //"\" Lat=\""
  writeDataFram(latitude.c_str());                  //10
  //getWriteFromFram(31087,7); //"\" Lon=\""
  writeDataFram(longitude.c_str());                 //11
  //getWriteFromFram(31094,7); //"\" Vit=\""
  writeDataFram(speed.c_str());                   //6
  //getWriteFromFram(31101,7); //"\" Sat=\""
  writeDataFram(used_satellites.c_str());             //2
  //getWriteFromFram(31108,7); //"\" Cap=\""
  writeDataFram(course.c_str());                  //6
  //getWriteFromFram(31115,16); //"\" BatteryLevel=\""
  writeDataFram(batteryLevel().c_str());              //3
  //getWriteFromFram(31131,6); //"\" Dh=\""
  writeDataFram(lastUnixTime.c_str());                  //10
  //getWriteFromFram(31137,3); //"\"/>"
  //writeDataFram("00");
  /* Wire.requestFrom(8, 2);
  char reception[3]={0};
  if(Wire.available()){
    Wire.readBytes(reception,2);
    writeDataFram(reception);
  }else writeDataFram("00"); */
  Wire.requestFrom(8, 4);
 byte lb1; byte hb1; byte lb2; byte hb2;
 while (Wire.available()){lb1=Wire.read();hb1=Wire.read();lb2=Wire.read();hb2=Wire.read();received=true;}
  if(received){
    char str1[3];sprintf(str1, "%d", word(hb1,lb1));
    char str2[3];sprintf(str2, "%d", word(hb2,lb2));
    writeDataFram(str1);
    writeDataFram(str2);
    received=false;
  }else writeDataFram("00");
  Wire.beginTransmission(8);
  Wire.write('r');
  Wire.endTransmission();
  incrementCounter();
}
void incrementCounter() {
  int countVal = getCounter();
  countVal++;
  writeDataFramDebug(complete(String(countVal), 3).c_str(), 32000);
}
String complete(String s, int t) {
  while (strlen(s.c_str()) < t) {
    s = '0' + s;
  }
  return s;
}
int getCounter() {
  String retour = " ";
  for (long i = 32000; i <= 32002; i++) {
    char Buffer[5] = {0};
    uint8_t test = fram.read8(i);
    sprintf(Buffer, "%c", test);
    retour += Buffer;
  }
  return retour.toInt();
}
int getValue(uint16_t position, uint8_t largeur) {
  String retour = " ";
  for (long i = position; i < position + largeur; i++) {
    char Buffer[2] = {0};
    uint8_t test = fram.read8(i);
    sprintf(Buffer, "%c", test);
    retour += Buffer;
  }
  return retour.toInt();
}
void incrementValue(uint16_t position, uint8_t largeur) {
  int countVal = getValue(position, largeur);
  countVal++;
  writeDataFramDebug(complete(String(countVal), largeur).c_str(), position);
}
void resetSS() {
  cfunReset();
  turnOnGns();
  while (getGsmStat() != 1) {delay(500);}
  gprsOn();
  restarted=true;
  gnsFailCounter = 0;
  gpsFailCounter = 0;
  httpActionFail = 0;
  FirstStartCounter = 0;
  httpPostFail = 0;
  ReStartCounter=0;
}
void cfunReset(){
  sendAtFram(6000, 31730, 9, "OK", "ERROR", 1);  //CFUN=0
  sendAtFram(6000, 31140, 9, "OK", "ERROR", 1);  //CFUN=1
}
bool sendAtFram(long timeout, uint16_t pos1, uint16_t pos2, char* Rep, char* Error, int nbRep) {
  flushSim();
  Serial.setTimeout(timeout);
  for (uint16_t a = pos1; a < pos1 + pos2; a++)
  {
    uint8_t test = fram.read8(a);
    char Buffer[2] = {0};
    sprintf(Buffer, "%c", test);
    Serial.print(Buffer);
  } Serial.println("");

  int compteur = 0;
  while ((!Serial.findUntil(Rep, Error)) && (compteur < nbRep)) {
    flushSim();
    for (uint16_t a = pos1; a < pos1 + pos2; a++)
    {
      uint8_t test = fram.read8(a);
      char Buffer[2] = {0};
      sprintf(Buffer, "%c", test);
      Serial.print(Buffer);
    } Serial.println("");
    compteur++;
    delay(50);
  }
  if (compteur <= nbRep)
  {
    return true;
  } else
  {
    return false;
  }
  Serial.setTimeout(1000);
}
bool fireHttpAction(long timeout, char* Commande, char* Rep, char* Error, int nbRep) {
  flushSim();
  Serial.setTimeout(timeout);
  Serial.print(Commande);
  Serial.println(1, DEC);
  int compteur = 0;
  while ((!Serial.findUntil(Rep, Error)) && (compteur < nbRep)) {
    flushSim();
    Serial.print(Commande);
    Serial.println(1, DEC);
    compteur++;
    delay(50);
  }
  if (compteur < nbRep)
  {
    return true;
  } else
  {
    return false;
  }
  Serial.setTimeout(1000);
}
void getWriteFromFram(uint16_t p1, uint16_t p2) {
  for (uint16_t a = p1; a < p1 + p2; a++)
  {
    uint8_t test = fram.read8(a);
    char Buffer[2] = {0};
    sprintf(Buffer, "%c", test);
    writeDataFram(Buffer);
  }
}
void trace(unsigned long unixTime, uint8_t type) {
  uint8_t jour = (int)(((unixTime / 86400) + 4) % 7) + 1; //1 dimanche
  uint16_t positionEcriture = 32000 + jour * 10;
  if (jour == 2) {
    clearValue();
  }
  switch (type) {
    case 1:
      incrementValue(positionEcriture, 2);
      break;
    case 2:
      incrementValue(positionEcriture + 3, 3);
      break;
    case 3:
      incrementValue(positionEcriture + 6, 3);
      break;
  }
}
void clearValue() {
  if (getValue(32010, 2) > 0 ) {
    clearMemoryDiff(32010, 32080);
  }
}
bool insertGpsData() {
  if (getGpsData()) {
    gpsFailCounter = 0;
    insertMem();
    t1 = t2;
    return true;
  } else return false;
}
