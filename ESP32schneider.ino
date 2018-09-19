#include <ModbusMaster.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <PubSubClient.h>


#define MAX485_DE 19 // was 5(moteino) //and was 2(d4) on esp8266 but have trouble
#define MAX485_RE_NEG 19 // was 6
int wifiSetupCount = 0;
const char* ssid     = "-----";
const char* password = "--------";

const char* broker = "--------";

ModbusMaster node;

WiFiClient espClient;
PubSubClient client(espClient);

HardwareSerial mySerial(1);
//HardwareSerial SerialAT(1);

float Voltage1;
float Voltage2;
float Voltage3;

float Current1;
float Current2;
float Current3;
float averagePhaseCurrent;
float totalPower;
float energy;
float totalFactor;

int dataCount = 0;

float sumVoltage1 = 0;
float sumVoltage2 = 0;
float sumVoltage3 = 0;
float sumCurrent = 0;
float sumCurrent1 = 0;
float sumCurrent2 = 0;
float sumCurrent3 = 0;
float sumPower = 0;
float sumEnergy = 0;
float sumFrequency = 0;
float sumFactor = 0;

float maxVoltage1 = 0;
float maxVoltage2 = 0;
float maxVoltage3 = 0;
float maxCurrent = 0;
float maxCurrent1 = 0;
float maxCurrent2 = 0;
float maxCurrent3 = 0;
float maxPower = 0;
float maxEnergy = 0;
float maxFrequency = 0;
float maxFactor = 0;

float minVoltage1 = 0;
float minVoltage2 = 0;
float minVoltage3 = 0;
float minCurrent = 0;
float minCurrent1 = 0;
float minCurrent2 = 0;
float minCurrent3 = 0;
float minPower = 0;
float minEnergy = 0;
float minFrequency = 0;
float minFactor = 0;
unsigned long lastTime = 0;
int maxminFlag = 0;

void preTransmission()
{

  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  delay(3);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

float f_2uint_float(unsigned int uint1, unsigned int uint2)
{ // reconstruct the float from 2 unsigned integers

  union f_2uint {
    float f;
    uint16_t i[2];
  };

  union f_2uint f_number;
  f_number.i[0] = uint1;
  f_number.i[1] = uint2;

  return f_number.f;
}


float current_float(unsigned int uint1, unsigned int uint2)
{ // reconstruct the float from 2 unsigned integers

  union f_3uint {
    float f;
    uint16_t i[2];
  };

  union f_3uint f_number;
  f_number.i[0] = uint1;
  f_number.i[1] = uint2;

  return f_number.f;
}

void setup()
{
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  Serial.begin(9600);
  mySerial.begin(9600, SERIAL_8N1, 4, 15);
  // SerialAT.begin(115200,SERIAL_8N1,16,17,false);
  WiFi.disconnect();
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting");
        Serial.print("Count:");
        Serial.println(wifiSetupCount);
        if(wifiSetupCount > 20)
        {
          wifiSetupCount = 0;
            WiFi.disconnect();
            WiFi.begin(ssid, password);
            Serial.println("Reconnecting");
        }
        wifiSetupCount++;
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  node.begin(3, mySerial);

  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  client.setServer(broker, 1883);
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    if (client.connect("--------", "-------", NULL)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      // client.publish("v1/devices/me/telemetry");
      // ... and resubscribe
      //client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(2000);
    }
  }
}


void loop()
{
  if ((WiFi.status() != WL_CONNECTED)) {
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    delay(5000);
   
  }else{
    
     if (!client.connected()) {
       reconnect();
    }
       client.loop();
    
  }

  uint8_t result;


  result = node.readHoldingRegisters(0x0A8B, 2);

  Serial.println(result);
  if (result == node.ku8MBSuccess)
  {
    //  Serial.print("Energy:");



    energy = f_2uint_float(node.getResponseBuffer(0x01), node.getResponseBuffer(0x00));

    //Serial.println(energy);
  }

  delay(10);



  result = node.readHoldingRegisters(0x0BB7, 6);

  //Serial.println(result);
  if (result == node.ku8MBSuccess)
  {


    Current1 = f_2uint_float(node.getResponseBuffer(0x01), node.getResponseBuffer(0x00));
    Current2 = f_2uint_float(node.getResponseBuffer(0x03), node.getResponseBuffer(0x02));
    Current3 = f_2uint_float(node.getResponseBuffer(0x05), node.getResponseBuffer(0x04));
    Serial.println(Current1);
    Serial.println(Current2);
    Serial.println(Current3);



  }

  delay(10);




  result = node.readHoldingRegisters(0x0BD3, 6);

  //Serial.println(result);
  if (result == node.ku8MBSuccess)
  {

    //Serial.print("averagePhaseVoltage:");
    //  averagePhaseVoltage = f_2uint_float(node.getResponseBuffer(0x01), node.getResponseBuffer(0x00));
    Voltage1 = f_2uint_float(node.getResponseBuffer(0x01), node.getResponseBuffer(0x00));
    Voltage2 = f_2uint_float(node.getResponseBuffer(0x03), node.getResponseBuffer(0x02));
    Voltage3 = f_2uint_float(node.getResponseBuffer(0x05), node.getResponseBuffer(0x04));
    //Serial.println(averagePhaseVoltage);
    Serial.println(Voltage1);
    Serial.println(Voltage2);
    Serial.println(Voltage3);

  }

  delay(10);


  result = node.readHoldingRegisters(0x0BF3, 26);

  //Serial.println(result);
  if (result == node.ku8MBSuccess)
  {
    //Serial.print("TotalPower:");
    totalPower = f_2uint_float(node.getResponseBuffer(0x01), node.getResponseBuffer(0x00));
    //Serial.println(totalPower);


  }

  delay(10);


  result = node.readHoldingRegisters(0x0C77, 2);

  //Serial.println(result);
  if (result == node.ku8MBSuccess)
  {
    //Serial.print("totalFactor:");



    totalFactor = f_2uint_float(node.getResponseBuffer(0x01), node.getResponseBuffer(0x00));

    Serial.println(totalFactor);
  }

  sumVoltage1 += Voltage1;
  sumVoltage2 += Voltage2;
  sumVoltage3 += Voltage3;
  sumCurrent1 += Current1;
  sumCurrent2 += Current2;
  sumCurrent3 += Current3;
  sumPower += totalPower * 1000;

  //sumFactor +=  totalFactor;

  if (totalFactor == 1 || isnan(totalFactor)) {
    Serial.println("No load");
    totalFactor = 0;
    sumFactor +=  totalFactor;
    if (totalFactor >= maxFactor) {
      maxFactor = totalFactor;
    }

    if (totalFactor <= minFactor) {
      minFactor = totalFactor;
    }

  } else {
    sumFactor +=  totalFactor;
    if (totalFactor >= maxFactor) {
      maxFactor = totalFactor;
    }

    if (totalFactor <= minFactor) {
      minFactor = totalFactor;
    }

  }



  if (maxminFlag < 1) {
    minVoltage1 = Voltage1;
    minVoltage2 = Voltage2;
    minVoltage3 = Voltage3;
    minCurrent1 = Current1;
    minCurrent2 = Current2;
    minCurrent3 = Current3;
    minPower = totalPower;
    minEnergy = energy;
    minFactor = totalFactor;


    maxVoltage1 = Voltage1;
    maxVoltage2 = Voltage2;
    maxVoltage3 = Voltage3;
    maxCurrent1 = Current1;
    maxCurrent2 = Current2;
    maxCurrent3 = Current3;
    maxPower = totalPower;
    maxEnergy = energy;
    maxFactor = totalFactor;

    maxminFlag = 1;
  }

  if (Voltage1 >= maxVoltage1) {
    maxVoltage1 = Voltage1 ;
  }
  if (Voltage2 >= maxVoltage2) {
    maxVoltage2 = Voltage2 ;
  }

  if (Voltage3 >= maxVoltage3) {
    maxVoltage3 = Voltage3 ;
  }


  if (Current1 >= maxCurrent1) {
    maxCurrent1 = Current1;
  }
  if (Current2 >= maxCurrent2) {
    maxCurrent2 = Current2;
  }
  if (Current3 >= maxCurrent3) {
    maxCurrent3 = Current3;
  }


  if (totalPower >= maxPower) {
    maxPower = totalPower * 1000;
  }

  if (energy >= maxEnergy) {
    maxEnergy = energy ;
  }
  /*
       if(totalFactor >= maxFactor){
          maxFactor = totalFactor;
       }
  */
  if (Voltage1 <= minVoltage1) {
    minVoltage1 = Voltage1;
  }

  if (Voltage2 <= minVoltage2) {
    minVoltage2 = Voltage2;
  }

  if (Voltage3 <= minVoltage3) {
    minVoltage3 = Voltage3;
  }

  if (Current1 <= minCurrent1) {
    minCurrent1 = Current1;
  }

  if (Current2 <= minCurrent2) {
    minCurrent2 = Current2;
  }

  if (Current3 <= minCurrent3) {
    minCurrent3 = Current3;
  }

  if (totalPower <= minPower) {
    minPower = totalPower * 1000;
  }

  if (energy <= minEnergy) {
    minEnergy = energy ;
  }
  /*
       if(totalFactor <= minFactor){
          minFactor = totalFactor;
       }
  */
  dataCount++;

  unsigned long currentTime = millis();
  Serial.print("DataCount: ");
  Serial.println(dataCount);
  Serial.print("Time: ");
  Serial.println((currentTime - lastTime) / 1000);

  if (((currentTime - lastTime) / 1000) > 60) {
    float postVoltage1 = (sumVoltage1 / dataCount);
    float postVoltage2 = (sumVoltage2 / dataCount);
    float postVoltage3 = (sumVoltage3 / dataCount);
    float postCurrent1 = (sumCurrent1 / dataCount);
    float postCurrent2 = (sumCurrent2 / dataCount);
    float postCurrent3 = (sumCurrent3 / dataCount);
    float postPower = (sumPower / dataCount);
    float postEnergy = energy;
    float postFactor = (sumFactor / dataCount);


    Serial.print("Average Voltage1: ");
    Serial.println(postVoltage1);
    Serial.print("Average Voltage2: ");
    Serial.println(postVoltage2);
    Serial.print("Average Voltage3: ");
    Serial.println(postVoltage3);
    Serial.print("Average Current1: ");
    Serial.println(postCurrent1);
    Serial.print("Average Current2: ");
    Serial.println(postCurrent2);
    Serial.print("Average Current3: ");
    Serial.println(postCurrent3);
    Serial.print("Average Power: ");
    Serial.println(postPower);
    Serial.print("Average Energy: ");
    Serial.println(postEnergy);
    Serial.print("Average Factor: ");
    Serial.println(postFactor);


    Serial.print("Max Voltage1: ");
    Serial.println(maxVoltage1);
    Serial.print("Max Voltage2: ");
    Serial.println(maxVoltage2);
    Serial.print("Max Voltage3: ");
    Serial.println(maxVoltage3);
    Serial.print("Max Current1: ");
    Serial.println(maxCurrent1);
    Serial.print("Max Current2: ");
    Serial.println(maxCurrent2);
    Serial.print("Max Current3: ");
    Serial.println(maxCurrent3);
    Serial.print("Max Power: ");
    Serial.println(maxPower);
    Serial.print("Max Energy: ");
    Serial.println(maxEnergy);
    Serial.print("Max Factor: ");
    Serial.println(maxFactor);


    Serial.print("Min Voltage1: ");
    Serial.println(minVoltage1);
    Serial.print("Min Voltage2: ");
    Serial.println(minVoltage2);
    Serial.print("Min Voltage3: ");
    Serial.println(minVoltage3);
    Serial.print("Min Current1: ");
    Serial.println(minCurrent1);
    Serial.print("Min Current2: ");
    Serial.println(minCurrent2);
    Serial.print("Min Current3: ");
    Serial.println(minCurrent3);
    Serial.print("Min Power: ");
    Serial.println(minPower);
    Serial.print("Min Energy: ");
    Serial.println(minEnergy);
    Serial.print("Min Factor: ");
    Serial.println(minFactor);



    String payload = "{";
    payload += "\"voltage1_min\":";
    payload += (minVoltage1);
    payload += ",";
    payload += "\"voltage1\":";
    payload += postVoltage1;
    payload += ",";
    payload += "\"voltage1_max\":";
    payload += (maxVoltage1);
    payload += ",";
    payload += "\"voltage2_min\":";
    payload += (minVoltage2);
    payload += ",";
    payload += "\"voltage2\":";
    payload += postVoltage2;
    payload += ",";
    payload += "\"voltag2_max\":";
    payload += (maxVoltage2);
    payload += ",";
    payload += "\"voltage3_min\":";
    payload += (minVoltage3);
    payload += ",";
    payload += "\"voltage3\":";
    payload += postVoltage3;
    payload += ",";
    payload += "\"voltage3_max\":";
    payload += (maxVoltage3);
    payload += ",";
    payload += "\"current1_min\":";
    payload += (minCurrent1);
    payload += ",";
    payload += "\"current1\":";
    payload += postCurrent1;
    payload += ",";
    payload += "\"current1_max\":";
    payload += (maxCurrent1);
    payload += ",";
    payload += "\"current2_min\":";
    payload += (minCurrent2);
    payload += ",";
    payload += "\"current2\":";
    payload += postCurrent2;
    payload += ",";
    payload += "\"current2_max\":";
    payload += (maxCurrent2);
    payload += ",";
    payload += "\"current3_min\":";
    payload += (minCurrent3);
    payload += ",";
    payload += "\"current3\":";
    payload += postCurrent3;
    payload += ",";
    payload += "\"current3_max\":";
    payload += (maxCurrent3);
    payload += ",";
    payload += "\"power_min\":";
    payload += (minPower);
    payload += ",";
    payload += "\"power\":";
    payload += postPower;
    payload += ",";
    payload += "\"power_max\":";
    payload += (maxPower);
    payload += ",";
    payload += "\"energy\":";
    payload += postEnergy;
    payload += ",";
    payload += "\"powerFactor_min\":";
    payload += (minFactor);
    payload += ",";
    payload += "\"powerFactor\":";
    payload += postFactor;
    payload += ",";
    payload += "\"powerFactor_max\":";
    payload += (maxFactor);
    payload += "}";
    char attributes[900];

    payload.toCharArray( attributes, 900 );
    client.publish( "v1/devices/me/telemetry", attributes );


    Serial.println("resetting data");


    dataCount = 0;
    lastTime = millis();
    sumVoltage1 = 0;
    sumVoltage2 = 0;
    sumVoltage3 = 0;
    sumCurrent1 = 0;
    sumCurrent2 = 0;
    sumCurrent3 = 0;
    sumPower = 0;
    sumEnergy = 0;
    sumFrequency = 0;
    sumFactor = 0;

    maxVoltage1 = 0;
    maxVoltage2 = 0;
    maxVoltage3 = 0;
    maxCurrent1 = 0;
    maxCurrent2 = 0;
    maxCurrent3 = 0;
    maxPower = 0;
    maxEnergy = 0;
    maxFrequency = 0;
    maxFactor = 0;

    minVoltage1 = 0;
    minVoltage2 = 0;
    minVoltage3 = 0;
    minCurrent1 = 0;
    minCurrent2 = 0;
    minCurrent3 = 0;
    minPower = 0;
    minEnergy = 0;
    minFrequency = 0;
    minFactor = 0;

    maxminFlag = 0;
  }


  delay(2000);






}
