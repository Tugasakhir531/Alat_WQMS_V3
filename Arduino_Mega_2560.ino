#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


// Data wire is conntec to the Arduino digital pin 4
#define ONE_WIRE_BUS 4

#define TdsSensorPin A1
#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point


// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

LiquidCrystal_I2C lcd(0x27, 20, 4);


int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,temperature = 25;
int tdsValue = 0;

// ph 
const int ph_Pin = A0;
float Po;
float PH_step;
int nilai_analog_PH;
double TeganganPh;

float PHEmpat = 1.73;
float PHTujuh = 1.93;


float volt;
int ntu;
float voltageKekeruhan; 

float suhu;
float ph;
int tds;
int kekeruhanVal;

int kekeruhan;


// ph 
#define SensorPin A0            //pH meter Analog output to Arduino Analog Input 0
#define Offset 0.00            //deviation compensate
#define LED 13
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;


void setup() {
  Serial.begin(9600);
  sensors.begin();
  pinMode(TdsSensorPin,INPUT);
  pinMode (ph_Pin, INPUT);
  lcd.init(); 
  lcd.backlight();
  lcd.setCursor(0, 1);
  lcd.print("      Welcome to     ");
  lcd.setCursor(0, 2);
  lcd.print("         WQMS  ");
  delay(2000);
  lcd.clear();
}


void loop() {

  kirim();
  String minta = "";
  while(Serial.available()>0){
    minta += char(Serial.read());
  }
  minta.trim();
  if(minta == "Ya"){
    String data = String(suhu)+"#"+String(tds)+"#"+String(ph)+"#"+String(kekeruhan);
    Serial.println(data);
  }
  minta = "";
 
  lcd.setCursor(0, 0);            
  lcd.print("Suhu : ");  
  lcd.print(suhu);
  lcd.print(char(223));
  lcd.print("C");   
  lcd.setCursor(0, 1);            
  lcd.print("Tds  : "); 
  lcd.print(tds);
  lcd.print(" PPM");
  lcd.setCursor(0, 2);           
  lcd.print("PH   : "); 
  lcd.print(ph);  
   lcd.setCursor(0, 3);           
  lcd.print("Kekeruhan   : "); 
  lcd.print(kekeruhan); 
  delay(1000);
  
}



void kirim(){
  sensorSuhu();
  sensorTds();
  sensorPh();
  sensorKekeruhan();
  
}


void sensorSuhu(){
  // Call sensors.requestTemperatures() to issue a global temperature and Requests to all devices on the bus
  sensors.requestTemperatures(); 
  suhu = sensors.getTempCByIndex(0);
}

void sensorTds(){
   static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 40U)     //every 40 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
   }   
   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 800U)
   {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient=1.0+0.02*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
      tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
      //Serial.print("voltage:");
      //Serial.print(averageVoltage,2);
      //Serial.print("V   ");
      // Serial.print("TDS Value:");
      // Serial.print(tdsValue,0);
      // Serial.println("ppm");
      tds = tdsValue;
   }
}
int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
      for (i = 0; i < iFilterLen - j - 1; i++) 
          {
        if (bTab[i] > bTab[i + 1]) 
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}

void sensorPh(){
  // int nilai_analog_PH = analogRead (ph_Pin);  //membaca nilai A0
  // Serial.print("Nilai ADC Ph: ");
  // Serial.println(nilai_analog_PH);
  // TeganganPh = 5 / 1024.0 * nilai_analog_PH;
  // Serial.print("TeganganPh: ");
  // Serial.println(TeganganPh, 3);

  // PH_step = (PHEmpat - PHTujuh) / 3;
  // Po = 7.00 + ((PHTujuh - TeganganPh) / PH_step);
  // // // Serial.print("Nilai PH cairan: ");
  // // Serial.println(Po, 2);

  // ph = (Po,2);

  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue,voltage;
  if(millis()-samplingTime > samplingInterval)
  {
      pHArray[pHArrayIndex++]=analogRead(SensorPin);
      if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
      voltage = avergearray(pHArray, ArrayLenth)*5.0/1024;
      pHValue = 3.5*voltage+Offset;
      samplingTime=millis();
  }
  if(millis() - printTime > printInterval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
    // Serial.print("Voltage:");
    //     Serial.print(voltage,2);
    //     Serial.print("    pH value: ");
    // Serial.println(pHValue,2);
    ph =(pHValue,2);
    // lcd.setCursor(1, 0);       // karakter dimulai dari kolom 5 baris 1
    // lcd.print(pHValue);   // Karakter yang akan ditampilkan pada LCD
    //     digitalWrite(LED,digitalRead(LED)^1);
        printTime=millis();
  }
}
double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;

}

void sensorKekeruhan(){
  int sensorValue = analogRead(A2);// read the input on analog pin 2:
  float voltage = sensorValue * (5.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  Serial.println(voltage); // print out the value you read:
  // kekeruhan = (voltage + 4.3454)/ (-0.0015);
  // Serial.println(kekeruhan);
  // Serial.println("-----");
  kekeruhan = (voltage - 4.1739)/ (-0.0022);

  //  kekeruhan= map (sensorValue,580,851,385,0);

  // kekeruhan = kekeruhanVal - 10;
  // Serial.println(kekeruhan);
  // delay(500);
  
}