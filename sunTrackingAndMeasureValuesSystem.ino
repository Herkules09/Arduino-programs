#include <ModbusMaster.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <Arduino.h>
#include <TimeLib.h>
#include <SD.h>
#include <SPI.h>
#include "RTClib.h"
#include <U8glib.h>
#include "TimerOne.h"
#include "TimerThree.h"

#define MAX_BUFFER_SIZE 8
#define modbusaddr 1  // adres modbusa sdm120 domyślnie 1
#define DEG_TO_RAD 0.01745329
#define PI 3.141592654
#define TWOPI 6.28318531


ModbusMaster node; //Definicja modbusa
RTC_DS3231 rtcDS;  //Definicja zegrara RTC
QMC5883LCompass compass; //Definicja kompasu

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_FAST);

//DO KOMPASU
// Mode Control (MODE)
const byte qmc5883l_mode_stby = 0x00;
const byte qmc5883l_mode_cont = 0x01;
// Output Data Rate (ODR)
const byte qmc5883l_odr_10hz  = 0x00;
const byte qmc5883l_odr_50hz  = 0x04;
const byte qmc5883l_odr_100hz = 0x08;
const byte qmc5883l_odr_200hz = 0x0C;
// Full Scale Range (RNG)
const byte qmc5883l_rng_2g    = 0x00;
const byte qmc5883l_rng_8g    = 0x10;
// Over Sample Ratio (OSR)
const byte qmc5883l_osr_512   = 0x00;
const byte qmc5883l_osr_256   = 0x40;
const byte qmc5883l_osr_128   = 0x80;
const byte qmc5883l_osr_64    = 0xC0;





const int analogPin1 = A1;            // Pin analogowy, do którego podłączono czujnik 1 prądu ACS712
const int analogPin2 = A2;            // Pin analogowy, do którego podłączono czujnik 2 prądu ACS712
const int analogPin3 = A3;            // Pin analogowy, do którego podłączono czujnik 3 prądu ACS712
const int analogPin4 = A4;            // Pin analogowy, do którego podłączono przetwornik napięcia U1
const int analogPin5 = A5;            // Pin analogowy, do którego podłączono przetwornik napięcia U2
const int pinLDR1 = A6;               // Pin analogowy, do którego podłączono diodę LDR 1
const int pinLDR2 = A7;               // Pin analogowy, do którego podłączono diodę LDR 2
const int analogPin8 = A8;   //Pin do miernika napięcia U3 (do zrobienia !!!)


const int przelacznik1 = 29;         //Pin cyfrowy do przełącznika do zmiany trybu
const int przelacznik2 = 28;         //Pin cyfrowy do przełącznika do zmiany trybu
const int kompasReset = 23;          //Pin cyfrowy do przycisku resetującego kompas 

// Definicje pinów dla przekaźników
const int relayPin1 = 2;
const int relayPin2 = 3;
const int relayPin3 = 4;
const int relayPin4 = 5;


const int chipSelect = 53; // Pin CS (Chip Select) dla karty SD
File file; // Obiekt pliku do zapisu danych

String pomiar;


// int odczytU1 =0;
// float napiecieU1 = 0;
// int odczytU2 =0;
// float napiecieU2 = 0;
const float przelicznik_U1 =0.159;
const float przelicznik_U2 =0.154;
const float przelicznik_U3 =0.27;

const float referenceVoltage = 5.0;   // Napięcie referencyjne
const float sensitivity_I3 = 0.1028;  // Czułość czujnika 3 ACS712 w mV/A 
const float sensitivity_I2 = 0.116;     // Czułość czujnika 2 ACS712 w mV/A 
const float sensitivity_I1 = 0.174;     // Czułość czujnika 1 ACS712 w mV/A 
const int liczbaPomiarow = 10;
float pomiary1[liczbaPomiarow]; //pomiary prądu na czujniku 1
float pomiary2[liczbaPomiarow]; //pomiary prądu na czujniku 2
float pomiary3[liczbaPomiarow]; //pomiary prądu na czujniku 3
int index = 0;
int compasCorrect =0;


bool isError =0; //okresla czy jest błąd podczas zapisu na karte SD

int sensorValue1; //Wartość na czujniku LDR 1
int sensorValue2; //Wartość na czujniku LDR 2

//Struktura określająca wartości odczytane z SDM120
struct EnergyValues {
  float voltage;
  float current;
  float activePower;
  float totalActiveEnergy;
};

//Struktura określająca średnie wartości pomiarów prądu 
struct AverageCurrents {
  float averageCurrent1;
  float averageCurrent2;
  float averageCurrent3;
};

//Funkcja wyświetlająca na ekranie tryb pozycjonowania oraz status zapisu na karte SD
void draw(const char* tryb, const char* status) {
  u8g.setFont(u8g_font_8x13);
  u8g.drawStr(0, 19, "Pozycjonowanie");
  u8g.drawStr(0, 32, "Tryb:");
  u8g.drawStr(40, 32, tryb);
  u8g.drawStr(0, 45, "Status");
  u8g.drawStr(40, 45, status);
}



unsigned long previousMillis1 = 0; // Zmienna do przechowywania poprzedniego czasu dla funkcji 1
unsigned long previousMillis2 = 0; // Zmienna do przechowywania poprzedniego czasu dla funkcji 2
const long interval1 = 5000;       // Interwał czasowy dla funkcji 1 (5 sekund)
const long interval2 = 5000;      // Interwał czasowy dla funkcji 2 (10 sekund)



void setup() {
  Serial.begin(9600);
  Serial3.begin(2400);  // Use Serial3 for hardware serial on Mega
  compass.init();
  compass.setCalibration(-832, 1698, -2033, 895, -1266, 1821);
  //Timer1.initialize(5000000);   //Przerwanie co 20 minut dwa 0 dodac (1000000 - 1 sekunda)
 // Timer1.pwm(9,512);               //Ustawienie na pinie 9 generowania sygnału PWM, oraz współczynnik wypełnienia na 50%
 // Timer1.attachInterrupt(rotatePanels); 
 // Timer3.initialize(2000000);         //Przerwanie co 15 sekund
//  Timer3.attachInterrupt(measureValues);
//  Timer3.pwm(9,512);
  Wire.begin();
  u8g.setColorIndex(1);


 // Ustawianie zegara RTC
  if (!rtcDS.begin()) {
    Serial.println("Couldn't find RTC");
    while (1)
      ;
  }
  if (rtcDS.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtcDS.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtcDS.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  node.begin(modbusaddr, Serial3);

 // Inicjalizacja pinów przekaźników
  pinMode(relayPin1, OUTPUT);
  pinMode(relayPin2, OUTPUT);
  pinMode(relayPin3, OUTPUT);
  pinMode(relayPin4, OUTPUT);

  // Wyłączenie wszystkich przekaźników na początku
  digitalWrite(relayPin1, HIGH);
  digitalWrite(relayPin2, HIGH);
  digitalWrite(relayPin3, HIGH);
  digitalWrite(relayPin4, HIGH);

  //Inicjalizacja pinów przełącznika trybu pozycjonowania
  pinMode(przelacznik1,INPUT_PULLUP);
  pinMode(przelacznik2,INPUT_PULLUP);
  
  //Inicjalizacja pinu resetu kompasu
  pinMode(kompasReset,INPUT_PULLUP);
  
  //Inicjalizacja pinu CS czytnika karty SD
  pinMode(chipSelect,OUTPUT);

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
   // while (1);
  }

  Serial.println("Initializing SD card...");

  // Otwórz plik do zapisu danych
  file = SD.open("dane.txt", FILE_WRITE);

  if (file) {
    Serial.println("SD card initialized.");
  } else {
    Serial.println("Error opening data file.");
  }

}

//Funkcja do kalibracji kompasu
int comopasCalibration(){
  compass.read();
  Serial.print("Kierunek: ");
  int aktualnyAzymutKompasu = compass.getAzimuth();
  if(aktualnyAzymutKompasu<0){
    aktualnyAzymutKompasu= 360 + aktualnyAzymutKompasu;
  }
  return 180 - aktualnyAzymutKompasu;
  }



void loop() {

   unsigned long currentMillis = millis();

   int x_value;
   int y_value;
   int z_value;
   int azimuth;  // 0° - 359°
   byte bearing; // 0 - 15 (N, NNE, NE, ENE, E, ...)
   char direction[strlen("NNE") + 1];
   
   compass.read(); // Read compass values via I2C

   azimuth   = compass.getAzimuth(); // Calculated from X and Y value 
   bearing   = compass.getBearing(azimuth);
   
   compass.getDirection(direction, azimuth);
   direction[3] = '\0';

    bool stanPrzelacznika1 = digitalRead(przelacznik1);
    bool stanPrzelacznika2 = digitalRead(przelacznik2);
  //wyswietlanie trybu pracy
   
  
    

    
       if (stanPrzelacznika1 == HIGH && stanPrzelacznika2 == LOW) {
       //Program z pozycjonowaniem
       
         wysunSilownik();
      } else if (stanPrzelacznika1 == LOW && stanPrzelacznika2 == HIGH) {
        schowajSilownik();
       
       } else {
         zatrzymajSilownik();
        
       }
    
 
  


  bool reset = digitalRead(kompasReset);
  if(reset ==LOW){
    compasCorrect=comopasCalibration();
    Serial.print("Korekta: ");
    Serial.println(compasCorrect);
    Serial.println();
  }

   
   
   //wysunSilownik();
   //schowajSilownik();
 // Serial.print("Kierunek: ");
  
 // Serial.print(azimuth);
 // Serial.println(" stopni");

  if(azimuth<0){
    azimuth= 360 + azimuth;
  }

  //Serial.print("Kierunek po zmianie: ");
  //Serial.print(azimuth);
 // Serial.println(" stopni");

 // Serial.print("Kierunek po korekcie: ");
  azimuth = azimuth+compasCorrect;
 // Serial.print(azimuth);
 // Serial.println(" stopni");
  delay(2000);
if (currentMillis - previousMillis1 >= interval1) {
    // Zapisanie bieżącego czasu jako poprzedniego czasu
    previousMillis1 = currentMillis;
    // Wywołanie funkcji 1
    measureValues();
  }

 // schowajSilownik();
  if (currentMillis - previousMillis2 >= interval2) {
    // Zapisanie bieżącego czasu jako poprzedniego czasu
    previousMillis2 = currentMillis;
    //schowajSilownik();
    // Wywołanie funkcji 2
   // Serial.println(" rotacja paneli");
    //rotatePanels();
  
  }
}

 //WYBÓR STEROWANIA SILOWNIKAMI ZA POMOCA PRZEŁĄCZNIKA
void rotatePanels(){
 
    bool stanPrzelacznika1 = digitalRead(przelacznik1);
    bool stanPrzelacznika2 = digitalRead(przelacznik2);
    
    if(stanPrzelacznika1 == LOW && stanPrzelacznika2==HIGH){
    sterowanieSilownikiemLDR();
    Serial.println("Sterowanie za pomoca LDR");
  } else if(stanPrzelacznika2 == LOW && stanPrzelacznika1 == HIGH){
    sterowanieSilownikiemKompas();
    Serial.println("Sterowanie za pomoca Kompasu");
  }else{
    Serial.println("STOP");
 }
   

}

//Funkcja mierzaca  wartosci prądu, energii, napiecia
void measureValues(){
//POMIAR PRĄDU
   AverageCurrents averages = calculateAverageCurrents();
  //Serial.print("Średni prąd 1: ");
 // Serial.println(averages.averageCurrent1);

  //Serial.print("Średni prąd 2: ");
 // Serial.println(averages.averageCurrent2);

 // Serial.print("Średni prąd 3: ");
 // Serial.println(averages.averageCurrent3);


  //POMIAR NAPIECIA
  float napiecie_U1 = odczytNapiecia(analogPin4, przelicznik_U1);
  float napiecie_U2 = odczytNapiecia(analogPin5, przelicznik_U2); 
  float napiecie_U3 = odczytNapiecia(analogPin8, przelicznik_U3);


  //POMIAR PRADU
  float currentSensorValue1 = analogRead(analogPin1);
  float currentSensorValue2 = analogRead(analogPin2);
  float currentSensorValue3 = analogRead(analogPin3);

  // Przeliczanie wartości na napięcie
  float voltage1 = (currentSensorValue1 / 1023.0) * referenceVoltage;
  float voltage2 = (currentSensorValue2 / 1023.0) * referenceVoltage;
  float voltage3 = (currentSensorValue3 / 1023.0) * referenceVoltage;

  // Przeliczanie napięcia na prąd
  double current1 = (voltage1 - (referenceVoltage / 2.0)) * sensitivity_I1;
  double current2 = (voltage2 - (referenceVoltage / 2.0)) * sensitivity_I2;
  double current3 = (voltage3 - (referenceVoltage / 2.0)) * sensitivity_I3;
  if(current3>0){
    current3=0;
  }else{
    current3=0.4;
    napiecie_U3=napiecie_U3*1.77;
  }


 // Serial.print("Napiecie U1: ");
 // Serial.print(napiecie_U1);
 // Serial.println(" V");

 // Serial.print("Napiecie U2: ");
 // Serial.print(napiecie_U2);
 // Serial.println(" V");

 // Serial.print("Napiecie U3: ");
  //Serial.print(napiecie_U3);
 // Serial.println(" V");


  //POMIAR MOCY
  EnergyValues energyValues = getRTU(0x0000);

  //Serial.println("Voltage : " + String(energyValues.voltage, 2));
 // Serial.println("Current : " + String(energyValues.current, 2));
  //Serial.println("Active Power : " + String(energyValues.activePower, 2));
 // Serial.println("Total Active Energy : " + String(energyValues.totalActiveEnergy, 2));

  //Serial.println("==================================");
  
  pomiar=String(energyValues.voltage, 2) +", "+ String(energyValues.current, 2) +", "+ String(energyValues.activePower, 2) +", "+ 
  String(energyValues.totalActiveEnergy, 2) +", "+ String(napiecie_U1)+", "+ String(napiecie_U2)+", "+ String(napiecie_U3)+
  ", "+ String(current1,4)+", "+ String(current2,4)+", "+ String(current3,4);
     

  file = SD.open("dane.txt", FILE_WRITE);
  DateTime now;
    // Sprawdzamy czy plik otwarto poprawnie

       now = rtcDS.now();
       Serial.print(now.year(), DEC);
       Serial.print('/');
       Serial.print(now.month(), DEC);
       Serial.print('/');
      Serial.print(now.day(), DEC);
      Serial.print(" ");
      Serial.print(now.hour(), DEC);
      Serial.print(':');
      Serial.print(now.minute(), DEC);
      Serial.print(':');
      Serial.print(now.second(), DEC);
      Serial.print("   ");
      Serial.print(pomiar);
      Serial.println();
      



    if (file) {
       now = rtcDS.now();
       file.print(now.year(), DEC);
       file.print('/');
       file.print(now.month(), DEC);
       file.print('/');
      file.print(now.day(), DEC);
      file.print(" ");
      file.print(now.hour(), DEC);
      file.print(':');
      file.print(now.minute(), DEC);
      file.print(':');
      file.print(now.second(), DEC);
      file.print("   ");
      file.print(pomiar);
      file.println();
      file.close();






        // Zamykamy plik
       
    } else {
        // Jeśli wystąpił błąd przy otwieraniu pliku
        //Serial.println("Błąd podczas otwierania pliku!");
    }

}


  

// POMIAR NAPIĘCIA
float odczytNapiecia(int pin, float przelicznik) {
  int odczyt =0;
  float napiecie = 0;
  odczyt = analogRead(pin);
  napiecie = (odczyt * (5.0 / 1024.0)) / przelicznik;
  return napiecie;
}

// POMIAR PRĄDU
AverageCurrents calculateAverageCurrents() {
  
  int currentSensorValue1 = analogRead(analogPin1);
  int currentSensorValue2 = analogRead(analogPin2);
  int currentSensorValue3 = analogRead(analogPin3);

  // Przeliczanie wartości na napięcie
  float voltage1 = (currentSensorValue1 / 1023.0) * referenceVoltage;
  float voltage2 = (currentSensorValue2 / 1023.0) * referenceVoltage;
  float voltage3 = (currentSensorValue3 / 1023.0) * referenceVoltage;

  // Przeliczanie napięcia na prąd
  float current1 = (voltage1 - (referenceVoltage / 2.0)) / sensitivity_I1;
  float current2 = (voltage2 - (referenceVoltage / 2.0)) / sensitivity_I2;
  float current3 = (voltage3 - (referenceVoltage / 2.0)) / sensitivity_I3;

  float nowyPomiar1 = current1;
  float nowyPomiar2 = current2;
  float nowyPomiar3 = current3;

  // Zapis pomiarów do tablic
  pomiary1[index] = nowyPomiar1;
  pomiary2[index] = nowyPomiar2;
  pomiary3[index] = nowyPomiar3;

  // Przesunięcie indeksu cyklicznie w tablicy
  index = (index + 1) % liczbaPomiarow;

  //średnia pomiarów
  float suma1 = 0;
  for (int i = 0; i < liczbaPomiarow; i++) {
    suma1 += pomiary1[i];
  }
  float srednia1 = suma1 / liczbaPomiarow;

  float suma2 = 0;
  for (int i = 0; i < liczbaPomiarow; i++) {
    suma2 += pomiary2[i];
  }
  float srednia2 = suma2 / liczbaPomiarow;

  float suma3 = 0;
  for (int i = 0; i < liczbaPomiarow; i++) {
    suma3 += pomiary3[i];
  }
  float srednia3 = suma3 / liczbaPomiarow;

  
  AverageCurrents averages;
  averages.averageCurrent1 = srednia1;
  averages.averageCurrent2 = srednia2;
  averages.averageCurrent3 = srednia3;

  return averages;
}



//METODY DO ODCZYTU MODBUS SDM120
float reform_uint16_2_float32(uint16_t u1, uint16_t u2) {
  uint32_t num = ((uint32_t)u1 & 0xFFFF) << 16 | ((uint32_t)u2 & 0xFFFF);
  float numf;
  memcpy(&numf, &num, 4);
  return numf;
}

EnergyValues getRTU(uint16_t m_startAddress) {
  uint8_t m_length = 8;
  uint16_t results[MAX_BUFFER_SIZE];
  EnergyValues values;

  uint16_t result = node.readInputRegisters(m_startAddress, m_length);
  if (result == node.ku8MBSuccess) {
    for (int i = 0; i < m_length; i++) {
      results[i] = node.getResponseBuffer(i);
    }

    values.voltage = reform_uint16_2_float32(results[0], results[1]);
    values.current = reform_uint16_2_float32(results[2], results[3]);
    values.activePower = reform_uint16_2_float32(results[4], results[5]);
    values.totalActiveEnergy = reform_uint16_2_float32(results[6], results[7]);
  }

  return values;
}


//Obliczanie czy miedzy dwoma wartosciami jest mniejsza niz 40 
bool sprawdzRoznice(int wartosc1, int wartosc2) {
  // Oblicz wartość bezwzględną różnicy dwóch wartości
  int roznica = abs(wartosc1 - wartosc2);

  // Sprawdź, czy wartość bezwzględna różnicy jest mniejsza niż 40
  if (roznica < 40) {
    return true;
  } else {
    return false;
  }
}


int zaokraglDoDziesiatek(int wartosc) {
  return ((wartosc + 5) / 10) * 10;
}


long JulianDate(int year, int month, int day) {
  long JD_whole;
  int A, B;
  if (month <= 2) {
    year--;
    month += 12;
  }
  A = year / 100;
  B = 2 - A + A / 4;
  JD_whole = (long)(365.25 * (year + 4716)) + (int)(30.6001 * (month + 1)) + day + B - 1524;
  return JD_whole;
}


int obliczAzymutSlonca(int hour, int minute, int second, int day, int month, int year, int zone, float Lat, float Lon) {

  float T, JD_frac, L0, M, e, C, L_true, f, R, GrHrAngle, Obl, RA, Decl, HrAngle, elev, azimuth;
  long JD_whole, JDx;

  Lon = Lon * DEG_TO_RAD;
  Lat = Lat * DEG_TO_RAD;

  JD_whole = JulianDate(year, month, day);
  JD_frac = (hour + minute / 60. + second / 3600.) / 24. - .5;
  T = JD_whole - 2451545;
  T = (T + JD_frac) / 36525.;
  L0 = DEG_TO_RAD * fmod(280.46645 + 36000.76983 * T, 360);
  M = DEG_TO_RAD * fmod(357.5291 + 35999.0503 * T, 360);
  e = 0.016708617 - 0.000042037 * T;
  C = DEG_TO_RAD * ((1.9146 - 0.004847 * T) * sin(M) + (0.019993 - 0.000101 * T) * sin(2 * M) + 0.00029 * sin(3 * M));
  f = M + C;
  Obl = DEG_TO_RAD * (23 + 26 / 60. + 21.448 / 3600. - 46.815 / 3600 * T);
  JDx = JD_whole - 2451545;
  GrHrAngle = 280.46061837 + (360 * JDx) % 360 + .98564736629 * JDx + 360.98564736629 * JD_frac;
  GrHrAngle = fmod(GrHrAngle, 360.);
  L_true = fmod(C + L0, TWOPI);
  R = 1.000001018 * (1 - e * e) / (1 + e * cos(f));
  RA = atan2(sin(L_true) * cos(Obl), cos(L_true));
  Decl = asin(sin(Obl) * sin(L_true));
  HrAngle = DEG_TO_RAD * GrHrAngle + Lon - RA;

  //Azimuth measured eastward from north.
  azimuth = PI + atan2(sin(HrAngle), cos(HrAngle) * sin(Lat) - tan(Decl) * cos(Lat));

  return azimuth / DEG_TO_RAD;
}

// Funkcja do wysuwania siłownika
void schowajSilownik() {
  //Serial.println("Schowanie silownika");
  digitalWrite(relayPin3, HIGH);
  digitalWrite(relayPin2, HIGH);
  digitalWrite(relayPin1, LOW);
  digitalWrite(relayPin4, LOW);
}

// Funkcja do chowania siłownika
void wysunSilownik() {
  //Serial.println("Wysuwanie silownika");
  digitalWrite(relayPin1, HIGH);
  digitalWrite(relayPin4, HIGH);
  digitalWrite(relayPin2, LOW);
  digitalWrite(relayPin3, LOW);
}

// Funkcja do zatrzymywania siłownika
void zatrzymajSilownik() {
  //Serial.println("Zatrzymanie silownika");
  digitalWrite(relayPin1, HIGH);
  digitalWrite(relayPin2, HIGH);
  digitalWrite(relayPin3, HIGH);
  digitalWrite(relayPin4, HIGH);
}

//FUNKCJA STERUJACA ZA POMOCA KOMPASU
void sterowanieSilownikiemKompas() {
  
   int aktualnyAzymutKompasu;  // 0° - 359°
   byte bearing; // 0 - 15 (N, NNE, NE, ENE, E, ...)
   char direction[strlen("NNE") + 1];
   
   compass.read(); // Read compass values via I2C

   aktualnyAzymutKompasu   = compass.getAzimuth(); // Calculated from X and Y value 
   bearing   = compass.getBearing(aktualnyAzymutKompasu);
   
   compass.getDirection(direction, aktualnyAzymutKompasu);
   direction[3] = '\0';



  DateTime t = rtcDS.now();
  Serial.print("Kierunek: ");
   aktualnyAzymutKompasu = compass.getAzimuth()+compasCorrect;
  Serial.print(aktualnyAzymutKompasu);
  Serial.println(" stopni");


  int aktualnyAzymutSlonca = obliczAzymutSlonca(hour(t.hour())-1, minute(t.minute()), second(t.second()), day(t.day()), month(t.month()), year(t.year()), 1, 52.23166, 21.00583);
  Serial.println(aktualnyAzymutSlonca);
  delay(4000);

  if (!sprawdzRoznice(aktualnyAzymutKompasu, aktualnyAzymutSlonca)) {
    if (aktualnyAzymutKompasu > aktualnyAzymutSlonca) {
      while (aktualnyAzymutKompasu >= aktualnyAzymutSlonca) {
        compass.read();
        aktualnyAzymutKompasu = compass.getAzimuth();
        Serial.println("Aktualny azymut kompasu: ");
        Serial.print(aktualnyAzymutKompasu);
        Serial.println("Aktualny azymut slonca: ");
        Serial.print(aktualnyAzymutSlonca);
        Serial.println("Silownik w lewo");
        schowajSilownik();
        delay(1000);
      }
    }

    if (aktualnyAzymutKompasu < aktualnyAzymutSlonca) {
      while (aktualnyAzymutKompasu <= aktualnyAzymutSlonca) {
        compass.read();
        aktualnyAzymutKompasu = compass.getAzimuth();
        Serial.println("Aktualny azymut kompasu: ");
        Serial.print(aktualnyAzymutKompasu);
        Serial.println("Aktualny azymut slonca: ");
        Serial.print(aktualnyAzymutSlonca);
        Serial.println("Silownik w prawo");
        wysunSilownik();
        delay(1000);
      }
    }
  }

  zatrzymajSilownik();
}

//FUNKCJA STERUJACA ZA POMOCĄ LDR 
void sterowanieSilownikiemLDR(){
  // Odczyt wartości napięcia na LDR
    sensorValue1 = zaokraglDoDziesiatek(analogRead(pinLDR1));
    sensorValue2 = zaokraglDoDziesiatek(analogRead(pinLDR2));
    
    Serial.print("Wartość czujnika LDR1: ");
    Serial.println(sensorValue1);
    Serial.print("Wartość czujnika LDR2: ");
    Serial.println(sensorValue2);
    

    if (!sprawdzRoznice(sensorValue1, sensorValue2)) {

      if (sensorValue1 < sensorValue2) {
        while (sensorValue1 <= sensorValue2) {
          sensorValue1 = zaokraglDoDziesiatek(analogRead(pinLDR1));
          sensorValue2 = zaokraglDoDziesiatek(analogRead(pinLDR2));
          Serial.print("-Wartość czujnika LDR1: ");
          Serial.println(sensorValue1);
          Serial.print("-Wartość czujnika LDR2: ");
          Serial.println(sensorValue2);
          Serial.println("silnik w prawo");
          wysunSilownik();
        }
      }

      if (sensorValue1 > sensorValue2) {
        while (sensorValue1 >= sensorValue2) {
          sensorValue1 = zaokraglDoDziesiatek(analogRead(pinLDR1));
          sensorValue2 = zaokraglDoDziesiatek(analogRead(pinLDR2));
          Serial.print("-Wartość czujnika LDR1: ");
          Serial.println(sensorValue1);
          Serial.print("-Wartość czujnika LDR2: ");
          Serial.println(sensorValue2);
          Serial.println("silnik w lewo");
          schowajSilownik();
        }
      }
    }
    zatrzymajSilownik();
  }

