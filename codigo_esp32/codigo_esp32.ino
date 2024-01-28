#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

/*************************************************************

  This is a simple demo of sending and receiving some data.
  Be sure to check out other examples!
 *************************************************************/

/* Fill-in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID "TMPL2ozjkqfD7"
#define BLYNK_TEMPLATE_NAME "nuevo"
#define BLYNK_AUTH_TOKEN "LbI7adJlBDYIj5TBDGZrqY0QcJZsdfoP"

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial


#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "cesar";
char pass[] = "00000002";

BlynkTimer timer;

// Define pins
#define startBtnPIN 2   // D2
#define selectBtnPIN 5  // D3
#define upBtnPIN 14     // D4
#define downBtnPIN 13   // D5


#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

bool stateStartBtn = 0;
bool stateSelectBtn = 0;
bool stateUpBtn = 0;
bool stateDownBtn = 0;
bool startd = 0;
bool mode = 0;
bool parameter = 0;
int AperatureValues = 16;
int IsoValues = 11;
int start = 0;
const int pwm_pin = 4;
const int ntc = 32;

//variables señal pwm

int canal = 0;
int resolucion = 8;
float frecuencia = 1000;
//variables para la ntc 100k
const double resistorValue = 100000.0;   // Valor nominal de la resistencia en ohmios a 25°C
const double betaValue = 3950.0;         // Valor beta de la NTC
const double nominalTemperature = 25.0;  // Temperatura nominal en grados Celsius
const int numReadings = 40;              // Número de lecturas a promediar
//variables para el controlador pi

int set_temperature = 110;
float temperature_read = 0;
int Kp = 3;
float Ki = 0.01;
float Kd = 0.3;
float previous_error = 0;
float elapsedTime, Time, timePrev;

//controlador

float PID_p = 0;
float PID_i = 0;
float PID_d = 0;
float control_PID = 0;


void setup() {
  // initialize serial port
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Wire.begin();
  pinMode(pwm_pin, OUTPUT);
  ledcAttachPin(pwm_pin, canal);
  ledcSetup(canal, frecuencia, resolucion);
  Time = millis();

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }


  display.clearDisplay();
  display.setTextSize(1);  // Draw 1X-scale text
  display.setTextColor(SSD1306_WHITE);
  //inicio


  // setup button
  pinMode(startBtnPIN, INPUT_PULLUP);
  pinMode(selectBtnPIN, INPUT_PULLUP);
  pinMode(upBtnPIN, INPUT_PULLUP);
  pinMode(downBtnPIN, INPUT_PULLUP);

  display.setCursor(40, 8);
  display.println("MENU");
  display.setCursor(40, 24);
  display.println("START");
  display.setCursor(0, 40);
  display.println("SELECT TO CONFIGURE");
  display.display();
  delay(1000);
}

void loop() {
  if (start == 1) {
    Blynk.run();
    timer.run();
  }

  if (start == 0) {
    Blynk.run();
    timer.run();

    PID_p = 0;
    PID_i = 0;
    PID_d = 0;
    control_PID = 0;
    readButtons();
    Serial.println(start);
    ledcWrite(0, control_PID);  // Escribe la señal PWM en el canal 0
  } else if (start == 1) {
    temperatura();
  }
}



void readButtons() {
  //=====start button==========
  if (digitalRead(startBtnPIN) == LOW && stateStartBtn == 0) {
    stateStartBtn = 1;
    start = 1;
    Serial.println("start button is pressed");

    if (mode == 0) {

      display.clearDisplay();
      display.setCursor(32, 8);
      display.println("THE PROCESS");
      display.setCursor(32, 24);
      display.println("HAS STARTED!");

      display.display();
    }

  } else if (digitalRead(startBtnPIN) == HIGH && stateStartBtn == 1) {
    stateStartBtn = 0;
  }
  //===select button=================
  if (digitalRead(selectBtnPIN) == LOW && stateSelectBtn == 0) {
    stateSelectBtn = 1;
    start = 0;
    if (mode == 0) {
      mode = 1;
      start = 0;

      display.clearDisplay();
      display.setCursor(10, 10);
      display.print("TEMPERATURA= ");
      display.println(set_temperature);
      display.display();

    } else if (mode == 1 && parameter == 0) {
      parameter = 1;
      start = 0;
      display.clearDisplay();
      display.setCursor(10, 10);
      display.print("KP=");
      display.println(Kp);
      display.display();

    } else if (parameter == 1) {
      mode = 0;
      parameter = 0;
      screenReady();
    }

    Serial.println("select button is pressed");

    //  Serial.print("Mode="); Serial.println(mode);
    //  Serial.print("Parameter="); Serial.println(parameter);

  } else if (digitalRead(selectBtnPIN) == HIGH && stateSelectBtn == 1) {
    stateSelectBtn = 0;
  }
  //=====up button==========
  if (digitalRead(upBtnPIN) == LOW && stateUpBtn == 0 && mode == 1) {
    stateUpBtn = 1;
    Serial.println("up button is pressed");

    display.clearDisplay();
    display.setCursor(10, 10);

    if (parameter == 0) {
      if (set_temperature < 200) set_temperature++;

      display.clearDisplay();
      display.setCursor(10, 10);
      display.print("TEMPERATURA= ");
      display.println(set_temperature);
      display.display();
    } else if (parameter == 1) {
      if (Kp < 100) Kp++;

      display.clearDisplay();
      display.setCursor(10, 10);
      display.print("KP= ");
      display.println(Kp);
      display.display();
    }



  } else if (digitalRead(upBtnPIN) == HIGH && stateUpBtn == 1) {
    stateUpBtn = 0;
  }
  //=====down button==========
  if (digitalRead(downBtnPIN) == LOW && stateDownBtn == 0 && mode == 1) {
    stateDownBtn = 1;
    Serial.println("down button is pressed");

    display.clearDisplay();
    display.setCursor(10, 10);

    if (parameter == 0) {
      if (set_temperature > 0) set_temperature--;

      display.clearDisplay();
      display.setCursor(10, 10);
      display.print("TEMPERATURA= ");
      display.println(set_temperature);
      display.display();
    } else if (parameter == 1) {
      if (Kp > 0) Kp--;

      display.clearDisplay();
      display.setCursor(10, 10);
      display.print("KP= ");
      display.println(Kp);
      display.display();
    }

  } else if (digitalRead(downBtnPIN) == HIGH && stateDownBtn == 1) {
    stateDownBtn = 0;
  }
}

void screenReady() {
  // Display start
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("READY");
  display.println("      ");
  display.println("PRESS START BUTTON");
  display.println("      ");
  display.println("SELECT TO CONFIGURE");
  display.display();
  delay(1000);
}
void temperatura() {

  // put your main code here, to run repeatedly:
  double totalTemperature = 0.0;

  // Realiza un bucle for para leer y promediar las lecturas analógicas
  for (int i = 0; i < numReadings; i++) {
    // Lee el valor analógico del pin de la NTC
    int rawValue = analogRead(ntc);

    // Convierte el valor analógico a resistencia
    double resistance = resistorValue / ((4095.0 / rawValue) - 1.0);

    // Calcula la temperatura en grados Celsius
    double temperature = 1.0 / ((1.0 / (nominalTemperature + 273.15)) + ((1.0 / betaValue) * log(resistance / resistorValue)));

    // Convierte la temperatura a grados Celsius restando 273.15
    temperature -= 273.15;

    // Acumula las temperaturas para el promedio
    totalTemperature += temperature;
    delay(15);  // Pausa breve entre lecturas
  }

  // Calcula el promedio de las temperaturas
  double averageTemperature = totalTemperature / numReadings;




  // Calcula el error entre la temperatura deseada y la temperatura real
  temperature_read = averageTemperature;
  Blynk.virtualWrite(V3, temperature_read);
  float error = set_temperature - temperature_read;


  // Calcula la señal de control proporcional (PWM)
  PID_p = Kp * error;
  PID_i = PID_i + (Ki * error);

  timePrev = Time;
  Time = millis();
  elapsedTime = (Time - timePrev) / 1000;
  PID_d = Kd * ((error - previous_error) / elapsedTime);




  control_PID = PID_p + PID_i + PID_d;
  // Limita el valor de la señal de control al rango [0, 255]


  if (control_PID < 0) {
    control_PID = 0;
  }
  if (control_PID > 255) {
    control_PID = 255;
  }





  // Escribe la señal PWM para el rele de estado solido
  ledcWrite(0, control_PID);  // Escribe la señal PWM en el canal 0

  // Imprime la temperatura y el error en el monitor serie
  Serial.print("Temperatura: ");
  Serial.print(averageTemperature);
  Serial.println(" grados Celsius  ");

  previous_error = error;
  display.clearDisplay();
  display.setCursor(32, 8);
  display.println("THE PROCESS");
  display.setCursor(32, 24);
  display.println("HAS STARTED!");
  display.setCursor(16, 40);
  display.println("TEMPERATURE:");
  display.setCursor(88, 40);
  display.println(averageTemperature);
  display.display();
  delay(100);
  if (digitalRead(selectBtnPIN) == LOW && stateSelectBtn == 0) {
    start = 0;
  }
}

BLYNK_WRITE(V0) {
  start = param.asInt();
  Serial.println(start);
}

BLYNK_WRITE(V1) {
  set_temperature = param.asInt();
  Serial.print("temperatura :");
  Serial.println(set_temperature);
}

BLYNK_WRITE(V2) {
  Kp = param.asInt();
  Serial.print("constante proporcional :");
  Serial.println(Kp);
}
