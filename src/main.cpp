#include <Arduino.h>
// Programa: Display LCD 16x2 e modulo I2C
// Autor: Arduino e Cia

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneButton.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <NewPing.h>

// Inicializa o display no endereco 0x27
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Define pins for joystic
#define ANALOG_L_R A0
#define ANALOG_U_D A1
#define ANALOG_BTN 3

// Control Button on joystick
OneButton button(ANALOG_BTN, true);

// Ultrassonic variables
// Blue
#define SONIC_TRIG 11
#define SONIC_ECHO 12
#define MAX_DISTANCE 200

NewPing sonar(SONIC_TRIG, SONIC_ECHO, MAX_DISTANCE);

long duration, cm, inches;

// Relay variables
// Purple
#define RELAY_PIN 13
bool relayState = false;
// Temeperature Sensor variables
// Red
#define TEMPERATURE_PIN 2

// PH Sensor variables
// Green
float valor_calibracao = 21.34;
int contagem = 0;      // Variável de contagem
float soma_tensao = 0; // Variável para soma de tensão
float media = 0;       // Variável que calcula a media
float entrada_A0;      // Variável de leitura do pino A0
float tensao;          // Variável para
unsigned long tempo;   // Float tempo

#define PH_PIN A3

OneWire oneWire(TEMPERATURE_PIN);

DallasTemperature sensors(&oneWire);

DeviceAddress insideThermometer;

// Menu variables
String lastDir = "";
String lastBtn = "";

int menuPos = 1;
int menuPage = 1;
int itemsPerPage = 2;

int subMenuPos;
boolean subMenuActive = false;

unsigned long millisTarefa1 = millis();

// 2 linhas 16 char

String menus[] = {"Leituras", "Temperatura", "Rele", "Nivel", "Ph", "Aj Wi-fi", "Ver IP"};

// Pré declared Functions to initialize before arduino boot
String getMenuItem(int pos, String text, bool Selected);
String checkAnalogDir();
String checkBtn();
void checkBtnPress();
void checkPos();
void controlPage(int pos);
void controlPos(String dir);
void menu(int pos, int menuPage);
void actions(int menuPos);
void printLine(int line, String text);
void btnClick();
void btnDoubleClick();
void btnLongPress();
void getDistance(String unitMode);
float getPH();
void calibratePH();
float getDistance();
void toggleRelay(String state);
float getTemperature();
String getRelayState();

String getMenuItem(int pos, String text, bool Selected)
{
  if (Selected)
  {
    return "> " + String(pos) + " " + text;
  }
  else
  {
    return "  " + String(pos) + " " + text;
  }
}

String checkAnalogDir()
{
  if (analogRead(ANALOG_L_R) < 350)
  {
    return "R";
  }
  else if (analogRead(ANALOG_L_R) > 650)
  {
    return "L";
  }
  else if (analogRead(ANALOG_U_D) < 350)
  {
    return "U";
  }
  else if (analogRead(ANALOG_U_D) > 650)
  {
    return "D";
  }
  else
  {
    return "";
  }
}

String checkBtn()
{
  if (digitalRead(ANALOG_BTN))
  {
    return "B";
  }
  else
  {
    return "";
  }
}

void checkBtnPress()
{
  if (lastBtn != checkBtn())
  {
    if (checkBtn() != "")
    {
      actions(menuPos);
    }
    lastBtn = checkBtn();
  }
}

void checkPos()
{
  if (lastDir != checkAnalogDir())
  {
    if (checkAnalogDir() != "")
    {
      if (subMenuActive)
      {
        actions(menuPos);
      }
      else
      {
        controlPos(checkAnalogDir());
        controlPage(menuPos);
        menu(menuPos, menuPage);
      }
    }
    lastDir = checkAnalogDir();
  }
}

void controlPage(int pos)
{

  if (pos > menuPage * itemsPerPage)
  {
    menuPage += 1;
  }
  else if (pos + 1 < menuPage * itemsPerPage)
  {
    menuPage -= 1;
  }
}

void controlPos(String dir)
{
  if (dir.equals("U") && menuPos > 1)
  {
    menuPos -= 1;
  }
  else if (dir.equals("D") && menuPos < (sizeof(menus) / sizeof(menus[0])))
  {
    menuPos += 1;
  }
}

void printLine(int line, String text)
{
  if (line == 1)
  {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 0);
    lcd.print(text);
  }
  else if (line == 2)
  {
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print(text);
  }
}

void menu(int pos, int menuPage)
{
  if (pos == (menuPage * itemsPerPage - 1))
  {
    Serial.println(getMenuItem((menuPage * itemsPerPage - 1), menus[(menuPage * itemsPerPage - 1) - 1], true));
    printLine(1, getMenuItem((menuPage * itemsPerPage - 1), menus[(menuPage * itemsPerPage - 1) - 1], true));
  }
  else
  {
    Serial.println(getMenuItem((menuPage * itemsPerPage - 1), menus[(menuPage * itemsPerPage - 1) - 1], false));
    printLine(1, getMenuItem((menuPage * itemsPerPage - 1), menus[(menuPage * itemsPerPage - 1) - 1], false));
  }

  if (pos == (menuPage * itemsPerPage))
  {
    Serial.println(getMenuItem((menuPage * itemsPerPage), menus[(menuPage * itemsPerPage) - 1], true));
    printLine(2, getMenuItem((menuPage * itemsPerPage), menus[(menuPage * itemsPerPage) - 1], true));
  }
  else
  {
    Serial.println(getMenuItem((menuPage * itemsPerPage), menus[(menuPage * itemsPerPage) - 1], false));
    printLine(2, getMenuItem((menuPage * itemsPerPage), menus[(menuPage * itemsPerPage) - 1], false));
  }
}

void actions(int menuPos)
{
  switch (menuPos)
  {
  case 1:
    Serial.println("T =  10c  Ph = 7,3");
    Serial.println("N = 20cm   M = off");
    printLine(1, "T=" + (String)ceil(getTemperature()) + "  Ph=" + (String)ceil(getPH()));
    printLine(2, "N=" + (String)ceil(getDistance()) + "   M=" + getRelayState());

    break;
  case 2:
    Serial.println("Temperatura");
    Serial.println("Boa " + (String)getTemperature() + "c");
    printLine(1, "Temperatura");
    printLine(2, "Boa 10c");
    break;
  case 3:
    Serial.println("Estado do motor");
    Serial.println("> OFF | ON");
    printLine(1, "Estado do motor");
    if (relayState)
    {
      printLine(2, " OFF | > ON");
      Serial.println(" OFF | > ON");
    }
    else
    {
      printLine(2, "> OFF |  ON");
      Serial.println("> OFF | ON");
    }

    break;
  case 4:
    printLine(1, "Nivel Da Agua");
    printLine(2, "" + (String)getDistance() + " cm");
    Serial.println(cm);
    break;
  case 5:
    printLine(1, "PH da Agua");
    printLine(2, " " + (String)getPH() + " ");
    break;
  case 6:
    Serial.println("Aj Wi-Fi");
    Serial.println("> Limpar config");
    printLine(1, "Aj Wi-Fi");
    printLine(2, "> Limpar config");
    break;
  case 7:
    Serial.println("IP");
    Serial.println("192.168.0.32");
    printLine(1, "IP");
    printLine(2, "192.168.0.32");
    break;

  default:
    Serial.println("Erro !");
    printLine(1, "Erro !");
    break;
  }
}

void btnClick()
{
  actions(menuPos);
  subMenuActive = true;
}

void btnDoubleClick()
{
  Serial.println(menuPos);
  if (menuPos == 3)
  {
    if (relayState == false)
    {
      toggleRelay("ON");
    }
    else
    {
      toggleRelay("OFF");
    }
  }
}

void btnLongPress()
{
  menu(menuPos, menuPage);
  subMenuActive = false;
}

void getDistance(String unitMode)
{
}

float getTemperature()
{
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  return tempC;
}

float getDistance()
{
  unsigned int distance = sonar.ping_cm();
  if (distance != 0)
  {
    // Serial.print(distance);
    // Serial.println("cm");
    return distance;
  }
}

void toggleRelay(String state)
{
  if (state == "ON")
  {
    digitalWrite(RELAY_PIN, HIGH);
    relayState = true;
  }
  else
  {
    digitalWrite(RELAY_PIN, LOW);
    relayState = false;
  }
}

String getRelayState()
{
  if (relayState == false)
  {
    return "OFF";
  }
  else
  {
    return "ON";
  }
}

void calibratePH()
{
  entrada_A0 = analogRead(PH_PIN);
  tensao = (entrada_A0 * 5.0) / 1024.0;
  Serial.println(tensao);
  delay(500);
}

float getPH()
{
  soma_tensao = 0; // Inicia soma_tensão em 0
  contagem = 0;    // Inicia a contagem em 0

  while (contagem < 10)
  {                                       // Executa enquanto contagem menor que 10
    tempo = millis();                     // Define o tempo em microssegundos
    entrada_A0 = analogRead(PH_PIN);      // Lê a entrada analógica
    tensao = (entrada_A0 * 5.0) / 1024.0; // Converte em tensão, o valor lido
    soma_tensao = (soma_tensao + tensao); // Soma a tensão anterior com a atual
    contagem++;                           // Soma 1 à variável de contagem
    // delay(5);                           // Aguarda para próxima leitura
  }

  media = soma_tensao / 10; // Calcula a média das leituras

  float valor_pH = -3.70 * media + valor_calibracao; // Calcula valor de pH
  // Serial.println(ceil(valor_pH));
  // delay(1000);
  return valor_pH;
}

void setup()
{
  Serial.begin(9600);
  Serial.println("START");
  pinMode(ANALOG_BTN, INPUT_PULLUP);
  lcd.init();

  menu(menuPos, menuPage);
  lcd.setBacklight(HIGH);

  // Setup for Ultrassonic sensor pins
  pinMode(SONIC_TRIG, OUTPUT);
  pinMode(SONIC_ECHO, INPUT);

  // Setup for Relay pins
  pinMode(RELAY_PIN, OUTPUT);

  // Setup for button functions
  button.attachClick(btnClick);
  button.attachDoubleClick(btnDoubleClick);
  button.attachLongPressStart(btnLongPress);
  sensors.begin();
}

void updateDisplay()
{
  if ((millis() - millisTarefa1) > 2000)
  {
    if (subMenuActive)
    {
      actions(menuPos);
    }
    else
    {
      menu(menuPos, menuPage);
    }
    millisTarefa1 = millis();
    Serial.println("Att");
  }
}

void loop()
{

  // lcd.setCursor(0, 0);
  // lcd.print("Arduino e Cia !!");
  // lcd.setCursor(0, 1);
  // lcd.print("LCD e modulo I2C");

  // checkBtnPress();

  // checkBtnPress();
  checkPos();
  button.tick();
  updateDisplay();
  // delay(1000);
  // lcd.setBacklight(LOW);
  // delay(1000);

  // button.handle();

  // delay(1000);

  /* code */

  // delay(1000);

  // delay(1000);
  // digitalWrite(RELAY_PIN, LOW);

  // Serial.println(getTemperature());

  // getPH();
  // calibratePH();
}
