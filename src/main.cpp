#include <Arduino.h>
#include <U8g2lib.h>
#include <vector>
#include <math.h>
#include <HardwareTimer.h>
#include <STM32FreeRTOS.h>

// Constants
const uint32_t interval = 100; // Display update interval

// My constants
SemaphoreHandle_t keyArrayMutex;
uint8_t keyArray[7];
std::vector<const char *> keysPressed;
bool firstTime = true;

// Step sizes for the notes from C to B
const int32_t stepSizes[] = {51076057, 54113197, 57330935, 60740010, 64351799, 68178356, 72232452, 76527617, 81078186, 85899346, 91007186, 96418756};
volatile int32_t currentStepSize;

volatile int8_t rotationValue = 6;

// Pin definitions
// Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

// Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

// Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

// Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

// Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

// Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

// Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value)
{
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN, value);
  digitalWrite(REN_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN, LOW);
}

void setRow(uint8_t rowIdx)
{
  digitalWrite(REN_PIN, LOW);

  digitalWrite(RA0_PIN, ((rowIdx & 0b00000001) >> 0));
  digitalWrite(RA1_PIN, ((rowIdx & 0b00000010) >> 1));
  digitalWrite(RA2_PIN, ((rowIdx & 0b00000100) >> 2));

  digitalWrite(REN_PIN, HIGH);
}

uint8_t readCols()
{
  digitalWrite(REN_PIN, HIGH);

  uint8_t c0 = digitalRead(C0_PIN);
  uint8_t c1 = digitalRead(C1_PIN);
  uint8_t c2 = digitalRead(C2_PIN);
  uint8_t c3 = digitalRead(C3_PIN);

  digitalWrite(REN_PIN, LOW);

  return c0 + (c1 << 1) + (c2 << 2) + (c3 << 3);
}

const char *appendKey()
{
  const char *findKeyArray[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};

  const char *neutralFace = "- _ -";
  const char *neutralValue = "";

  for (int k = 0; k < 12; k++)
  {
    if (!((0b1 << k) & (keyArray[0] + (keyArray[1] << 4) + (keyArray[2] << 8))))
    {
      neutralFace = "^ 0 ^";
      neutralValue = findKeyArray[k];
      break;
    }
    else
    {
      continue;
    }
  }

  // u8g2.drawStr(2, 30, neutralFace);
  // u8g2.drawStr(42, 30, neutralValue);

  return neutralValue;
}

const char *whichKey()
{
  const char *findKeyArray[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};

  const char *neutralValue = "";

  for (int k = 0; k < 12; k++)
  {
    if (!((0b1 << k) & (keyArray[0] + (keyArray[1] << 4) + (keyArray[2] << 8))))
    {
      neutralValue = findKeyArray[k];
      break;
    }
    else
    {
      continue;
    }
  }

  return neutralValue;
}

const char *whichFace()
{
  const char *findKeyArray[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};

  const char *neutralFace = "- _ -";

  for (int k = 0; k < 12; k++)
  {
    if (!((0b1 << k) & (keyArray[0] + (keyArray[1] << 4) + (keyArray[2] << 8))))
    {
      neutralFace = "^ 0 ^";
      break;
    }
    else
    {
      continue;
    }
  }

  // u8g2.drawStr(2, 30, neutralFace);
  // u8g2.drawStr(42, 30, neutralValue);

  return neutralFace;
}

void checkKeyState(int32_t localCurrentStepSize)
{
  // stepSizes
  const char *neutralValue = "";

  for (int k = 0; k < 12; k++)
  {
    if (!((0b1 << k) & (keyArray[0] + (keyArray[1] << 4) + (keyArray[2] << 8))))
    {
      localCurrentStepSize = stepSizes[k];
      break;
    }
    else
    {
      continue;
    }
  }

  // u8g2.drawStr(2, 30, "CS:" + char(localCurrentStepSize));
  // u8g2.drawStr(42, 30, neutralValue);

  __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);

  // return currentStepSize;
}

void knob3_turn(bool firstTime)
{
  static uint8_t previousA;
  static uint8_t previousB;
  static uint8_t previousTransistion;

  uint8_t currentA;
  uint8_t currentB;
  uint8_t currentTransistion;

  if (firstTime == true)
  {
    setRow(3);
    delayMicroseconds(3);
    previousA = readCols() & 0b00000001;
    previousB = readCols() & 0b00000010;
  }
  else
  {
    setRow(3);
    delayMicroseconds(3);
    currentA = readCols() & 0b00000001;
    currentB = readCols() & 0b00000010;

    uint8_t combinedPrev = previousA + (previousB << 1);
    uint8_t combinedCurr = currentA + (currentB << 1);

    if (((combinedPrev == 00) && (combinedCurr == 01)) || ((combinedPrev == 11) && (combinedCurr == 10)))
    {
      currentTransistion = 1;
    }
    else if (((combinedPrev == 01) && (combinedCurr == 00)) || ((combinedPrev == 10) && (combinedCurr == 11)))
    {
      currentTransistion = -1;
    }
  }

  previousA = currentA;
  previousB = currentB;
  previousTransistion = currentTransistion;

  __atomic_store_n(&rotationValue, rotationValue + currentTransistion, __ATOMIC_RELAXED);
}

void scanKeysTask(void *pvParameters)
{
  const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;

  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    uint32_t tempCurrentStepSize = 0;

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);

    for (int j = 0; j < 4; j++)
    {
      setRow(j);
      delayMicroseconds(3);
      uint8_t keys = readCols();
      keyArray[j] = keys;
      // u8g2.print(keys, HEX);
    }

    xSemaphoreGive(keyArrayMutex);

    const char *keyValueState = appendKey();

    knob3_turn(firstTime);

    if (firstTime == true)
    {
      firstTime == false;
    }

    checkKeyState(tempCurrentStepSize);
  }
}

void displayUpdateTask(void *pvParameters)
{
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;

  TickType_t xLastWakeTime = xTaskGetTickCount();

  static uint32_t next = millis();
  static uint32_t count = 0;

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    u8g2.clearBuffer();                  // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font
    u8g2.drawStr(2, 10, "Hello World!"); // write something to the internal memory
    u8g2.setCursor(2, 20);
    u8g2.print(rotationValue, DEC);

    u8g2.drawStr(2, 30, whichFace());
    u8g2.drawStr(42, 30, whichKey());

    u8g2.sendBuffer(); // transfer internal memory to the display
  }
}

void sampleISR()
{
  static int32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = phaseAcc >> 24;
  analogWrite(OUTR_PIN, Vout + 128);
}

void setup()
{

  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);

  noInterrupts();

  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  interrupts();
  // put your setup code here, to run once:

  // Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  // Initialise display
  setOutMuxBit(DRST_BIT, LOW); // Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH); // Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH); // Enable display power supply

  // Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
      scanKeysTask,     /* Function that implements the task */
      "scanKeys",       /* Text name for the task */
      64,               /* Stack size in words, not bytes */
      NULL,             /* Parameter passed into the task */
      2,                /* Task priority */
      &scanKeysHandle); /* Pointer to store the task handle */

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
      displayUpdateTask,     /* Function that implements the task */
      "displayUpdate",       /* Text name for the task */
      256,                   /* Stack size in words, not bytes */
      NULL,                  /* Parameter passed into the task */
      1,                     /* Task priority */
      &displayUpdateHandle); /* Pointer to store the task handle */

  keyArrayMutex = xSemaphoreCreateMutex();

  vTaskStartScheduler();
}

void loop()
{
  // put your main code here, to run repeatedly:

  // uint32_t tempCurrentStepSize = 0;

  // if (millis() > next)
  // {
  //   next += interval;

  //   // Update display
  //   u8g2.clearBuffer();                  // clear the internal memory
  //   u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font
  //   u8g2.drawStr(2, 10, "Hello World!"); // write something to the internal memory
  //   u8g2.setCursor(2, 20);

  //   u8g2.sendBuffer(); // transfer internal memory to the display

  //   // Toggle LED
  //   digitalToggle(LED_BUILTIN);
  // }
}