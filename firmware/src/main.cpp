/**
 * @file main.cpp
 * @brief Simple Virtual Oscilloscope Firmware for STM32F103C8T6 (PlatformIO)
 * 
 * Hardware Setup:
 * - MCU: STM32F103C8T6 (Blue Pill)
 * - PWM Pin: PA6 (TIM3_CH1) -> Connected to RC Filter
 * - ADC Pin: PA0 (ADC1_IN0) -> Connected to RC Filter Output
 * - USART1: PA9 (TX), PA10 (RX) -> Connected to CH340 USB-to-UART module (460800 bps)
 * - Onboard LED: PC13
 */

#include <Arduino.h>
#include "DdsWaveformGenerator.h"
#include <HardwareTimer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ─── Hardware & Communication Definitions ───
#define PWM_PIN       PA6
#define ADC_PIN       PA0
#define FRAME_SIZE    256       // Samples per packet
#define PWM_ARR       1799      // 72MHz / (0+1) / (1799+1) = 40kHz PWM carrier frequency (10.8 bit resolution)
#define LED_PIN       PC13
#define BUTTON_PIN    PB9

// ─── Communication Packet Formats ───
struct UploadFrame {
  uint8_t header[2];           // 0xAA, 0x55
  uint8_t waveType;            // 0: Sine, 1: Square, 2: Triangle
  uint32_t sampleRate;         // Current sample rate (Hz) (little endian)
  uint16_t nPoints;            // Frame size (little endian, fixed at 256)
  uint16_t data[FRAME_SIZE];   // ADC samples 0~4095 (little endian)
  uint8_t checksum;            // Cumulative sum of waveType, sampleRate, nPoints, data (lower 8 bits)
} __attribute__((packed));

struct ControlFrame {
  uint8_t header[2];           // 0x55, 0xAA
  uint8_t waveType;            // 0: Sine, 1: Square, 2: Triangle
  uint16_t frequency;          // Target wave frequency in Hz (little endian)
  uint16_t amplitude;          // Target wave amplitude in mV (0 ~ 1500)
  uint16_t offset;             // Target DC offset in mV (1000 ~ 2000)
  uint16_t sampleRate;         // Target sample rate in Hz (5000 / 10000 / 20000)
  uint8_t checksum;            // Sum of header[0..1], waveType, frequency, amplitude, offset, sampleRate
} __attribute__((packed));

// ─── Forward Declarations ───
void sampleISR();
void pwmISR();
void sendFrame();
void parseControlByte(uint8_t b);
void ADC_Init();
void updateOled();

// ─── OLED Configuration ───
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
uint32_t lastOledUpdate = 0;
bool oledEnabled = false;

// ─── Global State Variables ───
DdsWaveformGenerator waveGen;  // Custom DDS Waveform Generator
HardwareTimer *pwmTimer = nullptr;
HardwareTimer *sampleTimer = nullptr;

// Signal generation parameters (modified by incoming serial commands)
volatile uint8_t  waveType = 0;   
volatile float    waveFreq = 100.0f;  
volatile float    waveAmpl = 1.5f; 
volatile float    waveOffset = 1.65f; 
volatile uint32_t currentSampleRate = 5000; 

// ISR boundary cache variables (represented directly in PWM counts to avoid runtime float scaling)
volatile float currentMinPwm = 81.0f;    // (1.65 - 1.5) / 3.3 * 1799
volatile float currentMaxPwm = 1717.0f;  // (1.65 + 1.5) / 3.3 * 1799

// ADC buffers
volatile uint16_t adcBuf[FRAME_SIZE];
uint16_t oledSnapBuf[FRAME_SIZE];
volatile bool     frameReady = false;

// ─── Direct HAL ADC1 & DMA1 Initialization ───
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

void DMA_Init() {
  __HAL_RCC_DMA1_CLK_ENABLE();

  hdma_adc1.Instance = DMA1_Channel1;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode = DMA_NORMAL;
  hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
  HAL_DMA_Init(&hdma_adc1);

  // Link DMA to ADC
  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
}

void restartDmaSampling() {
  HAL_ADC_Stop_DMA(&hadc1);
  __HAL_DMA_CLEAR_FLAG(&hdma_adc1, DMA_FLAG_TC1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuf, FRAME_SIZE);
  frameReady = false;
}

void ADC_Init() {
  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  
  // Initialize DMA first
  DMA_Init();
  
  HAL_ADC_Init(&hadc1);

  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5; // Stable sampling time
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  // Start the ADC in DMA mode
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuf, FRAME_SIZE);
}

// ─── Signal Generation ISR (TIM3, Constant 40kHz) ───
void pwmISR() {
  float v = waveGen.next(waveType);
  int pwmVal = constrain((int)v, 0, PWM_ARR);
  TIM3->CCR1 = (uint32_t)pwmVal;
}

// ─── Sampling ISR (TIM2, Variable currentSampleRate) ───
void sampleISR() {
  // Trigger ADC hardware conversion immediately, DMA will automatically transfer the result
  if (!frameReady) {
    ADC1->CR2 |= ADC_CR2_SWSTART;
  }
}

// ─── I2C Bus Pull-up Detection ───
// Detects if the OLED's external pull-up resistors are present on PB6/PB7
bool checkI2cBus() {
  pinMode(PB6, OUTPUT);
  pinMode(PB7, OUTPUT);
  digitalWrite(PB6, LOW);
  digitalWrite(PB7, LOW);
  delay(5); // Hold low to discharge pin capacitance
  
  pinMode(PB6, INPUT);
  pinMode(PB7, INPUT);
  delayMicroseconds(150); // Wait for external pull-up resistors to pull lines high
  
  return (digitalRead(PB6) == HIGH) && (digitalRead(PB7) == HIGH);
}

// ─── Setup ───
void setup() {
  // Initialize onboard LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Turn on LED to indicate system starting
  
  // Initialize physical button pin
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Initialize Serial at 460800bps for high sample rate data rates
  Serial.begin(460800);

  // Detect if OLED display is physically connected by checking pull-ups
  if (checkI2cBus()) {
    // Initialize I2C1 with PB6/PB7 and SSD1306 OLED display
    Wire.setSCL(PB6);
    Wire.setSDA(PB7);
    Wire.begin();
    Wire.setClock(800000); // 800kHz I2C clock speed
    Wire.setTimeout(100);  // Set 100ms I2C timeout to prevent hardware hang
    
    if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      oledEnabled = true;
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(20, 15);
      display.println("OSCILLOSCOPE");
      display.setCursor(20, 30);
      display.println("Initializing...");
      display.display();
      delay(500); // Short delay for display boot visibility
    }
  } else {
    oledEnabled = false; // OLED disconnected, run without OLED
  }

  // Initialize ADC1 directly using HAL
  ADC_Init();


  waveGen.begin(waveFreq, 40000.0f, currentMinPwm, currentMaxPwm);

  // Initialize TIM3 for PWM generation on PA6 (40kHz, ARR=1799) and attach DDS update interrupt
  pwmTimer = new HardwareTimer(TIM3);
  pwmTimer->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PWM_PIN);
  pwmTimer->setOverflow(40000, HERTZ_FORMAT); // 40000 Hz = 40kHz
  pwmTimer->setCaptureCompare(1, 0, TICK_COMPARE_FORMAT);
  pwmTimer->attachInterrupt(pwmISR);
  pwmTimer->resume();

  // Initialize TIM2 for sampling ISR
  sampleTimer = new HardwareTimer(TIM2);
  sampleTimer->setOverflow(currentSampleRate, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  digitalWrite(LED_PIN, HIGH); // Turn off LED (active low) to show setup completed
}

// ─── Frame Transmission ───
void sendFrame() {
  static UploadFrame frame;
  
  // Toggle LED to show communications activity
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));

  frame.header[0] = 0xAA;
  frame.header[1] = 0x55;
  frame.waveType = waveType;
  frame.sampleRate = currentSampleRate;
  frame.nPoints = FRAME_SIZE;

  // Safe buffer snapshot
  noInterrupts();
  memcpy(frame.data, (void*)adcBuf, FRAME_SIZE * 2);
  interrupts();

  // Calculate Checksum: sum of waveType, sampleRate, nPoints, data
  uint8_t checksum = 0;
  uint8_t *ptr = (uint8_t *)&frame;
  for (uint32_t i = 2; i < sizeof(UploadFrame) - 1; i++) {
    checksum += ptr[i];
  }
  frame.checksum = checksum;

  // Send packet only if serial is open
  if (Serial) {
    Serial.write(ptr, sizeof(UploadFrame));
  }
}

// ─── Non-blocking State Machine Serial Command Parser ───
void parseControlByte(uint8_t b) {
  static uint8_t cmdBuf[sizeof(ControlFrame)];
  static uint8_t idx = 0;

  if (idx == 0) {
    if (b == 0x55) {
      cmdBuf[idx++] = b;
    }
  } else if (idx == 1) {
    if (b == 0xAA) {
      cmdBuf[idx++] = b;
    } else {
      idx = 0; // Header mismatch, reset alignment
    }
  } else {
    cmdBuf[idx++] = b;
    if (idx >= sizeof(ControlFrame)) {
      // Complete frame received, verify checksum
      ControlFrame *cmd = (ControlFrame *)cmdBuf;
      uint8_t calculatedChecksum = 0;
      for (uint32_t i = 0; i < sizeof(ControlFrame) - 1; i++) {
        calculatedChecksum += cmdBuf[i];
      }
      
      if (calculatedChecksum == cmd->checksum) {
        // Apply parameters safely
        noInterrupts();
        waveType = cmd->waveType;
        waveFreq = (float)cmd->frequency;
        
        // Convert mV parameters to Volts
        float ampVolts = (float)cmd->amplitude / 1000.0f;
        float offVolts = (float)cmd->offset / 1000.0f;

        // Safety constraint to prevent clipping at 0.0V and 3.3V
        ampVolts = constrain(ampVolts, 0.0f, 1.5f);
        offVolts = constrain(offVolts, 1.0f, 2.0f);
        if (offVolts - ampVolts < 0.1f) offVolts = ampVolts + 0.1f;
        if (offVolts + ampVolts > 3.2f) offVolts = 3.2f - ampVolts;

        waveAmpl = ampVolts;
        waveOffset = offVolts;

        // Re-calculate boundary PWM counts
        float pwmScale = (float)PWM_ARR / 3.3f;
        currentMinPwm = (waveOffset - waveAmpl) * pwmScale;
        currentMaxPwm = (waveOffset + waveAmpl) * pwmScale;

        // Apply sampling rate dynamically
        uint16_t targetFs = cmd->sampleRate;
        if (targetFs == 5000 || targetFs == 10000 || targetFs == 20000) {
          if (targetFs != currentSampleRate) {
            currentSampleRate = targetFs;
            sampleTimer->setOverflow(currentSampleRate, HERTZ_FORMAT);
          }
        }

        // Apply parameters to waveform generator (constant 40kHz DDS rate)
        waveGen.setFrequency(waveFreq, 40000.0f);
        waveGen.setRange(currentMinPwm, currentMaxPwm);

        restartDmaSampling();
        interrupts();
      }
      idx = 0; // Reset state for next command
    }
  }
}
void loop() {
  // Poll DMA Transfer Complete flag for Channel 1
  if (!frameReady && __HAL_DMA_GET_FLAG(&hdma_adc1, DMA_FLAG_TC1)) {
    __HAL_DMA_CLEAR_FLAG(&hdma_adc1, DMA_FLAG_TC1);
    frameReady = true;
  }

  // 1. Send data frame when ADC buffer is full
  if (frameReady) {
    sendFrame();
    uint32_t now = millis();
    bool doOled = oledEnabled && (now - lastOledUpdate >= 33); // 33ms throttle (~30 FPS)
    if (doOled) {
      memcpy(oledSnapBuf, (const void*)adcBuf, sizeof(oledSnapBuf));
    }
    // Restart DMA sampling for the next frame
    restartDmaSampling();

    // 1.1 OLED Update (Time-nested: executed while DMA is capturing in background)
    if (doOled) {
      lastOledUpdate = now;
      updateOled();
    }
  }

  // 2. Parse incoming serial controls byte by byte (completely non-blocking)
  while (Serial.available() > 0) {
    parseControlByte(Serial.read());
  }

  // 3. Scan physical button for waveform switching (software debounce)
  static uint32_t lastButtonPress = 0;
  if (digitalRead(BUTTON_PIN) == LOW) {
    if (millis() - lastButtonPress > 250) { // 250ms debounce
      noInterrupts();
      waveType = (waveType + 1) % 3;
      waveGen.setFrequency(waveFreq, 40000.0f);
      waveGen.setRange(currentMinPwm, currentMaxPwm);
      restartDmaSampling();
      interrupts();
      lastButtonPress = millis();
    }
  }
}

// ─── OLED Waveform & Parameter Display ───
void updateOled() {
  // Prepare the display buffer
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // Read from pre-DMA-restart snapshot buffer to prevent waveform tearing
  const uint16_t* localBuf = oledSnapBuf;

  // Calculate Vmax/Vmin from local buffer to show real-time metrics
  uint16_t maxVal = 0;
  uint16_t minVal = 4095;
  for (uint16_t i = 0; i < FRAME_SIZE; i++) {
    if (localBuf[i] > maxVal) maxVal = localBuf[i];
    if (localBuf[i] < minVal) minVal = localBuf[i];
  }
  float vMax = (float)maxVal * 3.3f / 4095.0f;
  float vMin = (float)minVal * 3.3f / 4095.0f;
  float vPp = vMax - vMin;

  // Draw UI Header Divider Line (at Y=11)
  display.drawFastHLine(0, 11, 128, SSD1306_WHITE);

  // Draw waveform (128 pixels wide, mapping 256 points downsampled by 2)
  // ADC range 0-4095 mapped to Y coordinates 63-12 (OLED Y=12 is top of wave area, Y=63 is bottom)
  int prevY = 63 - (int)((float)localBuf[0] * 51.0f / 4095.0f);
  prevY = constrain(prevY, 12, 63);
  
  for (int x = 1; x < 128; x++) {
    int y = 63 - (int)((float)localBuf[x * 2] * 51.0f / 4095.0f);
    y = constrain(y, 12, 63);
    display.drawLine(x - 1, prevY, x, y, SSD1306_WHITE);
    prevY = y;
  }

  // Overlay parameters on top header (Y=2)
  display.setCursor(2, 2);
  switch (waveType) {
    case 0: display.print("SIN"); break;
    case 1: display.print("SQR"); break;
    case 2: display.print("TRI"); break;
  }

  display.setCursor(28, 2);
  display.print((int)waveFreq);
  display.print("Hz");

  display.setCursor(85, 2);
  display.print(vPp, 2);
  display.print("V");

  // Commit the buffer to the screen (takes ~10ms at 800kHz I2C clock)
  display.display();
}
