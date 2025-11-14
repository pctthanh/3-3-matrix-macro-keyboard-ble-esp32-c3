/*
  ESP32-C3 SuperMini MacroPad - BLE HID + OLED + Encoder + 3x3 Matrix
  - OLED 128x32 (SDA=GPIO8, SCL=GPIO9)
  - Matrix rows: 0,1,3  cols: 4,5,10
  - Encoder: A=20, B=21, SW=2
  - BLE HID via BleKeyboard (use a C3-compatible fork)
  - Main screen: 3 lines (Mode / Enc+Time / rotating love messages)
  - Overlay shows "<n> <name>" for 3s when key pressed
  - Zoom keys send small step (Ctrl+= / Ctrl+-), repeat-on-hold slow
  - EditVideo mode (mode 3): encoder sends J/L to move playhead
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BleKeyboard.h> // install C3-compatible fork

// ---------- Display ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define OLED_ADDR 0x3C
#define SDA_PIN 8
#define SCL_PIN 9
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------- Matrix pins ----------
const uint8_t rowPins[3] = {0, 1, 3};    // outputs
const uint8_t colPins[3] = {4, 5, 10};   // inputs pullup

// ---------- Encoder ----------
#define ENC_A 20
#define ENC_B 21
#define ENC_SW 2

// ---------- Timing / UI ----------
const unsigned long DEBOUNCE_MS = 30;
const unsigned long SCAN_INTERVAL_MS = 10;
const unsigned long OVERLAY_MS_KEY = 3000;
const unsigned long OVERLAY_MS_MODE = 1200;
const unsigned long KEY_REPEAT_HOLD_MS = 450;
const unsigned long KEY_REPEAT_INTERVAL_MS = 140;

unsigned long bootMillis;
bool keyOverlay = false;
unsigned long keyOverlayUntil = 0;
String overlayText = "";
unsigned long lastOverlaySetMs = 0;

// ---------- Modes ----------
const uint8_t MAX_MODE = 3;
uint8_t currentMode = 1;
const char* modeNames[MAX_MODE] = {
  "SolidWorks",
  "Photoshop",
  "EditVideo"
};

// ---------- Love rotating messages (line 3) ----------
const char* loveMsgs[] = {
  "I <3 U",
  "I MISS U",
  "I LOVE YOU",
  ">_<",
  ":))"
};
const int loveMsgCount = 5;
int loveMsgIndex = 0;
unsigned long lastLoveChange = 0;
const unsigned long loveInterval = 3000; // 3s

// ---------- Key names ----------
const char* keyNames[9] = {
  "Copy", "Paste", "Undo",
  "Redo", "Save", "Cut",
  "Zoom+", "Zoom-", "Enter"
};

// ---------- BLE Keyboard ----------
BleKeyboard bleKeyboard("C3-MacroPad", "Maker", 100);

// ---------- Matrix state ----------
bool matrixState[3][3];
unsigned long matrixLastChangeTime[3][3];
// tracking key press times / sent flags
unsigned long keyLastPressMs[9] = {0};
bool keyPressSent[9] = {false};

// ---------- Encoder quadrature ----------
volatile uint8_t encLastAB = 0;
volatile int16_t encTransitionAccum = 0;
long encoderValue = 0;
volatile long encDetentTotal = 0;

// ---------- Encoder button ----------
bool lastBtnRaw = HIGH;
unsigned long btnLastChangeMs = 0;
bool btnHandled = false;
const unsigned long BTN_LOCK_MS = 200;

// --------- Prototypes ----------
void processEncoderAccum();
void scanMatrix();
void sendKeyHidOnce(int keyIndex);
void sendEncoderHid();
void sendEncButtonHid();
void drawUI();
void drawLoveMessage();

// ---------- Encoder ISR ----------
void IRAM_ATTR encISR() {
  uint8_t a = digitalRead(ENC_A);
  uint8_t b = digitalRead(ENC_B);
  uint8_t cur = (a << 1) | b;
  static const int8_t tbl[16] = {
     0, -1,  1, 0,
     1,  0,  0,-1,
    -1,  0,  0, 1,
     0,  1, -1, 0
  };
  uint8_t idx = (encLastAB << 2) | cur;
  encTransitionAccum += tbl[idx & 0x0F];
  encLastAB = cur;
}

// process transitions into detents (4 transitions = 1 detent)
void processEncoderAccum() {
  noInterrupts();
  int16_t t = encTransitionAccum;
  encTransitionAccum = 0;
  interrupts();
  static int pending = 0;
  if (t != 0) {
    pending += t;
    while (pending >= 4) {
      encoderValue++;
      pending -= 4;
      encDetentTotal++;
    }
    while (pending <= -4) {
      encoderValue--;
      pending += 4;
      encDetentTotal--;
    }
  }
}

// ---------- Matrix scan ----------
void scanMatrix() {
  unsigned long now = millis();
  for (int r=0;r<3;r++) {
    for (int i=0;i<3;i++) digitalWrite(rowPins[i], (i==r)?LOW:HIGH);
    delayMicroseconds(30);
    for (int c=0;c<3;c++) {
      bool pressed = (digitalRead(colPins[c]) == LOW);
      int idx = r*3 + c;
      if (pressed != matrixState[r][c]) {
        if (matrixLastChangeTime[r][c] == 0) matrixLastChangeTime[r][c] = now;
        else if (now - matrixLastChangeTime[r][c] > DEBOUNCE_MS) {
          matrixState[r][c] = pressed;
          matrixLastChangeTime[r][c] = 0;
          if (pressed) {
            keyLastPressMs[idx] = now;
            keyPressSent[idx] = false;
            char buf[32];
            snprintf(buf, sizeof(buf), "%d %s", idx+1, keyNames[idx]);
            overlayText = String(buf);
            keyOverlay = true;
            keyOverlayUntil = now + OVERLAY_MS_KEY;
            lastOverlaySetMs = now;
          } else {
            // released
            keyPressSent[idx] = false;
            keyLastPressMs[idx] = 0;
          }
        }
      } else {
        matrixLastChangeTime[r][c] = 0;
      }
    }
  }
  for (int i=0;i<3;i++) digitalWrite(rowPins[i], HIGH);
}

// ---------- HID mapping for keys (Redo = Ctrl+Shift+Z) ----------
void sendKeyHidOnce(int keyIndex) {
  if (!bleKeyboard.isConnected()) return;
  switch (keyIndex) {
    case 0: // Copy Ctrl+C
      bleKeyboard.press(KEY_LEFT_CTRL);
      bleKeyboard.press('c');
      delay(40);
      bleKeyboard.releaseAll();
      break;
    case 1: // Paste Ctrl+V
      bleKeyboard.press(KEY_LEFT_CTRL);
      bleKeyboard.press('v');
      delay(40);
      bleKeyboard.releaseAll();
      break;
    case 2: // Undo Ctrl+Z
      bleKeyboard.press(KEY_LEFT_CTRL);
      bleKeyboard.press('z');
      delay(40);
      bleKeyboard.releaseAll();
      break;
    case 3: // Redo -> Ctrl+Shift+Z (common in editors)
      bleKeyboard.press(KEY_LEFT_CTRL);
      bleKeyboard.press(KEY_LEFT_SHIFT);
      bleKeyboard.press('z');
      delay(40);
      bleKeyboard.releaseAll();
      break;
    case 4: // Save Ctrl+S
      bleKeyboard.press(KEY_LEFT_CTRL);
      bleKeyboard.press('s');
      delay(40);
      bleKeyboard.releaseAll();
      break;
    case 5: // Cut Ctrl+X
      bleKeyboard.press(KEY_LEFT_CTRL);
      bleKeyboard.press('x');
      delay(40);
      bleKeyboard.releaseAll();
      break;
    case 6: // Zoom+ small step (Ctrl + '=')
      bleKeyboard.press(KEY_LEFT_CTRL);
      bleKeyboard.press('=');
      delay(35);
      bleKeyboard.releaseAll();
      break;
    case 7: // Zoom- small step (Ctrl + '-')
      bleKeyboard.press(KEY_LEFT_CTRL);
      bleKeyboard.press('-');
      delay(35);
      bleKeyboard.releaseAll();
      break;
    case 8: // Enter
      bleKeyboard.write(KEY_RETURN);
      break;
    default:
      break;
  }
}


// ---------- Encoder HID behavior (mode-aware) ----------
void sendEncoderHid() {
  if (!bleKeyboard.isConnected()) return;
  static long lastSent = 0;
  long cur = encoderValue;
  if (cur == lastSent) return;

  if (cur > lastSent) {
    // CW (Clockwise)
    // Always send Right Arrow, regardless of mode
    bleKeyboard.write(KEY_RIGHT_ARROW);
    Serial.println("Encoder -> ArrowRight (CW)");
  } else {
    // CCW (Counter-Clockwise)
    // Always send Left Arrow, regardless of mode
    bleKeyboard.write(KEY_LEFT_ARROW);
    Serial.println("Encoder -> ArrowLeft (CCW)");
  }
  lastSent = cur;
}

void sendEncButtonHid() {
  if (!bleKeyboard.isConnected()) return;
  // default: Enter
  bleKeyboard.write(KEY_RETURN);
}

// ---------- Love rotating messages ----------
void drawLoveMessage() {
  unsigned long now = millis();
  if (now - lastLoveChange >= loveInterval) {
    loveMsgIndex = (loveMsgIndex + 1) % loveMsgCount;
    lastLoveChange = now;
  }
  display.setTextSize(1);
  display.setCursor(0, 24);
  display.print(loveMsgs[loveMsgIndex]);
}

// ---------- UI draw ----------
void drawUI() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Line1: Mode
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("Mode: ");
  display.print(modeNames[currentMode-1]);

  // Line2: Enc + Time
  display.setCursor(0,12);
  display.print("Enc: ");
  display.print(encoderValue);

  unsigned long up = (millis() - bootMillis) / 1000;
  unsigned int hh = up / 3600;
  unsigned int mm = (up % 3600) / 60;
  unsigned int ss = up % 60;
  char ubuf[16];
  snprintf(ubuf, sizeof(ubuf), "%02u:%02u:%02u", hh, mm, ss);
  display.setCursor(70,12);
  display.print(ubuf);

  // Line3: rotating love message
  drawLoveMessage();

  // Overlay (temporary) — draw on middle area if active
  if (keyOverlay) {
    display.fillRect(0, 10, 128, 12, SSD1306_BLACK);
    display.setTextSize(1);
    display.setCursor(2,12);
    display.print(overlayText);
    int secLeft = max(0, (int)((keyOverlayUntil - millis()) / 1000));
    display.setCursor(100,12);
    display.print(secLeft);
    display.print("s");
  }

  display.display();
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(50);

  // OLED init
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 init failed");
    while (1) delay(500);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Matrix pins
  for (int i=0;i<3;i++) {
    pinMode(rowPins[i], OUTPUT);
    digitalWrite(rowPins[i], HIGH);
  }
  for (int j=0;j<3;j++) pinMode(colPins[j], INPUT_PULLUP);

  memset(matrixState, 0, sizeof(matrixState));
  memset(matrixLastChangeTime, 0, sizeof(matrixLastChangeTime));
  for (int i=0;i<9;i++) { keyLastPressMs[i] = 0; keyPressSent[i] = false; }

  // Encoder pins
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  uint8_t a = digitalRead(ENC_A);
  uint8_t b = digitalRead(ENC_B);
  encLastAB = (a<<1) | b;

  attachInterrupt(digitalPinToInterrupt(ENC_A), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encISR, CHANGE);

  bootMillis = millis();

  // BLE keyboard init
  bleKeyboard.begin();

  // initial overlay
  overlayText = "Ready";
  keyOverlay = true;
  keyOverlayUntil = millis() + 800;
  lastOverlaySetMs = millis();

  Serial.println("Setup done");
}

// ---------- Main loop ----------
void loop() {
  static unsigned long lastScan = 0;
  static unsigned long lastEncProcess = 0;
  unsigned long now = millis();

  // scan matrix
  if (now - lastScan >= SCAN_INTERVAL_MS) {
    lastScan = now;
    scanMatrix();
  }

  // handle key sends: send once per press + slow repeat for zoom keys
  for (int k=0;k<9;k++) {
    if (keyLastPressMs[k] != 0) {
      if (!keyPressSent[k]) {
        // send once immediately (if BLE connected)
        if (bleKeyboard.isConnected()) {
          sendKeyHidOnce(k);
          Serial.printf("Sent HID for key %d\n", k+1);
        } else {
          Serial.printf("Key %d pressed but BLE not connected\n", k+1);
        }
        keyPressSent[k] = true;
        keyLastPressMs[k] = now; // reuse as last send timestamp
      } else {
        // handle repeat on hold only for zoom keys (6 and 7)
        if ((k==6 || k==7) && (now - keyLastPressMs[k] >= KEY_REPEAT_HOLD_MS)) {
          if (now - keyLastPressMs[k] >= KEY_REPEAT_HOLD_MS + KEY_REPEAT_INTERVAL_MS) {
            if (bleKeyboard.isConnected()) sendKeyHidOnce(k);
            keyLastPressMs[k] = now;
          }
        }
      }
    }
  }

  // process encoder accum frequently
  if (now - lastEncProcess >= 8) {
    lastEncProcess = now;
    processEncoderAccum();
  }

  // encoder button handling single-action
  bool rawBtn = digitalRead(ENC_SW); // active low
  if (rawBtn != lastBtnRaw) {
    lastBtnRaw = rawBtn;
    btnLastChangeMs = now;
  } else {
    if (now - btnLastChangeMs > 40) {
      if (rawBtn == LOW && !btnHandled) {
        // press -> change mode once
        currentMode++;
        if (currentMode > MAX_MODE) currentMode = 1;
        overlayText = String("Mode: ") + modeNames[currentMode-1];
        keyOverlay = true;
        keyOverlayUntil = now + OVERLAY_MS_MODE;
        lastOverlaySetMs = now;
        btnHandled = true;
        // send encoder button HID (Enter) once
        if (bleKeyboard.isConnected()) {
          sendEncButtonHid();
          Serial.println("Encoder button HID sent");
        }
      } else if (rawBtn == HIGH) {
        if (btnHandled && (now - btnLastChangeMs > BTN_LOCK_MS)) btnHandled = false;
      }
    }
  }

  // encoder HID
  sendEncoderHid();

  // overlay expiry
  if (keyOverlay && now > keyOverlayUntil) keyOverlay = false;

  // love message rotates/draw
  drawUI();

  delay(8);
}
