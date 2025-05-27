#include <SoftwareSerial.h>
#include <Adafruit_Fingerprint.h>

constexpr int pinA0 = 14;
constexpr int pinA1 = 15;
constexpr int pinA2 = 16;
constexpr int pinA3 = 17;
constexpr int pinA4 = 18;
constexpr int pinA5 = 19;

static constexpr uintptr_t PORTD_ADDR = 0x2B;
static constexpr uintptr_t PIND_ADDR = 0x29;
static constexpr uintptr_t PORTB_ADDR = 0x25;
static constexpr uintptr_t PINB_ADDR = 0x23;
static constexpr uintptr_t PORTC_ADDR = 0x28;
static constexpr uintptr_t PINC_ADDR = 0x26;

constexpr int BUTTON_PIN = 2;
constexpr int MOSFET_PIN = pinA4;
constexpr int ALARM_PIN = 9;
constexpr int SW_UART_RX_PIN = 12, SW_UART_TX_PIN = 13;

volatile int acceptUnlockAttempt = 1;
volatile int buttonState = 0;
volatile int failedAttempts = 0;

SoftwareSerial mySerial(SW_UART_RX_PIN, SW_UART_TX_PIN);
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

void delay_millis(uint16_t ms) {
  uint32_t us = (uint32_t)ms * 1000;
  __asm__ volatile(
    "1:\n\t"
    "subi %A0, 1\n\t"
    "sbci %B0, 0\n\t"
    "sbci %C0, 0\n\t"
    "sbci %D0, 0\n\t"
    "rjmp 2f\n\t"
    "2:\n\t"
    "rjmp 3f\n\t"
    "3:\n\t"
    "rjmp 4f\n\t"
    "4:\n\t"
    "rjmp 5f\n\t"
    "5:\n\t"
    "rjmp 6f\n\t"
    "6:\n\t"
    "brne 1b\n\t"
    : "+r"(us)
  );
}

template<int PIN>
struct AtmegaPort {
private:
  static_assert(PIN >= 0 && PIN <= 19);
  static constexpr uintptr_t writePort = (PIN <= 7) ? PORTD_ADDR : ((PIN <= 13) ? PORTB_ADDR : PORTC_ADDR);
  static constexpr uintptr_t readPin = (PIN <= 7) ? PIND_ADDR : ((PIN <= 13) ? PINB_ADDR : PINC_ADDR);
  static constexpr uint8_t bit = (PIN <= 7) ? PIN : ((PIN <= 13) ? PIN - 8 : PIN - 14);

public:
  static int read() {
    return !!(*(volatile uint8_t*)readPin & (1 << bit));
  }
  static void write(int state) {
    // 0 = low, 1 = high, 2 = toggle
    if (state == 0)
      *(volatile uint8_t*)writePort &= ~(1 << bit);
    else if (state == 1)
      *(volatile uint8_t*)writePort |= (1 << bit);
    else
      *(volatile uint8_t*)writePort ^= (1 << bit);
  }
};

void openLock() {
  AtmegaPort<MOSFET_PIN>::write(1);
}

void closeLock() {
  AtmegaPort<MOSFET_PIN>::write(0);
}

bool isButtonPressed() {
  return AtmegaPort<BUTTON_PIN>::read();
}

void soundAlarm() {
  TCCR1A = (1 << COM1A1) | (1 << WGM11) | (1 << WGM10);
  TCCR1B = (1 << WGM12) | (1 << CS11);
  OCR1A = 0;
  while (1) {
    OCR1A = 100;
    delay_millis(1000);
    OCR1A = 20;
    delay_millis(1000);
  }
}

void enrollFingerprint(int id) {
  Serial.println("Place your finger...");
  int p = finger.getImage();
  while (p == FINGERPRINT_NOFINGER) {
    p = finger.getImage();
  }
  if (p != FINGERPRINT_OK) {
    Serial.print("Error: ");
    Serial.println(p);
    return;
  }
  p = finger.image2Tz(1);
  if (p != FINGERPRINT_OK) {
    Serial.print("Error: ");
    Serial.println(p);
    return;
  }
  Serial.println("Remove finger...");
  p = finger.getImage();
  while (p != FINGERPRINT_NOFINGER) {
    p = finger.getImage();
  }

  Serial.println("Place the same finger again...");
  p = finger.getImage();
  while (p == FINGERPRINT_NOFINGER) {
    p = finger.getImage();
  }
  if (p != FINGERPRINT_OK) {
    Serial.print("Error: ");
    Serial.println(p);
    return;
  }
  p = finger.image2Tz(2);
  if (p != FINGERPRINT_OK) {
    Serial.print("Error: ");
    Serial.println(p);
    return;
  }

  Serial.println("Creating model...");
  p = finger.createModel();
  if (p != FINGERPRINT_OK) {
    Serial.print("Error: ");
    Serial.println(p);
    return;
  }

  Serial.println("Storing model...");
  p = finger.storeModel(id);
  if (p != FINGERPRINT_OK) {
    Serial.print("Error: ");
    Serial.println(p);
    return;
  }

  Serial.println("Remove finger...");
  while (p != FINGERPRINT_NOFINGER) {
    p = finger.getImage();
  }
}

ISR(INT0_vect) {
  if (acceptUnlockAttempt && isButtonPressed()) {
    acceptUnlockAttempt = 0;
    buttonState = 1;
  }
}

void checkButtonUnlock() {
  if (buttonState == 1) {
    failedAttempts = 0;
    buttonState = 0;
    Serial.println("Button unlock");
    openLock();
    delay_millis(2000);
    closeLock();
    delay_millis(170);
    acceptUnlockAttempt = 1;
  }
}

void fingerprintUnlock() {
  if (!finger.verifyPassword())
    Serial.println("No sensor there");
  int p = finger.getImage();
  if (p == FINGERPRINT_OK) {
    p = finger.image2Tz();
    if (p != FINGERPRINT_OK) {
      Serial.println("Error");
      return;
    }
    p = finger.fingerSearch();
    if (p != FINGERPRINT_OK) {
      Serial.println("Fingerprint not found");
      failedAttempts++;
      if (failedAttempts == 10) {
        soundAlarm();
      }
      return;
    }
    Serial.print("Found ID #"); Serial.print(finger.fingerID);
    Serial.print(" with confidence of "); Serial.println(finger.confidence);

    failedAttempts = 0;
    acceptUnlockAttempt = 0;
    buttonState = 0;
    openLock();
    delay_millis(2000);
    closeLock();
    delay_millis(50);
    acceptUnlockAttempt = 1;
  }
}

void adminOperation() {
  String command = Serial.readStringUntil('\n');
  command.trim();
  Serial.println("Command received");
  if (command == "delete_all_fingerprints") {
    int p = finger.emptyDatabase();
    if (p != FINGERPRINT_OK) {
      Serial.println("Error");
      return;
    }
    Serial.println("Done");
  } else if (command.startsWith("delete_fingerprint")) {
    int indexOfSpace = command.indexOf(' ');
    String num = command.substring(indexOfSpace + 1);

    int id = num.toInt();
    int p = finger.deleteModel(id);
    if (p != FINGERPRINT_OK) {
      Serial.println("Error");
      return;
    }
    Serial.println("Done");
  } else if (command.startsWith("add_fingerprint")) {
    int indexOfSpace = command.indexOf(' ');
    String num = command.substring(indexOfSpace + 1);

    int id = num.toInt();
    enrollFingerprint(id);
  }
}

void setup() {
  // Button on pin 2 with pull up resistor
  DDRD &= ~(1 << PD2);
  PORTD |= (1 << PD2);
  // Configure interrupt on pin 2
  EICRA = (1 << ISC01); // Triggered on falling edge
  EIMSK = (1 << INT0); // Activate INT0
  sei(); // Enable interrupts

  // MOSFET on pin A4
  DDRC |= (1 << PC4);
  AtmegaPort<MOSFET_PIN>::write(0);

  // Alarm on pin 9
  DDRB |= (1 << PB1);
  AtmegaPort<ALARM_PIN>::write(0);

  // Set all other pins to a defined level (output, low)
  DDRD |= 0b11111000; // only pin 2 is used (although we don't touch pins 0 and 1)
  PORTD &= 0b00000111;
  DDRB |= 0b00100011; // only pins 9, 12 and 13 are used
  PORTB &= 0b11011100;
  DDRC |= 0b00101111; // only pin A4 is used
  PORTC &= 0b11010000;

  Serial.begin(9600);
  finger.begin(57600);
  while (!finger.verifyPassword()) {
    Serial.println("Did not find fingerprint sensor :(");
    delay_millis(1000);
  }
  Serial.println("Found fingerprint sensor");
  finger.LEDcontrol(false);
}

void loop() {
  checkButtonUnlock();
  fingerprintUnlock();
  if (Serial.available())
    adminOperation();

}