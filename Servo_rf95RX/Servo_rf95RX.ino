union int_bytes {
    uint8_t b[4];
    struct yp {
        uint16_t y;
        uint16_t p;
    }
    yp;
}
int_bytes;

#define DEBUG_OUT

#include <RH_RF95.h>
#include <SPI.h>

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2

// Change to 868.0 or other frequency, must match RX's freq!
#define RF95_FREQ 868.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED_PIN 13

const static uint8_t PITCH_SERVO_PIN = 5;
const static uint8_t YAW_SERVO_PIN = 6;

void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);

    while (!Serial) { }
    Serial.begin(9600);

    // manual reset
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    while (!rf95.init()) {
        Serial.println("LoRa radio init failed");
        while (1) { }
    }
    Serial.println("LoRa radio init OK!");

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("setFrequency failed");
        while (1) { }
    }
    Serial.print("Set frequency to: ");
    Serial.println(RF95_FREQ);

    // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf =
    // 128chips/symbol, CRC on

    // The default transmitter power is 13dBm, using PA_BOOST.
    // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter
    // pin, then you can set transmitter powers from 5 to 23 dBm:
    rf95.setTxPower(23, false);

    pinMode(PITCH_SERVO_PIN, OUTPUT);
    pinMode(YAW_SERVO_PIN, OUTPUT);

    Serial.println("Headtracker Receiver initialised!");
}

void loop() {
    if (rf95.available()) {
        // Should be a message for us now

        uint8_t len = sizeof(int_bytes.b);

        if (rf95.recv(int_bytes.b, & len)) {

            servoPulse(PITCH_SERVO_PIN, int_bytes.yp.p);
            servoPulse(YAW_SERVO_PIN, int_bytes.yp.y);

#ifdef DEBUG_OUT
            Serial.print(int_bytes.yp.y);
            Serial.print(' ');
            Serial.println(int_bytes.yp.p);
#endif

            /*
              pwm.setPWM(0, 0, (int)int_bytes.yp.y * 2.5 + 150);
              pwm.setPWM(1, 0, (int)int_bytes.yp.p * 1.75 + 150);
             */
        } else {
            Serial.println("Receive failed");
        }
    }
}

void servoPulse(int servoPin, int myAngle) {
    int pulseWidth = (myAngle * 11) + 500; // converts angle to microseconds
    digitalWrite(servoPin, HIGH);      // set servo high.
    delayMicroseconds(pulseWidth);     // wait a very small amount.
    digitalWrite(servoPin, LOW);       // set servo low.
    delay(10);                  // refresh cycle of typical servos.
}
