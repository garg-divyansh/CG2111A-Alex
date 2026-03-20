#include "packets.h"
#include "serial_driver.h"

static unsigned long lastTime = 0, currTime;

#define THRESHOLD 50   // stronger debounce

// =============================================================
// Packet helpers
// =============================================================

static void sendResponse(TResponseType resp, uint32_t param) {
    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command    = resp;
    pkt.params[0]  = param;
    sendFrame(&pkt);
}

static void sendStatus(TState state) {
    sendResponse(RESP_STATUS, (uint32_t)state);
}

// =============================================================
// E-Stop state machine
// =============================================================

volatile TState buttonState = STATE_RUNNING;
volatile bool   stateChanged = false;

volatile static bool flag = false;
volatile static int count = 0;

ISR(INT3_vect) {
    currTime = millis();

    if(currTime - lastTime > THRESHOLD) {
        bool buttonpressed = (PIND & 0b00001000) != 0;

        if(buttonpressed && buttonState == STATE_RUNNING) {
            buttonState = STATE_STOPPED;
            stateChanged = true;
        }
        else if(!buttonpressed && buttonState == STATE_STOPPED) {
            if(count == 1 || flag) {
                buttonState = STATE_RUNNING;
                stateChanged = true;
                count = 0;
                flag = false;
            } else {
                count++;
            }
        }
        lastTime = currTime;
    }
}

// =============================================================
// Color sensor (TCS3200)
// =============================================================

static void redMode() {
    PORTA &= 0b11110011;
}

static void blueMode() {
    PORTA &= 0b11111011;
    PORTA |= 0b00001000;
}

static void greenMode() {
    PORTA &= 0b11110011;
    PORTA |= 0b00001100;
}

volatile static uint32_t frequencyCounter = 0;
volatile static bool colorWindowDone = false;

// --- Timer5 compare ISR (100 ms window)
ISR(TIMER5_COMPA_vect) {
    TCCR5B = 0;                     // stop timer
    EIMSK &= ~(1 << INT2);          // disable INT2 safely
    colorWindowDone = true;
}

// --- External interrupt (sensor pulses)
ISR(INT2_vect) {
    frequencyCounter++;
}

// --- Measure frequency
static uint32_t getFrequency() {
    frequencyCounter = 0;
    colorWindowDone = false;

    TCNT5 = 0;

    // Enable INT2 (sensor pulse counting)
    EIMSK |= (1 << INT2);

    // Start Timer5 (CTC mode already set)
    TCCR5B |= (1 << CS52);   // prescaler 256

    while(!colorWindowDone);  // wait 100 ms

    return frequencyCounter * 10;  // convert to Hz
}

// --- Read RGB
static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
    PORTA &= 0b11111101;
    PORTA |= 0b00000001;

    redMode();
    *r = getFrequency();

    greenMode();
    *g = getFrequency();

    blueMode();
    *b = getFrequency();
}

// =============================================================
// Command handler
// =============================================================

static int speed = 200;
static char lastCmd = 'x';

static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {

        case COMMAND_ESTOP:
            cli();
            buttonState  = STATE_STOPPED;
            stateChanged = false;
            flag = true;
            sei();

            sendStatus(STATE_STOPPED);
            break;

        case COMMAND_COLOR:
        {
            TPacket pkt;
            memset(&pkt, 0, sizeof(pkt));

            pkt.packetType = PACKET_TYPE_RESPONSE;
            pkt.command    = RESP_COLOR;

            readColorChannels(&(pkt.params[0]),
                              &(pkt.params[1]),
                              &(pkt.params[2]));

            sendFrame(&pkt);
            break;
        }

        case COMMAND_MOVE:

            if(cmd->data[0] == 'w') {
                speed = 200; forward(speed); lastCmd = 'w';
            }
            else if(cmd->data[0] == 'a') {
                speed = 200; ccw(speed); lastCmd = 'a';
            }
            else if(cmd->data[0] == 's') {
                speed = 200; backward(speed); lastCmd = 's';
            }
            else if(cmd->data[0] == 'd') {
                speed = 200; cw(speed); lastCmd = 'd';
            }
            else if(cmd->data[0] == 'x') {
                stop(); lastCmd = 'x'; speed = 200;
            }
            else if(cmd->data[0] == '+') {
                if(lastCmd != 'x') {
                    speed = constrain(speed + 10, 80, 255);
                }
            }
            else if(cmd->data[0] == '-') {
                if(lastCmd != 'x') {
                    speed = constrain(speed - 10, 80, 255);
                }
            }

            if((cmd->data[0] == '+' || cmd->data[0] == '-') && lastCmd != 'x') {
                switch(lastCmd) {
                    case 'w': forward(speed); break;
                    case 'a': ccw(speed); break;
                    case 's': backward(speed); break;
                    case 'd': cw(speed); break;
                }

                TPacket pkt;
                memset(&pkt, 0, sizeof(pkt));
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command = RESP_OK;
                snprintf(pkt.data, sizeof(pkt.data), "Speed: %d", speed);
                sendFrame(&pkt);
            }
            break;
    }
}

// =============================================================
// Setup & Loop
// =============================================================

void setup() {

#if USE_BAREMETAL_SERIAL
    usartInit(103);
#else
    Serial.begin(9600);
#endif

    // --- Button (INT3)
    DDRD = 0b00000000;

    // INT3 = any logical change
    EICRA = 0b01110000;

    // --- Sensor interrupt (INT2 rising edge)
    EICRA |= (1 << ISC21) | (1 << ISC20);

    // Enable INT3 only initially
    EIMSK = (1 << INT3);

    // --- Timer5 setup (CTC mode)
    TCCR5A = 0;
    TCCR5B = (1 << WGM52);   // CTC mode

    TIMSK5 = (1 << OCIE5A);  // enable compare interrupt

    TCNT5 = 0;
    OCR5A = 6250;            // 100 ms window

    // --- Color sensor pins
    DDRA = 0b00001111;

    sei();
}

void loop() {

    if (stateChanged) {
        cli();
        TState state = buttonState;
        stateChanged = false;
        sei();
        sendStatus(state);
    }

    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }
}