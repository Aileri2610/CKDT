// ============================================================
// oct_firmware.ino  --  Optical Communication Tester
// DUNG CHUNG: Arduino Uno (AVR) va ESP32. Chon board trong Arduino IDE la du.
// Logic (PRBS-7, frame, CRC, HUNT/LOCKED, BER) khong doi giua hai board;
// chi phan "BOARD" (chan, timer, in so) khac nhau.
// ============================================================
#define RUN_SELF_TEST 0

// ============================================================
// BOARD: chan, ngat, hang so dac thu
// ============================================================
#if defined(ARDUINO_ARCH_ESP32)
  #define ISR_ATTR IRAM_ATTR
  constexpr uint8_t TX_PIN = 25;      // -> R7 (1k) -> bazo Q1
  constexpr uint8_t RX_PIN = 26;      // <- ngo ra LM393 (PHAI keo len 3V3, khong phai 5V)
#elif defined(ARDUINO_ARCH_AVR)
  #define ISR_ATTR
  constexpr uint8_t TX_PIN = 8;       // -> R7 (1k) -> bazo Q1
  constexpr uint8_t RX_PIN = 7;       // <- ngo ra LM393 (Uno chiu duoc keo len 5V)
#else
  #error "Board chua duoc ho tro (chi Uno/AVR va ESP32)"
#endif

constexpr uint32_t BIT_RATE      = 1000;                    // bit/s (Uno: giu <= ~5000)
constexpr uint32_t TICKS_PER_BIT = 4;                       // 4 ngat moi bit
constexpr uint32_t TICK_US       = 1000000UL / (BIT_RATE * TICKS_PER_BIT);
constexpr uint8_t  SAMPLE_TICK   = 2;                       // 2 = giua bit; thu 1 hoac 3 de dich pha
constexpr bool     RX_INVERT     = false;                   // true neu qua mach quang bi dao muc

// ============================================================
// 1. Hang so, enum, struct
// ============================================================
constexpr size_t  FRAME_SIZE = 10;
constexpr size_t  PAYLOAD_OFFSET = 4;
constexpr size_t  PAYLOAD_SIZE = 4;
constexpr uint8_t DATA_TYPE = 0x01;
constexpr int     SYNC_LOSS_THRESHOLD = 24;
constexpr int     MAX_BAD_SLOTS = 3;

enum RxState { STATE_HUNT, STATE_LOCKED };

struct RxContext {
    RxState  state = STATE_HUNT;
    uint8_t  expected_seq = 0;
    uint32_t bad_slots = 0;
};

struct BERStats {
    uint32_t total_bit_errors = 0;
    uint32_t total_bits_compared = 0;
    uint32_t frames_received = 0;
    uint32_t frames_crc_ok = 0;
    uint32_t frames_crc_bad = 0;
    uint32_t frames_seq_mismatch = 0;
    uint32_t lost_frames = 0;
    uint32_t seq_resyncs = 0;
    uint32_t lock_count = 0;
    uint32_t lock_losses = 0;
};

// ============================================================
// 2. Logic xu ly (khong dung Serial)
// ============================================================
int PRBS7(uint16_t& state) {
    int output = (state >> 6) & 1;
    int feedback = ((state >> 6) & 1) ^ ((state >> 5) & 1);
    state = ((state << 1) & 0x7F) | feedback;
    return output;
}

uint8_t popcount8(uint8_t x) {
    uint8_t count = 0;
    while (x != 0) { x &= static_cast<uint8_t>(x - 1); ++count; }
    return count;
}

uint32_t countBitErrors(const uint8_t* expected, const uint8_t* received, size_t len) {
    uint32_t count = 0;
    for (size_t i = 0; i < len; i++) count += popcount8(expected[i] ^ received[i]);
    return count;
}

uint16_t updateCRC16(uint16_t crc, uint8_t data) {
    crc ^= (uint16_t(data) << 8);
    for (int i = 0; i < 8; i++) {
        if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
        else crc = crc << 1;
    }
    return crc;
}

uint16_t calculateFrameCRC(const uint8_t* frameData, int length) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < length; i++) crc = updateCRC16(crc, frameData[i]);
    return crc;
}

void generatePayload(uint8_t seq, uint8_t* payloadOut) {
    uint16_t state = (seq % 127) + 1;
    for (int i = 0; i < 4; i++) payloadOut[i] = 0;
    for (int i = 0; i < 32; i++) {
        int bit = PRBS7(state);
        payloadOut[i / 8] |= static_cast<uint8_t>(bit << (7 - i % 8));
    }
}

void buildFrame(uint8_t type, uint8_t seq, const uint8_t* payload, uint8_t* frameOut) {
    frameOut[0] = 0xEB; frameOut[1] = 0x90;
    frameOut[2] = type; frameOut[3] = seq;
    frameOut[4] = payload[0]; frameOut[5] = payload[1];
    frameOut[6] = payload[2]; frameOut[7] = payload[3];
    frameOut[8] = 0; frameOut[9] = 0;
    uint16_t crc = calculateFrameCRC(frameOut + 2, 6);
    frameOut[8] = (crc >> 8) & 0xFF;
    frameOut[9] = crc & 0xFF;
}

bool validateFrame(const uint8_t* frame) {
    if (frame[0] != 0xEB || frame[1] != 0x90 || frame[2] != DATA_TYPE) return false;
    uint16_t calculated_crc = calculateFrameCRC(frame + 2, 6);
    uint16_t received_crc = (static_cast<uint16_t>(frame[8]) << 8) | frame[9];
    return (calculated_crc == received_crc);
}

uint32_t hammingDistance(const uint8_t* a, const uint8_t* b, size_t len) {
    uint32_t distance = 0;
    for (size_t i = 0; i < len; ++i) distance += popcount8(a[i] ^ b[i]);
    return distance;
}

bool processRxFrame(const uint8_t* frame, RxContext& rx, BERStats& stats) {
    const bool valid = validateFrame(frame);

    if (rx.state == STATE_HUNT) {
        if (!valid) return false;
        const uint8_t seq = frame[3];
        stats.frames_received++;
        stats.frames_crc_ok++;
        rx.state = STATE_LOCKED;
        rx.expected_seq = static_cast<uint8_t>(seq + 1);
        rx.bad_slots = 0;
        stats.lock_count++;
        return true;                      // khung khoa KHONG tinh vao BER (duoc chon vi sach)
    }

    stats.frames_received++;
    const uint8_t seq_used = valid ? frame[3] : rx.expected_seq;

    uint8_t expected_payload[PAYLOAD_SIZE];
    generatePayload(seq_used, expected_payload);
    uint8_t expected_frame[FRAME_SIZE];
    buildFrame(DATA_TYPE, seq_used, expected_payload, expected_frame);

    const uint32_t distance = hammingDistance(frame, expected_frame, FRAME_SIZE);

    if (distance > (uint32_t)SYNC_LOSS_THRESHOLD) {
        rx.bad_slots++;
        if (!valid) stats.frames_crc_bad++;
        else stats.frames_seq_mismatch++;
        if (rx.bad_slots >= (uint32_t)MAX_BAD_SLOTS) {
            rx.state = STATE_HUNT;
            rx.bad_slots = 0;
            stats.lock_losses++;
        }
        return false;
    }

    rx.bad_slots = 0;
    const uint8_t* received_payload = frame + PAYLOAD_OFFSET;
    stats.total_bits_compared += 32;
    stats.total_bit_errors += countBitErrors(expected_payload, received_payload, 4);

    if (valid) {
        stats.frames_crc_ok++;
        if (frame[3] != rx.expected_seq) {
            stats.frames_seq_mismatch++;
            uint8_t gap = static_cast<uint8_t>(frame[3] - rx.expected_seq);
            if (gap > 0 && gap <= 127) stats.lost_frames += gap;
            else if (gap > 127) stats.seq_resyncs++;
        }
    } else {
        stats.frames_crc_bad++;
    }

    rx.expected_seq = static_cast<uint8_t>(seq_used + 1);
    return valid;
}

// ============================================================
// 3. Bo thu muc bit (cua so truot 80 bit)
// ============================================================
struct BitReceiver {
    static constexpr int FRAME_BITS = 80;
    uint8_t window[FRAME_SIZE];
    int bitsSeen = 0;
    int bitCount = 0;

    BitReceiver() { reset(); }

    void pushBit(uint8_t bit) {
        bit &= 0x01;
        for (size_t i = 0; i < FRAME_SIZE - 1; i++)
            window[i] = static_cast<uint8_t>((window[i] << 1) | (window[i + 1] >> 7));
        window[FRAME_SIZE - 1] = static_cast<uint8_t>((window[FRAME_SIZE - 1] << 1) | bit);
    }
    bool hasSync() const { return window[0] == 0xEB && window[1] == 0x90; }
    void reset() { memset(window, 0, FRAME_SIZE); bitsSeen = 0; bitCount = 0; }

    void onBitReceived(uint8_t bit, RxContext& rx, BERStats& stats) {
        pushBit(bit);
        if (bitsSeen < FRAME_BITS) {
            bitsSeen++;
            if (bitsSeen < FRAME_BITS) return;
        }
        if (rx.state == STATE_HUNT) {
            bitCount = 0;
            if (!hasSync()) return;
            processRxFrame(window, rx, stats);
            if (rx.state == STATE_LOCKED) bitCount = 0;
            return;
        }
        bitCount++;
        if (bitCount < FRAME_BITS) return;
        processRxFrame(window, rx, stats);
        bitCount = 0;
    }
};

#if RUN_SELF_TEST
void runSelfTests() {
    Serial.println(F("--- CHAY TEST NOI BO ---"));
    // Dat cac test cua ban vao day.
    Serial.println(F("--- HOAN THANH TEST ---"));
}
#endif

// ============================================================
// 4. Thoi gian thuc: ngat timer + hang doi vong
// ============================================================
volatile uint8_t  rxBuffer[256];          // chi so uint8_t tu xoay vong 0..255
volatile uint8_t  rxHead = 0;
volatile uint8_t  rxTail = 0;
volatile uint32_t rxOverflow = 0;

volatile uint8_t  txFrame[FRAME_SIZE];
volatile uint8_t  nextFrame[FRAME_SIZE];
volatile bool     nextFrameReady = false;
volatile uint8_t  txBitIndex = 80;        // 80 = "chua phat gi", ngat se nap frame dau tien
volatile uint8_t  tickCount = 0;

void ISR_ATTR onTimer() {
    if (tickCount == 0) {
        if (txBitIndex >= 80 && nextFrameReady) {
            for (uint8_t i = 0; i < FRAME_SIZE; i++) txFrame[i] = nextFrame[i];
            txBitIndex = 0;
            nextFrameReady = false;
        }
        if (txBitIndex < 80) {
            uint8_t bitVal = (txFrame[txBitIndex >> 3] >> (7 - (txBitIndex & 7))) & 0x01;
            digitalWrite(TX_PIN, bitVal);
            txBitIndex++;
        } else {
            digitalWrite(TX_PIN, LOW);    // loop() khong kip dua frame: giu LED tat
        }
    }
    if (tickCount == SAMPLE_TICK) {
        uint8_t val = digitalRead(RX_PIN);
        if (RX_INVERT) val = !val;
        uint8_t nextHead = rxHead + 1;
        if (nextHead != rxTail) { rxBuffer[rxHead] = val; rxHead = nextHead; }
        else                    { rxOverflow++; }
    }
    if (++tickCount >= TICKS_PER_BIT) tickCount = 0;
}

#if defined(ARDUINO_ARCH_AVR)
// Timer1, che do CTC, chia 8 => 2 MHz => 2 dem moi micro giay
ISR(TIMER1_COMPA_vect) { onTimer(); }

void setupHardwareTimer() {
    static_assert(TICK_US >= 10 && TICK_US <= 32000, "TICK_US ngoai pham vi Timer1 (prescaler 8)");
    noInterrupts();
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11);
    OCR1A  = (F_CPU / 8UL / 1000000UL) * TICK_US - 1;
    TCNT1  = 0;
    TIMSK1 = (1 << OCIE1A);
    interrupts();
}
#else  // ESP32
hw_timer_t* timer = NULL;
void setupHardwareTimer() {
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    timer = timerBegin(1000000);
    timerAttachInterrupt(timer, &onTimer);
    timerAlarm(timer, TICK_US, true, 0);
  #else
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, TICK_US, true);
    timerAlarmEnable(timer);
  #endif
}
#endif

// ============================================================
// 5. Ung dung
// ============================================================
RxContext   rx;
BERStats    stats;
BitReceiver receiver;
uint8_t     txSeq = 0;
uint32_t    lastPrintTime = 0;

void printCsv(uint32_t now) {
    float ber = 0.0;
    if (stats.total_bits_compared > 0)
        ber = (float)stats.total_bit_errors / (float)stats.total_bits_compared;
    Serial.print(now / 1000);                Serial.print(',');
    Serial.print((uint32_t)BIT_RATE);        Serial.print(',');
    Serial.print(stats.total_bits_compared); Serial.print(',');
    Serial.print(stats.total_bit_errors);    Serial.print(',');
    Serial.print(ber, 6);                    Serial.print(',');
    Serial.print(stats.frames_crc_ok);       Serial.print(',');
    Serial.print(stats.frames_crc_bad);      Serial.print(',');
    Serial.print(stats.lost_frames);         Serial.print(',');
    Serial.print(stats.lock_count);          Serial.print(',');
    Serial.print(stats.lock_losses);         Serial.print(',');
    Serial.println(rxOverflow);
}

void setup() {
    Serial.begin(115200);
    pinMode(TX_PIN, OUTPUT);
    digitalWrite(TX_PIN, LOW);
    pinMode(RX_PIN, INPUT);               // comparator co dien tro keo len ngoai

#if RUN_SELF_TEST
    runSelfTests();
#endif

    Serial.println(F("t_s,rate,bits,errors,ber,crc_ok,crc_bad,lost,locks,losses,overflow"));
    setupHardwareTimer();                 // loop() se dua frame dau tien vao ngay
}

void loop() {
    // 1) RX: xu ly bit trong hang doi
    while (rxHead != rxTail) {
        uint8_t bit = rxBuffer[rxTail];
        rxTail++;
        receiver.onBitReceived(bit, rx, stats);
    }

    // 2) TX: dung frame ke tiep khi ngat da lay frame truoc
    if (!nextFrameReady) {
        uint8_t payload[PAYLOAD_SIZE];
        uint8_t frame[FRAME_SIZE];
        generatePayload(txSeq, payload);
        buildFrame(DATA_TYPE, txSeq, payload, frame);
        for (uint8_t i = 0; i < FRAME_SIZE; i++) nextFrame[i] = frame[i];
        txSeq++;
        nextFrameReady = true;
    }

    // 3) Moi giay in mot dong CSV
    uint32_t now = millis();
    if (now - lastPrintTime >= 1000) {
        printCsv(now);
        lastPrintTime = now;
    }
}
