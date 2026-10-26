#include <iostream>
#include <vector>
#include <unistd.h>
#include <iomanip>
#include <cstdint>
#include <algorithm>
#include <random>
#include <cmath>
using namespace std;

uint8_t expected_seq = 0;
uint32_t lost_frames = 0;
bool sequence_initialized = false;
constexpr size_t FRAME_SIZE = 10;
constexpr size_t PAYLOAD_OFFSET = 4;
constexpr size_t PAYLOAD_SIZE = 4;
constexpr uint8_t DATA_TYPE = 0x01;
constexpr int MAX_CONSECUTIVE_ERRORS = 6;

enum class ParseResult{
    COMPLETE,
    INCOMPLETE,
    INVALID,
    NO_SYNC
};

enum RxState {
    STATE_HUNT,
    STATE_LOCKED
};

struct RxContext {
    RxState state = STATE_HUNT;
    uint8_t expected_seq = 0;       // SEQ kỳ vọng ở frame tiếp theo
    uint32_t consecutive_errors = 0; // Đếm số frame lỗi liên tiếp để quyết định bỏ lock
};

struct BERStats {
    uint64_t total_bit_errors = 0;
    uint64_t total_bits_compared = 0;

    uint64_t frames_received = 0;
    uint64_t frames_crc_ok = 0;
    uint64_t frames_crc_bad = 0;
    uint64_t frames_seq_mismatch = 0;
};

double calculateBER(const BERStats &stats);
void updateBER(BERStats &stats, const vector<uint8_t> &expected, const vector<uint8_t> &received);
bool processRxFrame(const vector<uint8_t> &frame, RxContext &rx, BERStats &stats);
void feedParses(vector<uint8_t> &input_buffer, const vector<uint8_t> &incoming, RxContext &rx, BERStats &stats);
int findSYNC(const vector<uint8_t> &buffer);
ParseResult parseFrame(vector<uint8_t> &rx_buffer, vector<uint8_t> &frame);
bool validateFrame(const vector<uint8_t> &frame);
void processReceivedData(vector<uint8_t> &rx_buffer);
void appendAndProcess(vector<uint8_t> &rx_buffer, const vector<uint8_t> &incoming);
void checkSequence(uint8_t received_seq);
uint8_t popcount8(uint8_t x);
uint32_t countBitErrors(const vector<uint8_t> &expected, const vector<uint8_t> &received);
void simulateChannel(vector<uint8_t> &data, double ber, mt19937 &rng);
void runChannelSimulation(double channel_ber, int num_frames);

int PRBS7(uint16_t& state) {
    int output = (state >> 6) & 1;

    int feedback = ((state >> 6) & 1) ^ ((state >> 5) & 1);
    state = ((state << 1) & 0x7F) | feedback;
    return output;
}

void verifyPRBS7Cycle() {
    uint16_t initial_seed = 127;
    uint16_t state = initial_seed;
    int count_1 = 0;
    int steps = 0;

    for (int i = 1; i <= 127; i++) {
        int bit = PRBS7(state);
        if (bit == 1) count_1++;
        if (state == initial_seed) {
            steps = i;
            break;
        }
    }

    cout << "[A-3] So buoc lap de quay ve seed 127: " << steps << " (Ky vọng: 127)\n";
    cout << "[A-3] Tong so bit 1 trong chu ky: " << count_1 << " (Ky vọng: 64)\n\n";
}

uint16_t prbs_state = 0x7F;

vector<uint8_t> generatePayload(uint8_t seq){
    uint16_t state = (seq % 127) + 1;

    vector<uint8_t> payload(4,0);

    for(int i = 0; i < 32; i++){
        int bit = PRBS7(state);
        payload[i/8] |= static_cast<uint8_t>(bit << (7 - i % 8));
    }
    return payload;
}

uint16_t updateCRC16(uint16_t crc, uint8_t data){
    crc ^= (uint16_t(data) << 8);
    for(int i = 0; i < 8; i++){
        if(crc & 0x8000){
            crc = (crc << 1) ^ 0x1021;
        }else{
            crc = crc << 1;
        }
    }
    return crc;
}

uint16_t calculateFrameCRC(const uint8_t* frameData, int length){
    uint16_t crc = 0xFFFF; // Giá trị khởi tạo chuẩn
    
    for (int i = 0; i < length; i++) {
        crc = updateCRC16(crc, frameData[i]);
    }
    
    return crc;
}

int frameBit(const vector<uint8_t>& f, int i){
    int byte_index = i/8;
    int bit_index = 7 - (i%8);
    return (f[byte_index] >> bit_index) & 1;
}

vector<uint8_t> buildFrame(uint8_t type, uint8_t seq, const vector<uint8_t>& payload){
    //PAYLOAD 
    if(payload.size() != 4){
        return {};
    }
    vector<uint8_t> frame = {0xEB, 0x90, type, seq, payload[0], payload[1], payload[2], payload[3], 0, 0};
    uint16_t crc = calculateFrameCRC(frame.data() + 2, 6);

    frame[8] = (crc >> 8) & 0xFF;
    frame[9] = crc & 0xFF;
    return frame;
}

int findSYNC(const vector<uint8_t> &buffer){
    if(buffer.size() < 2){
        return -1;
    }

    for(size_t i = 0; i + 1 < buffer.size(); i++){
        if(buffer[i] == 0xEB && buffer[i + 1] == 0x90){
            return static_cast<int>(i);
        }
    }
    return -1;
}

void printPayload(const vector<uint8_t>& payload) {
    for (uint8_t byte : payload) {
        cout << hex
             << uppercase
             << setw(2)
             << setfill('0')
             << static_cast<int>(byte)
             << ' ';
    }

    cout << dec << '\n';
}

int main() {
    verifyPRBS7Cycle();

    uint16_t prbs_state = 0x7F;
    auto payload = generatePayload(prbs_state);
    vector<uint8_t> frame = buildFrame(0x01, 10, payload);
    cout << "8 bit dau tien cua frame(MSB first): ";
    for(int i = 0; i<8; i++){
        cout << frameBit(frame, i);
    }
    cout << endl;

    vector<uint8_t> rx_buffer;

    // Test 1: hai frame hợp lệ nối tiếp nhau (Đã truyền đủ 3 tham số: type, seq, payload)
    cout << "=== Test 1: two valid frames ===\n";
    auto f1 = buildFrame(0x01, 1, {0x12, 0x34, 0x56, 0x78});
    auto f2 = buildFrame(0x01, 2, {0xAB, 0xCD, 0xEF, 0x01});

    appendAndProcess(rx_buffer, f1);
    appendAndProcess(rx_buffer, f2);

    // Test 2: payload có chứa chuỗi EB 90.
    cout << "\n=== Test 2: SYNC bytes inside payload ===\n";
    auto f3 = buildFrame(0x01, 3, {0x11, 0xEB, 0x90, 0x22});
    appendAndProcess(rx_buffer, f3);

    // Test 3: frame lỗi CRC, theo sau bởi frame tốt.
    cout << "\n=== Test 3: corrupted frame then valid frame ===\n";
    auto bad = buildFrame(0x01, 4, {0x10, 0x20, 0x30, 0x40});
    bad[5] ^= 0x01; // Lật một bit payload, không cập nhật CRC.

    auto good = buildFrame(0x01, 5, {0xAA, 0xBB, 0xCC, 0xDD});

    vector<uint8_t> combined = bad;
    combined.insert(combined.end(), good.begin(), good.end());

    appendAndProcess(rx_buffer, combined);

    cout << "\nRemaining bytes in buffer: " << rx_buffer.size() << '\n';


    cout << endl;
    auto received = generatePayload(10);
    auto expected = generatePayload(10);
    auto p3 = generatePayload(11);
    received[0] ^= 0x01;

    cout << "SEQ = 10, lan 1: ";
    printPayload(received);

    cout << "SEQ = 10, lan 2: ";
    printPayload(expected);

    cout << "SEQ = 11:       ";
    printPayload(p3);
    uint32_t errors = countBitErrors(expected, received);

    cout << endl << errors << endl;

    int num_frames = 2000;
    cout << "=== TEST 1: Kênh lý tưởng (Không nhiễu) ===" << endl;
    runChannelSimulation(0.0, num_frames);
    cout << "\n=== TEST 2: Kênh nhiễu nhẹ (BER = 0.001) ===" << endl;
    runChannelSimulation(0.001, num_frames);
    cout << "\n=== TEST 3: Kênh nhiễu trung bình (BER = 0.01) ===" << endl;
    runChannelSimulation(0.01, num_frames);
    return 0;
}

ParseResult parseFrame(vector<uint8_t> &rx_buffer, vector<uint8_t> &frame){
    int position = findSYNC(rx_buffer);

    if(position == -1){
        if(!rx_buffer.empty() && rx_buffer.back() == 0xEB){
            rx_buffer = {0xEB};
        }else{
            rx_buffer.clear();
        }
        return ParseResult::NO_SYNC;
    }

    if(position > 0){
        // vector<uint8_t> copy_buffer;
        // copy_buffer.resize(rx_buffer.size());
        // copy(rx_buffer.begin(), rx_buffer.end(), copy_buffer.begin());
        rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + position);
    }

    if(rx_buffer.size() < 10){
        return ParseResult::INCOMPLETE;
    }
    
    frame.assign(rx_buffer.begin(), rx_buffer.begin() + 10);
    // rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + 10);
    return ParseResult::COMPLETE;
}

bool validateFrame(const vector<uint8_t> &frame){
     if(frame.size() != 10){
        return false;
    }
    if(frame[0] != 0xEB || frame[1] != 0x90 || frame[2] != 0x01){
        return false;
    }

    uint16_t calculated_crc = calculateFrameCRC(frame.data() + 2, 6);
    uint16_t received_crc = (static_cast<uint16_t>(frame[8]) << 8) | frame[9];

    return (calculated_crc == received_crc);
}

void processReceivedData(vector<uint8_t> &rx_buffer){
    vector<uint8_t> frame;
    while(true){
        ParseResult result = parseFrame(rx_buffer, frame);
        if(result != ParseResult::COMPLETE){
            break;
        }
        else{
            if(!validateFrame(frame)){
                cerr << "Invalid frame" << endl;
                rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + 1);
            }else{
                cout << "Valid frame" << " SEQ = " << static_cast<unsigned int>(frame[3]) << endl;
                rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + 10);
                checkSequence(frame[3]);
            }
        }
    }
}

void appendAndProcess(vector<uint8_t> &rx_buffer, const vector<uint8_t> &incoming){
    rx_buffer.insert(rx_buffer.end(), incoming.begin(), incoming.end());
    processReceivedData(rx_buffer);
}

void checkSequence(uint8_t received_seq) {
    if (!sequence_initialized) {
        expected_seq = received_seq;
        sequence_initialized = true;
    }

    uint8_t missing = static_cast<uint8_t>(received_seq - expected_seq);

    if (missing == 0) {
        cout << "SEQ OK: " << static_cast<unsigned int>(received_seq)
             << '\n';
    } else {
        lost_frames += missing;

        cout << "Missing " << static_cast<unsigned int>(missing)
             << " frame(s) before SEQ "
             << static_cast<unsigned int>(received_seq) << '\n';
    }

    // Frame hiện tại đã được xử lý; frame tiếp theo được mong đợi.
    expected_seq = static_cast<uint8_t>(received_seq + 1);
}

uint8_t popcount8(uint8_t x){
    uint8_t count = 0;
    while(x != 0){
        x &= static_cast<uint8_t>(x-1);
        ++count;
    }
    return count;
}

uint32_t countBitErrors(const vector<uint8_t> &expected, const vector<uint8_t> &received){
    uint32_t count = 0;
    if(expected.size() != 4 || received.size() != 4){
        return 0xFFFFFFFF;
    }
    for(int i = 0; i < expected.size(); i++){
        uint8_t errors = popcount8(expected[i] ^ received[i]);
        count += errors;
    }
    return count;
}

double calculateBER(const BERStats &stats){
    if(stats.total_bits_compared == 0){
        return 0.0;
    }
    return static_cast<double>(stats.total_bit_errors) / stats.total_bits_compared;
}

void updateBER(BERStats &stats, const vector<uint8_t> &expected, const vector<uint8_t> &received){
    if(expected.size() != 4 || received.size() != 4){
        return ;
    }
    stats.total_bits_compared += 32;
    stats.total_bit_errors += countBitErrors(expected, received);
}

bool processRxFrame(const vector<uint8_t> &frame, RxContext &rx, BERStats &stats){
    if(frame.size() != 10) return false;
    
    stats.frames_received++;
    bool crc_ok = validateFrame(frame);

    if(!crc_ok){
        stats.frames_crc_bad++;
        if(rx.state == STATE_LOCKED){
            rx.consecutive_errors++;
            if(rx.consecutive_errors >= MAX_CONSECUTIVE_ERRORS){
                rx.state = STATE_HUNT;
                rx.consecutive_errors = 0;
                cerr << "Reset to HUNT" << endl;
            }
        }
        return false; // CRC lỗi
    }

    stats.frames_crc_ok++;
    uint8_t received_seq = frame[3];
    vector<uint8_t> received_payload(frame.begin() + 4, frame.begin() + 8);

    if(rx.state == STATE_HUNT){
        rx.state = STATE_LOCKED;
        rx.expected_seq = static_cast<uint8_t>(received_seq + 1);
        rx.consecutive_errors = 0;
        auto expected_payload = generatePayload(received_seq);
        updateBER(stats, expected_payload, received_payload);
        return true;
    }

    if(received_seq == rx.expected_seq){
        rx.consecutive_errors = 0;
        auto expected_payload = generatePayload(received_seq);
        updateBER(stats, expected_payload, received_payload);
        rx.expected_seq = static_cast<uint8_t>(received_seq + 1);
    } else {
        stats.frames_seq_mismatch++;
        rx.consecutive_errors++;
        rx.expected_seq = static_cast<uint8_t>(received_seq + 1);
        if(rx.consecutive_errors >= MAX_CONSECUTIVE_ERRORS){
            rx.state = STATE_HUNT;
            rx.consecutive_errors = 0;
            cerr << "Reset to HUNT" << endl;
        }
    }
    return true;
}

void feedParses(vector<uint8_t> &input_buffer, const vector<uint8_t> &incoming, RxContext &rx, BERStats &stats){
    input_buffer.insert(input_buffer.end(), incoming.begin(), incoming.end());
    while(true){
        int position = findSYNC(input_buffer);
        if(position == -1){
            if(!input_buffer.empty() && input_buffer.back() == 0xEB){
                input_buffer = {0xEB};
            }else{
                input_buffer.clear();
            }
            return;
        }
        if(position > 0){
            input_buffer.erase(input_buffer.begin(), input_buffer.begin() + position);
        }

        if(input_buffer.size() < 10){
            return;
        }
        vector<uint8_t> frame;
        frame.assign(input_buffer.begin(), input_buffer.begin() + 10);
        bool success = processRxFrame(frame, rx, stats);
        if(success){
            input_buffer.erase(input_buffer.begin(), input_buffer.begin() + 10);
        }else{
            input_buffer.erase(input_buffer.begin());
        }
    }
}

//Channel Simulation
void simulateChannel(vector<uint8_t> &data, double ber, mt19937 &rng){
    bernoulli_distribution bit_error(ber);
    for(uint8_t &byte : data){
        for(int bit = 0; bit < 8; ++bit){
            if(bit_error(rng)){
                byte ^= static_cast<uint8_t>(1u << bit);
            }
        }
    }
}

void runChannelSimulation(double channel_ber, int num_frames){
    random_device rd;
    mt19937 rng(rd());

    uint64_t total_bits_compared = 0;
    uint64_t total_bit_errors = 0;
    uint32_t crc_bad_count = 0;
    uint32_t crc_ok_count = 0;

    for(int i = 0; i< num_frames; i++){
        uint8_t seq = static_cast<uint8_t>(i % 256);

        vector<uint8_t> payload = generatePayload(seq);
        vector<uint8_t> original_frame = buildFrame(0x01, seq, payload);
        vector<uint8_t> original_payload(original_frame.begin() + 4, original_frame.begin() + 8);
        vector<uint8_t> rx_frame = original_frame;
        simulateChannel(rx_frame, channel_ber, rng);

        if(validateFrame(rx_frame)){
            crc_ok_count++;
        }else{
            crc_bad_count++;
        }

        vector<uint8_t> rx_payload(rx_frame.begin() + 4, rx_frame.begin() + 8);
        total_bits_compared += 32;
        for(size_t b = 0; b < 4; ++b){
            uint8_t diff = original_payload[b] ^ rx_payload[b];
            while(diff != 0){
                total_bit_errors += (diff & 1);
                diff >>= 1;
            }
        }
    }

    double measured_ber = (total_bits_compared == 0) ? 0.0 : static_cast<double>(total_bit_errors)/total_bits_compared;
    double crc_bad_rate = static_cast<double>(crc_bad_count)/num_frames;

    cout << "\n==================================================\n";
    cout << " BÁO CÁO MÔ PHỎNG KÊNH (BER cài đặt = " << channel_ber << ", Số frame = " << num_frames << ")" << endl;
    cout << "==================================================\n";
    cout << "Tổng số bit so sánh        : " << total_bits_compared << endl;
    cout << "Tổng số bit lỗi phát hiện  : " << total_bit_errors << endl;
    cout << "BER thực nghiệm đo được    : " << fixed << setprecision(6) << measured_ber << endl;
    cout << "Số frame CRC OK            : " << crc_ok_count << endl;
    cout << "Số frame CRC Lỗi (Bad)     : " << crc_bad_count << endl;
    cout << "Tỷ lệ lỗi khung (PER)      : " << fixed << setprecision(2) << (crc_bad_rate * 100) << "%" << endl;
    cout << "==================================================\n";
}



