#include <iostream>
#include <vector>
#include <cstdint>
#include <cassert>
#include <iomanip>
#include <string>
#include <algorithm>
using namespace std;
// ============================================================
// 0. CAU HINH VA CAU TRUC DU LIEU
// ============================================================
constexpr size_t HEADER_SIZE = 4;
constexpr size_t PAYLOAD_SIZE = 4;
constexpr size_t CRC_SIZE = 2;
constexpr size_t FRAME_SIZE = 10;
constexpr int MAX_CONSECUTIVE_ERRORS = 6; 
enum RxState {
    STATE_HUNT,
    STATE_LOCKED
};
struct RxContext {
    RxState state = STATE_HUNT;
    uint8_t expected_seq = 0;
    uint32_t consecutive_errors = 0;
};
struct BERStats {
    uint32_t frames_received = 0;
    uint32_t frames_crc_ok = 0;
    uint32_t frames_crc_bad = 0;
    uint32_t frames_seq_mismatch = 0;
    uint64_t total_bits_compared = 0;
    uint64_t total_bit_errors = 0;
};
// ============================================================
// 1. CRC-16/CCITT-FALSE
// ============================================================
uint16_t calculateCRC16(const vector<uint8_t>& data) {
    uint16_t crc = 0xFFFF;
    for (uint8_t byte : data) {
        crc ^= static_cast<uint16_t>(byte) << 8;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x8000) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}
// ============================================================
// 2. PRBS-7
// Quy uoc:
// - LFSR 7 bit, seed khac 0
// - Output = bit 6 cua trang thai hien tai
// - Feedback = bit 6 XOR bit 5
// - Dich trai, dua feedback vao bit 0
// - Dong goi MSB-first
// ============================================================
int PRBS7(uint16_t& state) {
    int output = (state >> 6) & 1;
    int feedback = ((state >> 6) & 1) ^ ((state >> 5) & 1);
    state = static_cast<uint16_t>(((state << 1) & 0x7F) | feedback);
    return output;
}
vector<uint8_t> generatePayload(uint8_t seq) {
    vector<uint8_t> payload(PAYLOAD_SIZE, 0);
    uint16_t state = (seq % 127) + 1;
    for (int i = 0; i < 32; ++i) {
        int bit = PRBS7(state);
        payload[i / 8] |= static_cast<uint8_t>(bit << (7 - i % 8));
    }
    return payload;
}
// ============================================================
// 3. BER
// ============================================================
void updateBER(BERStats& stats, const vector<uint8_t>& expected,
               const vector<uint8_t>& received) {
    if (expected.size() != received.size()) {
        return;
    }
    stats.total_bits_compared += expected.size() * 8;
    for (size_t i = 0; i < expected.size(); ++i) {
        uint8_t diff = expected[i] ^ received[i];
        // Dem so bit 1 trong diff
        while (diff != 0) {
            stats.total_bit_errors += diff & 1;
            diff >>= 1;
        }
    }
}
// ============================================================
// 4. FRAME TX / CRC VALIDATION
// ============================================================
bool validateFrame(const vector<uint8_t>& frame) {
    if (frame.size() != FRAME_SIZE) return false;
    if (frame[0] != 0xEB || frame[1] != 0x90) {
        return false;
    }
    if (frame[2] != 0x01) return false;
    // CRC bao gom TYPE, SEQ va PAYLOAD: byte 2..7
    vector<uint8_t> crc_data(frame.begin() + 2, frame.begin() + 8);
    uint16_t calculated_crc = calculateCRC16(crc_data);
    uint16_t received_crc = (static_cast<uint16_t>(frame[8]) << 8) |
                            static_cast<uint16_t>(frame[9]);
    return calculated_crc == received_crc;
}
vector<uint8_t> buildFrame(uint8_t seq, const vector<uint8_t>& payload) {
    assert(payload.size() == PAYLOAD_SIZE);
    vector<uint8_t> frame = {0xEB, 0x90, 0x01, seq};
    frame.insert(frame.end(), payload.begin(), payload.end());
    vector<uint8_t> crc_data(frame.begin() + 2, frame.end());
    uint16_t crc = calculateCRC16(crc_data);
    // CRC high byte first
    frame.push_back(static_cast<uint8_t>(crc >> 8));
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    return frame;
}
vector<uint8_t> buildFrame(uint8_t seq) {
    return buildFrame(seq, generatePayload(seq));
}
// Dung trong test: tinh lai CRC sau khi sua payload
void recomputeFrameCRC(vector<uint8_t>& frame) {
    assert(frame.size() == FRAME_SIZE);
    vector<uint8_t> crc_data(frame.begin() + 2, frame.begin() + 8);
    uint16_t crc = calculateCRC16(crc_data);
    frame[8] = static_cast<uint8_t>(crc >> 8);
    frame[9] = static_cast<uint8_t>(crc & 0xFF);
}
// ============================================================
// 5. RX
//
// Chinh sach BER:
// - CRC sai: loai frame, khong tinh BER.
// - HUNT + CRC dung: bat dau LOCKED, tinh BER.
// - LOCKED + SEQ dung: tinh BER.
// - LOCKED + SEQ sai: ghi nhan mismatch, khong tinh BER,
//   re-anchor expected_seq theo SEQ vua nhan.
// ============================================================
bool processRxFrame(const vector<uint8_t>& frame, RxContext& rx, BERStats& stats) {
    if (frame.size() != FRAME_SIZE) {
        return false;
    }
    stats.frames_received++;
    if (!validateFrame(frame)) {
        stats.frames_crc_bad++;
        if (rx.state == STATE_LOCKED) {
            rx.consecutive_errors++;
            if (rx.consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
                rx.state = STATE_HUNT;
                rx.consecutive_errors = 0;
            }
        }
        return false;
    }
    stats.frames_crc_ok++;
    uint8_t received_seq = frame[3];
    vector<uint8_t> received_payload(frame.begin() + 4, frame.begin() + 8);
    vector<uint8_t> expected_payload = generatePayload(received_seq);
    if (rx.state == STATE_HUNT) {
        // Frame CRC dung duoc dung de bat dau dong bo
        rx.state = STATE_LOCKED;
        rx.expected_seq = static_cast<uint8_t>(received_seq + 1);
        rx.consecutive_errors = 0;
        updateBER(stats, expected_payload, received_payload);
        return true;
    }
    if (received_seq == rx.expected_seq) {
        rx.consecutive_errors = 0;
        rx.expected_seq = static_cast<uint8_t>(received_seq + 1);
        updateBER(stats, expected_payload, received_payload);
    } else {
        stats.frames_seq_mismatch++;
        rx.consecutive_errors++;
        // Re-anchor theo frame moi nhan
        rx.expected_seq = static_cast<uint8_t>(received_seq + 1);
        if (rx.consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
            rx.state = STATE_HUNT;
            rx.consecutive_errors = 0;
        }
    }
    return true;
}
// ============================================================
// 6. STREAM PARSER
// ============================================================
int findSYNC(const vector<uint8_t>& buffer) {
    if (buffer.size() < 2) return -1;
    for (size_t i = 0; i + 1 < buffer.size(); ++i) {
        if (buffer[i] == 0xEB &&
            buffer[i + 1] == 0x90) {
            return static_cast<int>(i);
        }
    }
    return -1;
}
void feedParser(vector<uint8_t>& input_buffer, const vector<uint8_t>& incoming,
                RxContext& rx, BERStats& stats) {
    input_buffer.insert(input_buffer.end(), incoming.begin(), incoming.end());
    while (true) {
        int position = findSYNC(input_buffer);
        if (position == -1) {
            // Giu lai EB cuoi cung vi byte 90 co the den sau
            if (!input_buffer.empty() &&
                input_buffer.back() == 0xEB) {
                input_buffer = {0xEB};
            } else {
                input_buffer.clear();
            }
            return;
        }
        if (position > 0) {
            input_buffer.erase(input_buffer.begin(), input_buffer.begin() + position);
        }
        if (input_buffer.size() < FRAME_SIZE) {
            return;
        }
        vector<uint8_t> frame(input_buffer.begin(), input_buffer.begin() + FRAME_SIZE);
        bool success = processRxFrame(frame, rx, stats);
        if (success) {
            input_buffer.erase(input_buffer.begin(), input_buffer.begin() + FRAME_SIZE);
        } else {
            // Khong bo qua ca 10 byte:
            // SYNC that co the nam ben trong candidate bi loi.
            input_buffer.erase(input_buffer.begin());
        }
    }
}
// ============================================================
// 7. TEST UTILITIES
// ============================================================
int test_count = 0;
void pass(const string& name) {
    ++test_count;
    cout << "[PASS] " << name << '\n';
}
void resetAll(RxContext& rx, BERStats& stats, vector<uint8_t>& buffer) {
    rx = RxContext{};
    stats = BERStats{};
    buffer.clear();
}
vector<uint8_t> extractPayload(const vector<uint8_t>& frame) {
    return vector<uint8_t>(frame.begin() + 4, frame.begin() + 8);
}
// ============================================================
// 8. TEST SUITE
// ============================================================
void runUnitTests() {
    test_count = 0;
    cout << "\n========================================\n";
    cout << " OPTICAL COMMUNICATION TESTER - TESTS\n";
    cout << "========================================\n";
    RxContext rx;
    BERStats stats;
    vector<uint8_t> buffer;
    // --------------------------------------------------------
    // GROUP A: CRC
    // --------------------------------------------------------
    {
        // CRC-16/CCITT-FALSE known check value
        // "123456789" -> 0x29B1
        vector<uint8_t> data = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
        assert(calculateCRC16(data) == 0x29B1);
        pass("A1 - CRC known vector 123456789 = 0x29B1");
    }
    {
        vector<uint8_t> empty;
        assert(calculateCRC16(empty) == 0xFFFF);
        pass("A2 - CRC empty input");
    }
    // --------------------------------------------------------
    // GROUP B: PRBS-7
    // --------------------------------------------------------
    {
        for (int seq = 0; seq <= 255; ++seq) {
            auto payload = generatePayload(static_cast<uint8_t>(seq));
            assert(payload.size() == PAYLOAD_SIZE);
        }
        pass("B1 - PRBS payload luon co 4 byte");
    }
    {
        for (int seq = 0; seq <= 255; ++seq) {
            auto p1 = generatePayload(static_cast<uint8_t>(seq));
            auto p2 = generatePayload(static_cast<uint8_t>(seq));
            assert(p1 == p2);
        }
        pass("B2 - PRBS deterministic voi cung SEQ");
    }
    {
        uint16_t state = 1;
        for (int i = 0; i < 1000; ++i) {
            int bit = PRBS7(state);
            assert(bit == 0 || bit == 1);
            assert(state > 0 && state <= 0x7F);
        }
        pass("B3 - LFSR khong thoat khoi 7-bit va khong ve seed 0");
    }
    // --------------------------------------------------------
    // GROUP C: FRAME BUILD / VALIDATION
    // --------------------------------------------------------
    {
        auto frame = buildFrame(10);
        assert(frame.size() == FRAME_SIZE);
        assert(frame[0] == 0xEB);
        assert(frame[1] == 0x90);
        assert(frame[2] == 0x01);
        assert(frame[3] == 10);
        assert(validateFrame(frame));
        pass("C1 - Tao frame dung format va CRC");
    }
    {
        auto frame = buildFrame(10);
        frame[0] = 0x00;
        assert(!validateFrame(frame));
        frame = buildFrame(10);
        frame[1] = 0x00;
        assert(!validateFrame(frame));
        pass("C2 - Phat hien SYNC sai");
    }
    {
        auto frame = buildFrame(10);
        frame[2] = 0x02;
        recomputeFrameCRC(frame);
        assert(!validateFrame(frame));
        pass("C3 - Phat hien TYPE khong ho tro");
    }
    {
        auto frame = buildFrame(10);
        frame[5] ^= 0x01;
        assert(!validateFrame(frame));
        pass("C4 - Phat hien payload bi sua ma khong cap nhat CRC");
    }
    {
        auto frame = buildFrame(10);
        frame[8] ^= 0x01;
        assert(!validateFrame(frame));
        frame = buildFrame(10);
        frame.pop_back();
        assert(!validateFrame(frame));
        pass("C5 - Phat hien CRC sai va frame sai do dai");
    }
    // --------------------------------------------------------
    // GROUP D: BER COUNTER
    // --------------------------------------------------------
    {
        stats = BERStats{};
        vector<uint8_t> expected(4, 0x00);
        vector<uint8_t> received = expected;
        updateBER(stats, expected, received);
        assert(stats.total_bits_compared == 32);
        assert(stats.total_bit_errors == 0);
        pass("D1 - BER counter voi payload khong loi");
    }
    {
        stats = BERStats{};
        vector<uint8_t> expected(4, 0x00);
        vector<uint8_t> received = expected;
        received[0] ^= 0x01;
        updateBER(stats, expected, received);
        assert(stats.total_bits_compared == 32);
        assert(stats.total_bit_errors == 1);
        pass("D2 - Dem chinh xac mot bit loi");
    }
    {
        stats = BERStats{};
        vector<uint8_t> expected(4, 0x00);
        vector<uint8_t> received = expected;
        received[0] ^= 0x05; // Hai bit 1
        received[1] ^= 0xF0; // Bon bit 1
        received[3] ^= 0xFF; // Tam bit 1
        updateBER(stats, expected, received);
        assert(stats.total_bits_compared == 32);
        assert(stats.total_bit_errors == 14);
        pass("D3 - Dem chinh xac nhieu bit loi");
    }
    {
        stats = BERStats{};
        vector<uint8_t> expected = {0x00, 0x01};
        vector<uint8_t> received = {0x00};
        updateBER(stats, expected, received);
        assert(stats.total_bits_compared == 0);
        assert(stats.total_bit_errors == 0);
        pass("D4 - Khong so sanh hai payload khac do dai");
    }
    // --------------------------------------------------------
    // GROUP E: RX STATE AND SEQUENCE
    // --------------------------------------------------------
    {
        resetAll(rx, stats, buffer);
        auto f = buildFrame(10);
        assert(processRxFrame(f, rx, stats));
        assert(rx.state == STATE_LOCKED);
        assert(rx.expected_seq == 11);
        assert(stats.frames_crc_ok == 1);
        assert(stats.total_bits_compared == 32);
        assert(stats.total_bit_errors == 0);
        pass("E1 - HUNT -> LOCKED voi frame CRC dung");
    }
    {
        resetAll(rx, stats, buffer);
        auto f1 = buildFrame(10);
        auto f2 = buildFrame(11);
        processRxFrame(f1, rx, stats);
        processRxFrame(f2, rx, stats);
        assert(rx.state == STATE_LOCKED);
        assert(rx.expected_seq == 12);
        assert(stats.frames_seq_mismatch == 0);
        assert(stats.total_bits_compared == 64);
        pass("E2 - Nhan SEQ lien tiep");
    }
    {
        resetAll(rx, stats, buffer);
        processRxFrame(buildFrame(10), rx, stats);
        processRxFrame(buildFrame(15), rx, stats);
        assert(stats.frames_seq_mismatch == 1);
        assert(rx.expected_seq == 16);
        assert(rx.state == STATE_LOCKED);
        // Frame mismatch khong duoc tinh BER theo chinh sach nay
        assert(stats.total_bits_compared == 32);
        pass("E3 - Phat hien SEQ mismatch va re-anchor");
    }
    {
        resetAll(rx, stats, buffer);
        processRxFrame(buildFrame(255), rx, stats);
        assert(rx.expected_seq == 0);
        processRxFrame(buildFrame(0), rx, stats);
        assert(rx.expected_seq == 1);
        assert(stats.frames_seq_mismatch == 0);
        assert(stats.total_bits_compared == 64);
        pass("E4 - SEQ wrap-around 255 -> 0");
    }
    {
        resetAll(rx, stats, buffer);
        processRxFrame(buildFrame(10), rx, stats);
        for (int i = 0; i < MAX_CONSECUTIVE_ERRORS; ++i) {
            uint8_t seq = static_cast<uint8_t>(20 + i * 2);
            processRxFrame(buildFrame(seq), rx, stats);
        }
        assert(stats.frames_seq_mismatch == MAX_CONSECUTIVE_ERRORS);
        assert(rx.state == STATE_HUNT);
        assert(rx.consecutive_errors == 0);
        pass("E5 - Tro ve HUNT sau 6 SEQ mismatch lien tiep");
    }
    {
        resetAll(rx, stats, buffer);
        processRxFrame(buildFrame(10), rx, stats);
        for (int i = 0; i < MAX_CONSECUTIVE_ERRORS; ++i) {
            auto bad = buildFrame(static_cast<uint8_t>(11 + i));
            bad[4] ^= 0x01;
            assert(!processRxFrame(bad, rx, stats));
        }
        assert(stats.frames_crc_bad == MAX_CONSECUTIVE_ERRORS);
        assert(rx.state == STATE_HUNT);
        assert(rx.consecutive_errors == 0);
        pass("E6 - Tro ve HUNT sau 6 CRC errors lien tiep");
    }
    // --------------------------------------------------------
    // GROUP F: PARSER
    // --------------------------------------------------------
    {
        resetAll(rx, stats, buffer);
        auto f = buildFrame(1);
        feedParser(buffer, f, rx, stats);
        assert(stats.frames_crc_ok == 1);
        assert(buffer.empty());
        pass("F1 - Parser nhan mot frame hoan chinh");
    }
    {
        // Thu tat ca vi tri chia frame: 1..9 byte dau
        for (size_t split = 1; split < FRAME_SIZE; ++split) {
            resetAll(rx, stats, buffer);
            auto f = buildFrame(10);
            vector<uint8_t> first(f.begin(), f.begin() + split);
            vector<uint8_t> second(f.begin() + split, f.end());
            feedParser(buffer, first, rx, stats);
            assert(stats.frames_received == 0);
            feedParser(buffer, second, rx, stats);
            assert(stats.frames_crc_ok == 1);
            assert(stats.frames_crc_bad == 0);
            assert(buffer.empty());
        }
        pass("F2 - Frame chia tai moi vi tri byte");
    }
    {
        resetAll(rx, stats, buffer);
        auto f = buildFrame(1);
        feedParser(buffer, vector<uint8_t>(f.begin(), f.begin() + 1), rx, stats);
        assert(buffer.size() == 1);
        assert(buffer[0] == 0xEB);
        feedParser(buffer, vector<uint8_t>(f.begin() + 1, f.end()), rx, stats);
        assert(stats.frames_crc_ok == 1);
        assert(buffer.empty());
        pass("F3 - SYNC EB 90 bi chia giua hai lan nhan");
    }
    {
        resetAll(rx, stats, buffer);
        vector<uint8_t> stream = {0x00, 0x11, 0x22, 0x33, 0x44};
        auto f = buildFrame(1);
        stream.insert(stream.end(), f.begin(), f.end());
        feedParser(buffer, stream, rx, stats);
        assert(stats.frames_crc_ok == 1);
        assert(buffer.empty());
        pass("F4 - Bo qua byte rac truoc SYNC");
    }
    {
        resetAll(rx, stats, buffer);
        vector<uint8_t> stream;
        for (uint8_t seq = 1; seq <= 25; ++seq) {
            auto f = buildFrame(seq);
            stream.insert(stream.end(), f.begin(), f.end());
        }
        feedParser(buffer, stream, rx, stats);
        assert(stats.frames_received == 25);
        assert(stats.frames_crc_ok == 25);
        assert(stats.frames_crc_bad == 0);
        assert(stats.frames_seq_mismatch == 0);
        assert(stats.total_bits_compared == 25 * 32);
        assert(stats.total_bit_errors == 0);
        assert(buffer.empty());
        pass("F5 - Parser xu ly 25 frame lien tiep");
    }
    {
        resetAll(rx, stats, buffer);
        auto bad = buildFrame(1);
        bad[4] ^= 0x01;
        feedParser(buffer, bad, rx, stats);
        assert(stats.frames_received == 1);
        assert(stats.frames_crc_bad == 1);
        assert(stats.frames_crc_ok == 0);
        assert(stats.total_bits_compared == 0);
        pass("F6 - Parser loai frame co CRC sai");
    }
    {
        // Candidate SYNC gia, sau do la mot frame that.
        // Parser phai tim lai SYNC that sau khi candidate loi.
        resetAll(rx, stats, buffer);
        vector<uint8_t> stream = {0xEB, 0x90, 0x77, 0x01};
        auto real_frame = buildFrame(7);
        stream.insert(stream.end(), real_frame.begin(), real_frame.end());
        feedParser(buffer, stream, rx, stats);
        assert(stats.frames_crc_ok == 1);
        assert(stats.frames_crc_bad >= 1);
        assert(buffer.empty());
        pass("F7 - Phuc hoi SYNC sau candidate frame loi");
    }
    {
        resetAll(rx, stats, buffer);
        // Chua du 10 byte: parser phai giu lai du lieu
        auto f = buildFrame(1);
        vector<uint8_t> partial(f.begin(), f.begin() + 6);
        feedParser(buffer, partial, rx, stats);
        assert(stats.frames_received == 0);
        assert(buffer.size() == 6);
        pass("F8 - Giu frame chua du byte, khong xu ly som");
    }
    // --------------------------------------------------------
    // GROUP G: BER THROUGH RX
    // --------------------------------------------------------
    {
        resetAll(rx, stats, buffer);
        auto f = buildFrame(5);
        // Tao payload sai nhung CRC van hop le
        f[4] ^= 0x05;
        recomputeFrameCRC(f);
        assert(validateFrame(f));
        feedParser(buffer, f, rx, stats);
        assert(stats.frames_crc_ok == 1);
        assert(stats.total_bits_compared == 32);
        assert(stats.total_bit_errors == 2);
        pass("G1 - BER end-to-end phat hien dung 2 bit sai");
    }
    {
        resetAll(rx, stats, buffer);
        auto f = buildFrame(5);
        // Loi payload nhung KHONG cap nhat CRC
        f[4] ^= 0x01;
        feedParser(buffer, f, rx, stats);
        assert(stats.frames_crc_bad == 1);
        assert(stats.frames_crc_ok == 0);
        assert(stats.total_bits_compared == 0);
        assert(stats.total_bit_errors == 0);
        pass("G2 - CRC bad khong duoc tinh vao BER payload");
    }
    {
        resetAll(rx, stats, buffer);
        for (uint8_t seq = 0; seq < 10; ++seq) {
            auto f = buildFrame(seq);
            // Moi frame loi 1 bit, tinh lai CRC de RX chap nhan
            f[4] ^= 0x01;
            recomputeFrameCRC(f);
            feedParser(buffer, f, rx, stats);
        }
        assert(stats.frames_crc_ok == 10);
        assert(stats.total_bits_compared == 320);
        assert(stats.total_bit_errors == 10);
        double ber = static_cast<double>(stats.total_bit_errors) /
                     stats.total_bits_compared;
        assert(ber > 0.0);
        assert(ber < 1.0);
        cout << "[INFO] BER test = "
             << fixed << setprecision(6)
             << ber << '\n';
        pass("G3 - BER tong hop qua 10 frame co loi");
    }
    // --------------------------------------------------------
    // SUMMARY
    // --------------------------------------------------------
    cout << "\n========================================\n";
    cout << " ALL " << test_count << " TESTS PASSED\n";
    cout << "========================================\n";
}
int main() {
    runUnitTests();
    return 0;
}
