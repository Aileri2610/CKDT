#include <iostream>
#include <vector>
#include <unistd.h>
#include <cstdint>

using namespace std;

int PRBS7(uint16_t& state) {
    int feedback = ((state >> 6) & 1) ^ ((state >> 5) & 1);
    state = ((state << 1) & 0x7F) | feedback;
    return feedback;
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
vector<uint8_t> makePayload(uint16_t &state) {
    vector<uint8_t> payload(4);

    for (int i = 0; i < 4; i++) {
        uint8_t byte = 0;
        for (int j = 0; j < 8; j++) {
            byte = (byte << 1) | PRBS7(state);
        }
        payload[i] = byte;
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
    vector<uint8_t> frame(10);
    //SYNC
    frame[0] = 0xEB;
    frame[1] = 0x90;
    //TYPE and SEQ 
    frame[2] = type;
    frame[3] = seq;

    //PAYLOAD 
    for(int i = 0; i<4; i++){
        frame[4+i] = payload[i];
    }

    uint16_t crc = calculateFrameCRC(&frame[2], 6);

    frame[8] = (crc >> 8) & 0xFF;
    frame[9] = crc & 0xFF;
    return frame;
}

int main() {
    verifyPRBS7Cycle();

    uint16_t prbs_state = 0x7F;
    auto payload = makePayload(prbs_state);
    vector<uint8_t> frame = buildFrame(0x01, 10, payload);
    cout << "8 bit dau tien cua frame(MSB first): ";
    for(int i = 0; i<8; i++){
        cout << frameBit(frame, i);
    }
    cout << endl;

    return 0;
}
//Ham tin hieu nhan RX
int findSYNC(const vector<uint8_t> &buffer){
    if(buffer.size() < 2){
        return -1;
    }

    for(size_t i = 0; i <= buffer.size() -2; i++){
        if(buffer[i] == 0xEB && buffer[i+1] == 0x90){
            return static_cast<int>(i);
        }
    }
    return -1;
}
