// src/ublox_reader.cpp
// Simple UBX parser for UBX-NAV-PVT (class 0x01 id 0x07).
// Namespaced implementation: ublox::parse_ubx_file(...)
#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>
#include <iomanip>
#include <string>

using namespace std;

namespace ublox {

using u8  = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using i32 = int32_t;

static inline u16 read_u16_le(const vector<u8>& b, size_t i) {
    return u16(b[i]) | (u16(b[i+1]) << 8);
}
static inline i32 read_i32_le(const vector<u8>& b, size_t i) {
    u32 v = u32(b[i]) | (u32(b[i+1]) << 8) | (u32(b[i+2]) << 16) | (u32(b[i+3]) << 24);
    return static_cast<i32>(v);
}

bool compute_and_check_checksum(const vector<u8>& data, size_t start_class_index, size_t /*payload_len*/, size_t ck_index) {
    u8 ck_a = 0, ck_b = 0;
    // checksum covers CLASS, ID, LEN(2), PAYLOAD
    for (size_t idx = start_class_index; idx < ck_index; ++idx) {
        ck_a = ck_a + data[idx];
        ck_b = ck_b + ck_a;
    }
    return (ck_a == data[ck_index]) && (ck_b == data[ck_index + 1]);
}

void parse_ubx_file(const string &path) {
    ifstream ifs(path, ios::binary);
    if (!ifs) {
        cerr << "Cannot open file: " << path << "\n";
        return;
    }
    vector<u8> buf((istreambuf_iterator<char>(ifs)), istreambuf_iterator<char>());
    size_t i = 0;
    size_t n = buf.size();

    while (i + 8 < n) { // minimal UBX frame size check
        // find sync bytes 0xB5 0x62
        if (buf[i] != 0xB5 || buf[i+1] != 0x62) { ++i; continue; }
        if (i + 6 >= n) break; // need at least header bytes

        u8 cls = buf[i+2];
        u8 id  = buf[i+3];
        u16 length = read_u16_le(buf, i+4);
        size_t payload_start = i + 6;
        size_t payload_end = payload_start + length; // one past the payload
        size_t ck_pos = payload_end;
        // bounds check
        if (payload_end + 2 > n) { break; }

        // Validate checksum
        if (!compute_and_check_checksum(buf, i+2, length, ck_pos)) {
            // checksum failed; skip this sync and keep searching
            ++i;
            continue;
        }

        // If NAV-PVT (class 0x01 id 0x07)
        if (cls == 0x01 && id == 0x07 && length >= 32) {
            // lon @ payload offset 24 (i4, 1e-7 deg)
            // lat @ payload offset 28 (i4, 1e-7 deg)
            size_t lon_off = payload_start + 24;
            size_t lat_off = payload_start + 28;
            if (lon_off + 4 <= n && lat_off + 4 <= n) {
                i32 raw_lon = read_i32_le(buf, lon_off);
                i32 raw_lat = read_i32_le(buf, lat_off);
                double lon_deg = raw_lon * 1e-7;
                double lat_deg = raw_lat * 1e-7;

                // Also print time-of-week and fixType optionally
                u32 iTOW = u32(read_i32_le(buf, payload_start + 0));
                u8 fixType = buf[payload_start + 20];

                cout << fixed << setprecision(7);
                cout << "NAV-PVT: lat = " << lat_deg << " deg, lon = " << lon_deg << " deg";
                cout << "  (iTOW=" << iTOW << " ms, fixType=" << int(fixType) << ")\n";
            }
        }

        // Move index past this frame
        i = payload_end + 2; // ck_a and ck_b consumed
    }
}

} 

int main(int argc, char** argv) {
    if (argc != 2) {
        cerr << "Usage: " << argv[0] << " path_to_file.ubx\n";
        return 1;
    }
    ublox::parse_ubx_file(argv[1]);
    return 0;
}
