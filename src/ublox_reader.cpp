#include <iostream>
#include <vector>
#include <cstdint>
#include <utility>  
using namespace std;

class UbloxReader {
public:
    // Decode NAV-PVT payload (latitude, longitude in degrees)
    // UBX spec: lon = I4 (1e-7 deg) at offset 24, lat = I4 (1e-7 deg) at offset 28
    pair<double, double> decodeNavPvt(const vector<uint8_t>& payload) {
        if (payload.size() < 32) {  // need at least up to byte 31
            cerr << "Payload too short for NAV-PVT!" << endl;
            return {0.0, 0.0};
        }

        // Longitude (little-endian int32 at offset 24)
        int32_t lon_raw =
            (static_cast<int32_t>(payload[24])) |
            (static_cast<int32_t>(payload[25]) << 8) |
            (static_cast<int32_t>(payload[26]) << 16) |
            (static_cast<int32_t>(payload[27]) << 24);

        // Latitude (little-endian int32 at offset 28)
        int32_t lat_raw =
            (static_cast<int32_t>(payload[28])) |
            (static_cast<int32_t>(payload[29]) << 8) |
            (static_cast<int32_t>(payload[30]) << 16) |
            (static_cast<int32_t>(payload[31]) << 24);

        // Scale from 1e-7 degrees → degrees
        double lon = lon_raw * 1e-7;
        double lat = lat_raw * 1e-7;

        return {lat, lon};  // return {latitude, longitude}
    }
};
