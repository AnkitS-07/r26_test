#include <iostream>
#include <vector>
#include <cstdint>
#include <cmath>
using namespace std;

class UbloxReader {
public:
    // Decode NAV-PVT payload (latitude, longitude in 1e-7 degrees)
    pair<double, double> decodeNavPvt(const vector<uint8_t>& payload) {
        if (payload.size() < 36) {
            cerr << "Payload too short!" << endl;
            return {0.0, 0.0};
        }

        // Latitude = int32 at offset 28 (1e-7 deg)
        int32_t lat_raw = (int32_t)(
            payload[28] | (payload[29] << 8) | (payload[30] << 16) | (payload[31] << 24)
        );

        // Longitude = int32 at offset 24 (1e-7 deg)
        int32_t lon_raw = (int32_t)(
            payload[24] | (payload[25] << 8) | (payload[26] << 16) | (payload[27] << 24)
        );

        double lat = lat_raw * 1e-7;
        double lon = lon_raw * 1e-7;

        return {lat, lon};
    }
};
