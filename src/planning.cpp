#include <iostream>
#include <fstream>
#include <vector>
#include "ublox_reader.h"

struct GPSData {
    double latitude;
    double longitude;
};

std::vector<GPSData> readUbloxFile(const std::string &filename) {
    std::ifstream file(filename, std::ios::binary);
    std::vector<GPSData> data;

    if (!file.is_open()) {
        std::cerr << "Error opening UBX file: " << filename << std::endl;
        return data;
    }

    unsigned char buffer[100];
    while (file.read(reinterpret_cast<char*>(buffer), sizeof(buffer))) {
        // UBX NAV-POSLLH payload starts after header 0xB5 0x62 0x01 0x02
        int32_t lon_raw = *reinterpret_cast<int32_t*>(&buffer[10]); // in 1e-7 deg
        int32_t lat_raw = *reinterpret_cast<int32_t*>(&buffer[14]);

        GPSData point;
        point.longitude = lon_raw / 1e7;
        point.latitude  = lat_raw / 1e7;
        data.push_back(point);
    }
    return data;
}

