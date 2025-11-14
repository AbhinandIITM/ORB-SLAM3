/*

build command: 
g++ echo_imu.cpp -o echo_imu     -std=c++20     -IuWebSockets/src     -IuWebSockets/uSockets/src     uWebSockets/uSockets/*.o     -lssl -lcrypto -luv -lz -lpthread

run command: 
 ./echo_imu 
*/
#include "App.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <string_view>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

struct PerSocketData {};

int main() {
    std::ofstream log_file("imu_euroc.txt");
    if (!log_file.is_open()) {
        std::cerr << "Failed to open imu_euroc.txt\n";
        return 1;
    }

    uWS::App()
        .ws<PerSocketData>("/*", {
            .open = [](auto *ws) {
                std::cout << "Client connected\n";
            },

            .message = [&](auto *ws, std::string_view msg, uWS::OpCode op) {

                try {
                    json j = json::parse(msg);

                    auto gyro  = j["imu"]["ICM42632M Gyroscope"]["values"];
                    auto accel = j["imu"]["Samsung Linear Acceleration Sensor"]["values"];
                    double t_sec = j["timestamp"];

                    long long t_ns = static_cast<long long>(t_sec * 1e9);

                    double ax = accel[0];
                    double ay = accel[1];
                    double az = accel[2];

                    double gx = gyro[0];
                    double gy = gyro[1];
                    double gz = gyro[2];

                    // Save in EuRoC format
                    log_file << t_ns << " "
                            << ax << " " << ay << " " << az << " "
                            << gx << " " << gy << " " << gz << "\n";

                    log_file.flush();

                    // Print full IMU vector
                    std::cout 
                        << "t=" << t_ns
                        << "  a=[" << ax << ", " << ay << ", " << az << "]"
                        << "  w=[" << gx << ", " << gy << ", " << gz << "]"
                        << std::endl;

                } catch (std::exception &e) {
                    std::cerr << "JSON parse error: " << e.what() << "\n";
                }
            }

        })
        .listen(8001, [](auto *token) {
            if (token)
                std::cout << "Listening on port 8001...\n";
        })
        .run();

    return 0;
}
