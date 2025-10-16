#pragma once
#include <modbus/modbus.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <utility>

class Moons {
public:
    Moons(int left_inv = 1, int right_inv = -1);
    ~Moons();

    // Motor control
    void resetAlarm();
    void startJog();
    void stopJog();
    void setSpeed(int motor_add, long speed1, long speed2);
    void setEncoder(std::pair<int32_t,int32_t> enc);

    // Read values
    std::pair<int32_t,int32_t> getEncoder(int32_t prev1, int32_t prev2);
    std::pair<int32_t,int32_t> getSpeed();
    std::pair<double,double> getCurrent();  // Returns current in Amps
    std::vector<int> readAlarm(int slave);

    // Helper
    bool brakeStatus(int slave);

    // Members for inversion
    int l_inv_, r_inv_;

private:
    modbus_t* mod;
    int motor_status;

    void end();
    void writeRegister(int address, int value, int slave);
    int32_t readRegister(int address, int slave, bool signed_mode);
    void longWrite(int address, int32_t value, int slave);
    int32_t longRead(int address, int slave, bool unsigned_mode, int32_t prev);

    int32_t two_cmp(uint32_t val, int bits);
    std::vector<int> getBits(uint32_t val, int bits);
};
