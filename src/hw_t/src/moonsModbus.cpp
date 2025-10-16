#include "hw_t/moonsModbus.hpp"
#include <iostream>
#include <vector>
#include <stdint.h>




// Helpers
int32_t Moons::two_cmp(uint32_t val, int bits) {
    if (val >= (1u << (bits - 1))) return (int32_t)(val - (1u << bits));
    return (int32_t)val;
}

std::vector<int> Moons::getBits(uint32_t val, int bits) {
    std::vector<int> bits_set;
    for (int i = 0; i < bits; i++) {
        if (val & (1u << i)) bits_set.push_back(i);
    }
    return bits_set;
}

bool Moons::brakeStatus(int slave) {
    int val = readRegister(4, slave, false);
    return (val & 4) == 4;
}

// Constructor / Destructor
Moons::Moons(int left_inv, int right_inv) : l_inv_(left_inv), r_inv_(right_inv) {
    mod = modbus_new_rtu("/dev/ttyUSB0", 115200, 'N', 8, 1);
    if (!mod || modbus_connect(mod) == -1) {
        std::cerr << "Modbus connection failed\n";
        exit(-1);
    }
    motor_status = 0;
}

Moons::~Moons() {
    end();
}

// Low-level Modbus
void Moons::writeRegister(int address, int value, int slave) {
    modbus_set_slave(mod, slave);
    if (modbus_write_register(mod, address, value) == -1) motor_status = 0;
    else motor_status = 1;
}

int32_t Moons::readRegister(int address, int slave, bool signed_mode) {
    modbus_set_slave(mod, slave);
    uint16_t reg[1];
    if (modbus_read_registers(mod, address, 1, reg) == -1) {
        motor_status = 0;
        return 0;
    }
    motor_status = 1;
    return signed_mode ? two_cmp(reg[0], 16) : reg[0];
}

void Moons::longWrite(int address, int32_t value, int slave) {
    modbus_set_slave(mod, slave);
    uint16_t regs[2] = {(uint16_t)(value >> 16), (uint16_t)(value & 0xFFFF)};
    if (modbus_write_registers(mod, address, 2, regs) == -1) motor_status = 0;
    else motor_status = 1;
}

int32_t Moons::longRead(int address, int slave, bool unsigned_mode, int32_t prev) {
    modbus_set_slave(mod, slave);
    uint16_t regs[2];
    if (modbus_read_registers(mod, address, 2, regs) == -1) {
        motor_status = 0;
        return prev;
    }
    motor_status = 1;
    uint32_t val = ((uint32_t)regs[0] << 16) | regs[1];
    return unsigned_mode ? (int32_t)val : two_cmp(val, 32);
}

// Motor functions

void Moons::resetAlarm() {
    writeRegister(124, 186, 1);
    writeRegister(124, 186, 2);
    std::cout << "Alarms reset" << std::endl;
}

void Moons::startJog() {
    writeRegister(124, 159, 1);
    writeRegister(124, 159, 2);
    if (brakeStatus(1) || brakeStatus(2)) {
        motor_status = 3;
        std::cout << "Brake engaged, reset alarm" << std::endl;
    } else {
        writeRegister(124, 150, 1);
        writeRegister(124, 150, 2);
        std::cout << "Jog started" << std::endl;
    }
}

void Moons::stopJog() {
    writeRegister(124, 226, 1);
    writeRegister(124, 226, 2);
    writeRegister(124, 158, 1);
    writeRegister(124, 158, 2);
    std::cout << "Jog stopped" << std::endl;
}

// Now apply inversion ONLY here in setSpeed:
void Moons::setSpeed(int motor_add, long speed1, long speed2) {
    long s1 = speed1 * l_inv_;
    long s2 = speed2 * r_inv_;
    if (motor_add & 1) longWrite(342, s1, 1);
    if (motor_add & 2) longWrite(342, s2, 2);
}

// Apply inversion in setEncoder
void Moons::setEncoder(std::pair<int32_t,int32_t> enc) {
    long e1 = enc.first * l_inv_;
    long e2 = enc.second * r_inv_;
    longWrite(125, e1, 1);
    writeRegister(124, 0x98, 1);
    longWrite(125, e2, 2);
    writeRegister(124, 0x98, 2);
    std::cout << "Encoders set: " << e1 << ", " << e2 << std::endl;
}

// Apply inversion also when reading out (getEncoder)
std::pair<int32_t,int32_t> Moons::getEncoder(int32_t prev1, int32_t prev2) {
    int32_t enc1 = longRead(10, 1, false, prev1) * l_inv_;
    int32_t enc2 = longRead(10, 2, false, prev2) * r_inv_;
    return {enc1, enc2};
}

// Apply inversion when reading speed
std::pair<int32_t,int32_t> Moons::getSpeed() {
    int32_t vel1 = readRegister(16, 1, true) * l_inv_;
    int32_t vel2 = readRegister(16, 2, true) * r_inv_;
    return {vel1, vel2};
}

std::pair<double,double> Moons::getCurrent() {
    int32_t cur1 = readRegister(20, 1, true);
    int32_t cur2 = readRegister(20, 2, true);
    return {cur1 * 0.01, cur2 * 0.01};
}

std::vector<int> Moons::readAlarm(int slave) {
    int32_t val = longRead(0, slave, false, 0);
    std::vector<int> alarms = getBits(val, 32);

    std::vector<std::string> alarm_messages = {
        "position error overrun", "reverse prohibition limit", "positive prohibition limit", "over temperature",
        "internal error", "supply voltage out of range", "reserved", "drive overcurrent", "reserved",
        "motor encoder not connected", "communication exception", "reserved", "vent failure", "motor overload protection",
        "reserved", "unusual start alarm", "input phase loss", "STO", "reserved", "motor speed exceeds limit",
        "drive undervoltage", "emergency stop", "second encoder not connected", "full closed loop deviation overrun",
        "absolute encoder battery undervoltage", "accurate position lost", "absolute position overflow",
        "communication interrupt", "absolute encoder multi-turn error", "abnormal motor action protection",
        "EtherCAT communication error", "Back to origin parameter configuration error"
    };

    for (int i : alarms) {
        if (i < (int)alarm_messages.size())
            std::cout << "Alarm (slave " << slave << "): " << alarm_messages[i] << std::endl;
    }
    return alarms;
}

void Moons::end() {
    stopJog();
    std::cout << "Closing connection" << std::endl;
    modbus_close(mod);
    modbus_free(mod);
}
