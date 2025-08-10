#pragma once

#include <array>
#include <memory>
#include "MotorNtState.h"
#include "ServoNtState.h"
#include "I2CNtState.h"
#include "LynxModuleTypes.h"

namespace eh {

struct LynxModuleNtState {
    LynxModuleInfo moduleInfo;
    
    // Motor support (only for expansion/control hubs)
    std::unique_ptr<std::array<MotorNtState, 4>> motors;
    
    // Servo support (all modules)
    std::array<ServoNtState, 6> servos;
    
    // I2C support (only for expansion/control hubs)
    std::unique_ptr<I2CNtState> i2c;
    
    // Module status
    double lastBattery{0};
    nt::DoublePublisher batteryVoltagePublisher;
    nt::BooleanPublisher isConnectedPublisher;
    nt::IntegerPublisher moduleTypePublisher;
    nt::IntegerPublisher parentAddressPublisher;
    
    // Error tracking
    nt::IntegerPublisher numNacksPublisher;
    nt::IntegerPublisher numCrcFailuresPublisher;
    nt::IntegerPublisher numMissedSendLoopsPublisher;
    nt::IntegerPublisher transactionTimePublisher;
    
    uint64_t numNacks{0};
    uint64_t numCrcFailures{0};
    uint64_t numMissedSendLoops{0};
    
    void Initialize(const nt::NetworkTableInstance& instance, int busId, const LynxModuleInfo& info);
    void SetModuleType(LynxModuleType type);
    
    bool HasMotors() const { return motors != nullptr; }
    bool HasI2C() const { return i2c != nullptr; }
    
    MotorNtState& GetMotor(int index) {
        if (!HasMotors() || index < 0 || index >= 4) {
            throw std::out_of_range("Motor index out of range or motors not supported");
        }
        return (*motors)[index];
    }
    
    ServoNtState& GetServo(int index) {
        if (index < 0 || index >= 6) {
            throw std::out_of_range("Servo index out of range");
        }
        return servos[index];
    }
    
    I2CChannelState& GetI2CChannel(int index) {
        if (!HasI2C() || index < 0 || index >= NUM_I2C_CHANNELS) {
            throw std::out_of_range("I2C channel index out of range or I2C not supported");
        }
        return i2c->channels[index];
    }
};

} // namespace eh