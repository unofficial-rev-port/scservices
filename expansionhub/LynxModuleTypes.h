#pragma once

#include <cstdint>

namespace eh {

enum class LynxModuleType {
    UNKNOWN = 0,
    EXPANSION_HUB = 1,
    CONTROL_HUB = 2,
    SERVO_HUB = 3
};

struct LynxModuleInfo {
    uint8_t address{0};
    uint8_t parentAddress{0xFF};
    LynxModuleType moduleType{LynxModuleType::UNKNOWN};
    bool isParent{false};
    bool isRS485Device{false};
    std::string serialPath;
    
    bool IsValidAddress() const {
        return address != 0 && address != 0xFF;
    }
    
    bool IsChainedDevice() const {
        return isRS485Device && parentAddress != 0xFF;
    }
    
    int GetNumMotors() const {
        switch (moduleType) {
            case LynxModuleType::EXPANSION_HUB:
            case LynxModuleType::CONTROL_HUB:
                return 4;
            case LynxModuleType::SERVO_HUB:
                return 0;
            default:
                return 0;
        }
    }
    
    int GetNumServos() const {
        switch (moduleType) {
            case LynxModuleType::EXPANSION_HUB:
            case LynxModuleType::CONTROL_HUB:
                return 6;
            case LynxModuleType::SERVO_HUB:
                return 6;
            default:
                return 0;
        }
    }
    
    bool HasI2C() const {
        switch (moduleType) {
            case LynxModuleType::EXPANSION_HUB:
            case LynxModuleType::CONTROL_HUB:
                return true;
            case LynxModuleType::SERVO_HUB:
                return false;
            default:
                return false;
        }
    }
};

} // namespace eh