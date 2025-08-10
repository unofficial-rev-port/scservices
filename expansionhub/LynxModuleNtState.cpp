#include "LynxModuleNtState.h"

#include <string>

namespace eh {

void LynxModuleNtState::Initialize(const nt::NetworkTableInstance& instance, int busId, const LynxModuleInfo& info) {
    moduleInfo = info;
    
    std::string baseTopicName = "/ExpansionHub/" + std::to_string(busId) + "/Module_" + std::to_string(moduleInfo.address);
    
    auto statusTable = instance.GetTable(baseTopicName + "/Status");
    isConnectedPublisher = statusTable->GetBooleanTopic("Connected").Publish();
    moduleTypePublisher = statusTable->GetIntegerTopic("ModuleType").Publish();
    parentAddressPublisher = statusTable->GetIntegerTopic("ParentAddress").Publish();
    batteryVoltagePublisher = statusTable->GetDoubleTopic("BatteryVoltage").Publish();
    
    auto errorTable = instance.GetTable(baseTopicName + "/Errors");
    numNacksPublisher = errorTable->GetIntegerTopic("NumNacks").Publish();
    numCrcFailuresPublisher = errorTable->GetIntegerTopic("NumCrcFailures").Publish();
    numMissedSendLoopsPublisher = errorTable->GetIntegerTopic("NumMissedSendLoops").Publish();
    transactionTimePublisher = errorTable->GetIntegerTopic("TransactionTime").Publish();
    
    // Initialize motors if supported
    if (moduleInfo.GetNumMotors() > 0) {
        motors = std::make_unique<std::array<MotorNtState, 4>>();
        for (int i = 0; i < 4; i++) {
            (*motors)[i].Initialize(instance, busId, moduleInfo.address, i);
        }
    }
    
    // Initialize servos (all modules have servos)
    for (int i = 0; i < 6; i++) {
        servos[i].Initialize(instance, busId, moduleInfo.address, i);
    }
    
    // Initialize I2C if supported
    if (moduleInfo.HasI2C()) {
        i2c = std::make_unique<I2CNtState>();
        i2c->Initialize(instance, busId, moduleInfo.address);
    }
    
    // Publish initial module info
    moduleTypePublisher.Set(static_cast<int>(moduleInfo.moduleType));
    parentAddressPublisher.Set(moduleInfo.parentAddress);
}

void LynxModuleNtState::SetModuleType(LynxModuleType type) {
    moduleInfo.moduleType = type;
    moduleTypePublisher.Set(static_cast<int>(type));
}

} // namespace eh