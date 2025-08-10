#include "I2CNtState.h"

#include <string>

namespace eh {

void I2CChannelState::Initialize(const nt::NetworkTableInstance& instance, int busId, int moduleAddress, int channel) {
    std::string baseTopicName = "/ExpansionHub/" + std::to_string(busId) + "/Module_" + std::to_string(moduleAddress) + "/I2C_" + std::to_string(channel);
    
    auto configTable = instance.GetTable(baseTopicName + "/Config");
    speedCodeSubscriber.Initialize(configTable->GetIntegerTopic("SpeedCode"), 0);
    blockReadAddressSubscriber.Initialize(configTable->GetIntegerTopic("BlockReadAddress"), 0);
    blockReadRegisterSubscriber.Initialize(configTable->GetIntegerTopic("BlockReadRegister"), 0);
    blockReadBytesSubscriber.Initialize(configTable->GetIntegerTopic("BlockReadBytes"), 0);
    blockReadIntervalSubscriber.Initialize(configTable->GetIntegerTopic("BlockReadInterval"), 0);
    
    auto statusTable = instance.GetTable(baseTopicName + "/Status");
    statusPublisher = statusTable->GetIntegerTopic("Status").Publish();
    dataPublisher = statusTable->GetRawTopic("Data").Publish("raw");
}

void I2CNtState::Initialize(const nt::NetworkTableInstance& instance, int busId, int moduleAddress) {
    for (int i = 0; i < NUM_I2C_CHANNELS; i++) {
        channels[i].Initialize(instance, busId, moduleAddress, i);
    }
}

} // namespace eh