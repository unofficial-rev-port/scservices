#include "I2CNtState.h"

#include <string>

namespace eh {

void I2CChannelState::Initialize(const nt::NetworkTableInstance& instance, int busId, int moduleAddress, int channel) {
    std::string baseTopicName = "/ExpansionHub/" + std::to_string(busId) + "/Module_" + std::to_string(moduleAddress) + "/I2C_" + std::to_string(channel);

    auto configTable = instance.GetTable(baseTopicName + "/Config");
    speedCodeSubscriber = configTable->GetIntegerTopic("SpeedCode").Subscribe(0);
    blockReadAddressSubscriber = configTable->GetIntegerTopic("BlockReadAddress").Subscribe(0);
    blockReadRegisterSubscriber = configTable->GetIntegerTopic("BlockReadRegister").Subscribe(0);
    blockReadBytesSubscriber = configTable->GetIntegerTopic("BlockReadBytes").Subscribe(0);
    blockReadIntervalSubscriber = configTable->GetIntegerTopic("BlockReadInterval").Subscribe(0);

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