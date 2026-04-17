#pragma once

#include "networktables/NetworkTableInstance.h"
#include "networktables/RawTopic.h"
#include "networktables/IntegerTopic.h"
#include "networktables/DoubleTopic.h"
#include "networktables/BooleanTopic.h"

#include "CachedCommand.h"

#define NUM_I2C_CHANNELS 4

namespace eh {

struct I2CChannelState {
    // Configuration
    CachedCommand<nt::IntegerSubscriber> speedCodeSubscriber;

    // Status
    nt::IntegerPublisher statusPublisher;
    nt::RawPublisher dataPublisher;

    // Block read configuration
    CachedCommand<nt::IntegerSubscriber> blockReadAddressSubscriber;
    CachedCommand<nt::IntegerSubscriber> blockReadRegisterSubscriber;
    CachedCommand<nt::IntegerSubscriber> blockReadBytesSubscriber;
    CachedCommand<nt::IntegerSubscriber> blockReadIntervalSubscriber;
    
    uint8_t lastStatus{0};
    std::vector<uint8_t> lastData;
    
    void Initialize(const nt::NetworkTableInstance& instance, int busId, int moduleAddress, int channel);
};

struct I2CNtState {
    std::array<I2CChannelState, NUM_I2C_CHANNELS> channels;
    
    void Initialize(const nt::NetworkTableInstance& instance, int busId, int moduleAddress);
};

} // namespace eh