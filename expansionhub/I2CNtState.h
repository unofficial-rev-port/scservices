#pragma once

#include "wpi/nt/NetworkTableInstance.hpp"
#include "wpi/nt/RawTopic.hpp"
#include "wpi/nt/IntegerTopic.hpp"
#include "wpi/nt/DoubleTopic.hpp"
#include "wpi/nt/BooleanTopic.hpp"

#include "CachedCommand.h"

#define NUM_I2C_CHANNELS 4

namespace eh {

struct I2CChannelState {
    // Configuration
    CachedCommand<wpi::nt::IntegerSubscriber> speedCodeSubscriber;

    // Status
    wpi::nt::IntegerPublisher statusPublisher;
    wpi::nt::RawPublisher dataPublisher;

    // Block read configuration
    CachedCommand<wpi::nt::IntegerSubscriber> blockReadAddressSubscriber;
    CachedCommand<wpi::nt::IntegerSubscriber> blockReadRegisterSubscriber;
    CachedCommand<wpi::nt::IntegerSubscriber> blockReadBytesSubscriber;
    CachedCommand<wpi::nt::IntegerSubscriber> blockReadIntervalSubscriber;

    uint8_t lastStatus{0};
    std::vector<uint8_t> lastData;

    void Initialize(const wpi::nt::NetworkTableInstance& instance, int busId, int moduleAddress, int channel);
};

struct I2CNtState {
    std::array<I2CChannelState, NUM_I2C_CHANNELS> channels;

    void Initialize(const wpi::nt::NetworkTableInstance& instance, int busId, int moduleAddress);
};

} // namespace eh
