#pragma once

#include "wpi/net/uv/Loop.hpp"
#include "wpi/net/uv/Poll.hpp"
#include "functional"
#include "LynxModuleNtState.h"
#include "MessageNumbers.h"
#include "ReceiveStateMachine.h"
#include <deque>
#include <map>
#include <memory>
#include <set>

namespace eh {

class LynxUsbDevice {
   public:
    enum class DeviceState {
        Initializing,
        Discovering,
        ConfiguringInterface,
        ConfiguringFtdi,
        Ready,
        Error
    };

    enum class SendState {
        ReadyToSend,
        WaitingForPackets,
        WaitingForFinish,
    };

    LynxUsbDevice() = default;
    ~LynxUsbDevice() noexcept;

    bool Initialize(wpi::net::uv::Loop& loop, int fd, std::string path, int busId, bool isUart = false);

    void SetNtInstance(const wpi::nt::NetworkTableInstance* instance) { ntInstance = instance; }

    void RunDiscoverySteps();
    bool HasFinishedInitialization() const { return deviceState == DeviceState::Ready; }

    void StartTransaction(bool canDoEnable);

    void Recover();
    bool AllowSend();

    void Flush();

    std::string_view SerialPath() const { return serialPath; }

    // Device management
    void OnModuleDiscovered(uint8_t address, uint8_t parentAddress);
    LynxModuleNtState* GetModule(uint8_t address);
    std::vector<std::unique_ptr<LynxModuleNtState>>& GetModules() { return modules; }

    // Communication methods
    void SendBatteryRequest(uint8_t moduleAddress);
    void SendMotorCurrentRequest(uint8_t moduleAddress, uint8_t channel);
    void SendEncoderResetRequest(uint8_t moduleAddress, uint8_t channel);
    void SendServoConfiguration(uint8_t moduleAddress, uint8_t channel, uint16_t framePeriod);
    void SendServoPulseWidth(uint8_t moduleAddress, uint8_t channel, uint16_t pulseWidth);
    void SendServoEnable(uint8_t moduleAddress, uint8_t channel, bool enable);
    void SendMotorConstantPower(uint8_t moduleAddress, uint8_t channel, double power);
    void SendMotorMode(uint8_t moduleAddress, uint8_t channel, bool floatVal);
    void SendMotorEnable(uint8_t moduleAddress, uint8_t channel, bool enable);
    void GetModuleStatus(uint8_t moduleAddress);
    void SendKeepAlive(uint8_t moduleAddress);
    void SendBulkInput(uint8_t moduleAddress);

    // I2C methods
    void SendI2CWriteMultipleBytes(uint8_t moduleAddress, uint8_t channel, uint8_t deviceAddress, const std::vector<uint8_t>& data);
    void SendI2CReadMultipleBytes(uint8_t moduleAddress, uint8_t channel, uint8_t deviceAddress, uint8_t numBytes);
    void SendI2CConfigureChannel(uint8_t moduleAddress, uint8_t channel, uint8_t speedCode);
    void SendI2CBlockReadConfig(uint8_t moduleAddress, uint8_t channel, uint8_t deviceAddress, uint8_t startRegister, uint8_t numBytes, uint8_t interval);

   private:
    const wpi::nt::NetworkTableInstance* ntInstance{nullptr};
    int busId{0};
    
    std::vector<std::unique_ptr<LynxModuleNtState>> modules;
    std::map<uint8_t, size_t> addressToModuleIndex;

    void RunDiscoverInternal();
    void RunInterfacePacketIdInternal();
    void RunFtdiConfigureInternal();

    void SendPacket(uint8_t destAddr, uint8_t messageNumber,
                    uint16_t packetTypeId, std::span<const uint8_t> payload,
                    bool direct = false);

    void DoRead();

    void CheckSendStateAdvance();
    void HandlePayload(std::span<const uint8_t> data, uint8_t crc);

    LynxUsbDevice(LynxUsbDevice&) = delete;
    LynxUsbDevice(LynxUsbDevice&&) = delete;
    LynxUsbDevice& operator=(LynxUsbDevice&) = delete;
    LynxUsbDevice& operator=(LynxUsbDevice&&) = delete;

    uint8_t readBuf[256];
    int serialFd{-1};
    std::weak_ptr<wpi::net::uv::Poll> serialPoll;
    eh::ReceiveStateMachine stateMachine{
        [this](auto data, auto crc) { HandlePayload(data, crc); }};

    uint64_t discoverStartTime{0};
    std::optional<uint16_t> packetInterfaceId{};

    uint8_t outstandingMessages{0};
    std::vector<uint8_t> writeBuffer;
    std::deque<size_t> pendingWrites;
    size_t currentCount{0};

    // Diagnostic: per-packet send tracking. Each non-direct SendPacket pushes
    // here; HandlePayload removes the matching (dest, msgNum) entry on response.
    // If something stays here long enough to hit Recover, the hub silently
    // dropped it.
    struct PendingSend {
        uint64_t sentAt;
        uint8_t dest;
        uint8_t msgNum;
        uint16_t packetTypeId;
    };
    std::deque<PendingSend> pendingSends;
    uint64_t totalSent{0};
    uint64_t totalReceived{0};

    uint8_t txBuffer[1024];

    std::string serialPath;
    uint64_t lastLoop;
    bool canEnable{false};
    
    DeviceState deviceState{DeviceState::Initializing};
    bool configuredFtdiReset{false};
    bool isUartConnection{false};

    SendState sendState{SendState::ReadyToSend};
    bool haveBulk{false};
    bool haveBattery{false};
    bool haveModuleStatus{false};

    // Discovery tracking
    std::set<uint8_t> discoveredAddresses;
    bool discoveryComplete{false};
};

}  // namespace eh