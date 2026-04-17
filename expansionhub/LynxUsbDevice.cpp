#include "LynxUsbDevice.h"

#include "stdio.h"
#include "wpi/timestamp.h"
#include <algorithm>
#include <set>
#include <unistd.h>
#include <termios.h>

// EXPERIMENT 2026-04-16: lowered from 8 to 1 to disambiguate why dest=2
// silently drops every packet. Combined with DEBUG_ONLY_MODULE_2 in
// main_enhanced.cpp, this forces a strictly serial "send one, wait for reply"
// pattern with no first-tick burst. If module 2 still gives no response,
// the issue is routing (parent doesn't relay unicast post-discovery), not
// burst-induced confusion. Revert to 8 once root cause is fixed.
#define MAX_NUM_OUTSTANDING_MESSAGES 1

#define MODULE_STATUS_ID 0x7F03
#define KEEP_ALIVE_ID 0x7F04
#define QUERY_INTERFACE_ID 0x7F07
#define INTERFACE_STRING "DEKA"
#define DISCOVER_ID 0x7F0F
#define DISCOVER_ADDRESS 0xFF

#define MOTOR_0_ADC 8
#define BATTERY_ADC 13
#define POWER_CONVERSION 32767

#define MESSAGE_TIMEOUT 1000000

using namespace eh;

static constexpr void WriteUint16(std::span<uint8_t> buffer, uint16_t value) {
    buffer[0] = static_cast<uint8_t>(value);
    buffer[1] = static_cast<uint8_t>(value >> 8);
}

static constexpr int16_t ReadInt16(std::span<const uint8_t> buffer) {
    return static_cast<int16_t>(buffer[0]) |
           (static_cast<int16_t>(buffer[1]) << 8);
}

static constexpr int32_t ReadInt32(std::span<const uint8_t> buffer) {
    return static_cast<int32_t>(buffer[0]) |
           (static_cast<int32_t>(buffer[1]) << 8) |
           (static_cast<int32_t>(buffer[2]) << 16) |
           (static_cast<int32_t>(buffer[3]) << 24);
}

static constexpr uint16_t ReadUint16(std::span<const uint8_t> buffer) {
    return static_cast<uint16_t>(buffer[0] |
                                 (static_cast<uint16_t>(buffer[1]) << 8));
}

static constexpr uint8_t PacketReferenceNumber(std::span<const uint8_t> buffer) {
    return buffer[7];
}

static constexpr uint8_t PacketDestinationAddress(std::span<const uint8_t> buffer) {
    return buffer[4];
}

static constexpr uint8_t PacketSourceAddress(std::span<const uint8_t> buffer) {
    return buffer[5];
}

static constexpr uint16_t PacketId(std::span<const uint8_t> buffer) {
    return ((uint16_t)(buffer[9]) << 8 | (uint16_t)(buffer[8]));
}

static constexpr std::span<const uint8_t> PacketPayloadBuffer(std::span<const uint8_t> buffer) {
    return buffer.subspan(10, buffer.size() - 11);
}

static constexpr uint8_t PacketCrc(std::span<const uint8_t> buffer) {
    return buffer[buffer.size() - 1];
}

static constexpr bool PacketIsAck(uint16_t packetId) {
    return packetId == 0x7F01;
}

static constexpr bool PacketIsNack(uint16_t packetId) {
    return packetId == 0x7F02;
}

static constexpr bool PacketIsDiscover(uint16_t packetId) {
    return packetId == (DISCOVER_ID | 0x8000);
}

static constexpr bool PacketIsQueryInterface(uint16_t packetId) {
    return packetId == (QUERY_INTERFACE_ID | 0x8000);
}

static constexpr uint8_t CalcChecksum(std::span<const uint8_t> buffer) {
    uint8_t sum = 0;
    for (auto&& b : buffer) {
        sum += b;
    }
    return sum;
}

bool LynxUsbDevice::Initialize(wpi::uv::Loop& loop, int fd, std::string path, int busId, bool isUart) {
    serialFd = fd;
    serialPath = std::move(path);
    this->busId = busId;
    this->isUartConnection = isUart;
    tcflush(serialFd, TCIFLUSH);

    auto poll = wpi::uv::Poll::Create(loop, serialFd);
    if (!poll) {
        return false;
    }

    poll->pollEvent.connect([this](int flags) {
        if ((flags & UV_READABLE) != 0) {
            DoRead();
        }
    });

    poll->Start(UV_READABLE);
    serialPoll = poll;

    deviceState = DeviceState::Discovering;
    return true;
}

void LynxUsbDevice::OnModuleDiscovered(uint8_t address, uint8_t parentAddress) {
    discoveredAddresses.insert(address);
    
    // Create module info
    LynxModuleInfo moduleInfo;
    moduleInfo.address = address;
    moduleInfo.parentAddress = parentAddress;
    moduleInfo.isParent = (parentAddress == 0xFF);
    moduleInfo.isRS485Device = !moduleInfo.isParent;
    moduleInfo.serialPath = serialPath;
    
    // Default to EXPANSION_HUB for all discovered modules (parent or chained)
    // Servo hubs would need to be identified separately
    moduleInfo.moduleType = LynxModuleType::EXPANSION_HUB;
    
    auto module = std::make_unique<LynxModuleNtState>();
    if (ntInstance) {
        module->Initialize(*ntInstance, busId, moduleInfo);
        printf("Module NT initialized: bus=%d addr=%d type=%d ntConnected=%d\n",
               busId, address, static_cast<int>(moduleInfo.moduleType),
               ntInstance->IsConnected() ? 1 : 0);
    } else {
        printf("WARNING: ntInstance is null during module discovery!\n");
    }

    size_t moduleIndex = modules.size();
    addressToModuleIndex[address] = moduleIndex;
    modules.push_back(std::move(module));

    printf("Discovered module at address %d (parent: %d, isParent: %d)\n", address, parentAddress, moduleInfo.isParent ? 1 : 0);
}

LynxModuleNtState* LynxUsbDevice::GetModule(uint8_t address) {
    auto it = addressToModuleIndex.find(address);
    if (it != addressToModuleIndex.end()) {
        return modules[it->second].get();
    }
    return nullptr;
}

void LynxUsbDevice::RunDiscoverInternal() {
    auto now = wpi::Now();
    auto delta = now - discoverStartTime;

    // If we've found at least one module, use a shorter timeout (500ms) before proceeding
    if (discoveryComplete && delta > 500000) {
        printf("Discovery complete: found %zu module(s)\n", discoveredAddresses.size());
        deviceState = DeviceState::ConfiguringInterface;
        discoverStartTime = 0;
        return;
    }

    if (delta <= MESSAGE_TIMEOUT) {
        return;
    }

    discoverStartTime = now;
    SendPacket(DISCOVER_ADDRESS, MESSAGE_DISCOVER, DISCOVER_ID, {}, true);
}

void LynxUsbDevice::RunInterfacePacketIdInternal() {
    auto now = wpi::Now();
    auto delta = now - discoverStartTime;

    if (delta <= MESSAGE_TIMEOUT) {
        return;
    }

    discoverStartTime = now;

    std::string_view interfaceString = INTERFACE_STRING;
    
    // Query the first discovered module for interface ID
    if (!discoveredAddresses.empty()) {
        uint8_t address = *discoveredAddresses.begin();
        SendPacket(address, MESSAGE_QUERY_INTERFACE, QUERY_INTERFACE_ID,
                   std::span<const uint8_t>{
                       reinterpret_cast<const uint8_t*>(interfaceString.data()),
                       (interfaceString.size() + 1)},
                   true);
    }
}

void LynxUsbDevice::RunFtdiConfigureInternal() {
    auto now = wpi::Now();
    auto delta = now - discoverStartTime;

    if (delta <= MESSAGE_TIMEOUT) {
        return;
    }

    discoverStartTime = now;
    uint16_t packetId = *packetInterfaceId + 49;
    uint8_t buffer[1] = {1};

    // Configure FTDI for the first discovered module
    if (!discoveredAddresses.empty()) {
        uint8_t address = *discoveredAddresses.begin();
        SendPacket(address, MESSAGE_FTDI_RESET_CONTROL, packetId, buffer, true);
    }
}

void LynxUsbDevice::RunDiscoverySteps() {
    switch (deviceState) {
        case DeviceState::Discovering:
            RunDiscoverInternal();
            break;
        case DeviceState::ConfiguringInterface:
            RunInterfacePacketIdInternal();
            break;
        case DeviceState::ConfiguringFtdi:
            RunFtdiConfigureInternal();
            break;
        default:
            break;
    }
}

LynxUsbDevice::~LynxUsbDevice() noexcept {
    auto lock = serialPoll.lock();
    if (lock) {
        printf("Closing serial poller\n");
        lock->Stop();
        lock->Close();
    }
    if (serialFd != -1) {
        printf("Closed fd\n");
        close(serialFd);
    }
}

void LynxUsbDevice::SendPacket(uint8_t destAddr, uint8_t messageNumber, uint16_t packetTypeId,
                std::span<const uint8_t> payload, bool direct) {
    assert(payload.size() < (1024 - 11));
    uint16_t bytesToSend = 10 + payload.size() + 1;

    std::span<uint8_t> txBufferSpan = txBuffer;
    txBufferSpan = txBufferSpan.subspan(0, bytesToSend);
    txBufferSpan[0] = 0x44;
    txBufferSpan[1] = 0x4B;

    WriteUint16(txBufferSpan.subspan(2, 2), bytesToSend);
    txBufferSpan[4] = destAddr;
    txBufferSpan[5] = 0x00;
    txBufferSpan[6] = messageNumber;
    txBufferSpan[7] = 0x00;
    WriteUint16(txBufferSpan.subspan(8, 2), packetTypeId);

    if (!payload.empty()) {
        memcpy(&txBufferSpan[10], payload.data(), payload.size());
    }
    txBufferSpan[10 + payload.size()] =
        CalcChecksum(txBufferSpan.subspan(0, 10 + payload.size()));

    if (direct) {
        write(serialFd, txBufferSpan.data(), txBufferSpan.size());
    } else {
        writeBuffer.insert(writeBuffer.end(), txBufferSpan.begin(),
                           txBufferSpan.end());
        pendingWrites.emplace_back(txBufferSpan.size());
        pendingSends.push_back(
            {wpi::Now(), destAddr, messageNumber, packetTypeId});
        totalSent++;
    }
}

void LynxUsbDevice::DoRead() {
    ssize_t readVal = read(serialFd, readBuf, sizeof(readBuf));
    if (readVal <= 0) {
        printf("Read error\n");
        return;
    }

    static uint64_t readCount = 0;
    if (readCount % 100 == 0) {
        printf("DoRead: %zd bytes (call #%lu) state=%d outstanding=%d "
               "pendingSends=%zu sent=%lu recv=%lu\n",
               readVal, (unsigned long)readCount,
               static_cast<int>(deviceState), outstandingMessages,
               pendingSends.size(),
               (unsigned long)totalSent, (unsigned long)totalReceived);
    }
    readCount++;

    stateMachine.HandleBytes(
        std::span<const uint8_t>{readBuf, static_cast<size_t>(readVal)});
    Flush();
}

bool LynxUsbDevice::AllowSend() {
    return deviceState == DeviceState::Ready && sendState == SendState::ReadyToSend;
}

void LynxUsbDevice::StartTransaction(bool canDoEnable) {
    writeBuffer.clear();
    pendingWrites.clear();
    currentCount = 0;
    lastLoop = wpi::Now();
    canEnable = canDoEnable;
    haveBattery = false;
    haveBulk = false;
    haveModuleStatus = false;
    sendState = SendState::WaitingForPackets;
}

void LynxUsbDevice::Flush() {
    size_t available = pendingWrites.size();
    size_t allowed = MAX_NUM_OUTSTANDING_MESSAGES - outstandingMessages;
    size_t toWrite = (std::min)(allowed, available);
    size_t count = 0;
    for (size_t i = 0; i < toWrite; i++) {
        count += pendingWrites.front();
        pendingWrites.pop_front();
    }
    if (count == 0) {
        return;
    }

    write(serialFd, writeBuffer.data() + currentCount, count);
    outstandingMessages += toWrite;
    currentCount += count;
    if (sendState == SendState::WaitingForPackets) {
        sendState = SendState::WaitingForFinish;
        printf("Flush: total_pending=%zu first_batch=%zu\n", pendingWrites.size() + toWrite, toWrite);
    }
}

void LynxUsbDevice::HandlePayload(std::span<const uint8_t> data, uint8_t crc) {
    if (crc != PacketCrc(data)) {
        printf("CRC failure, bus will recover\n");
        if (deviceState == DeviceState::Ready && outstandingMessages > 0) {
            outstandingMessages--;
        }
        CheckSendStateAdvance();
        return;
    }

    uint16_t packetId = PacketId(data);
    uint8_t packetReferenceNumber = PacketReferenceNumber(data);
    uint8_t sourceAddress = PacketSourceAddress(data);
    auto payload = PacketPayloadBuffer(data);

    if (PacketIsNack(packetId)) {
        if (outstandingMessages > 0) outstandingMessages--;
        auto it = std::find_if(pendingSends.begin(), pendingSends.end(),
            [&](const PendingSend& p) {
                return p.dest == sourceAddress && p.msgNum == packetReferenceNumber;
            });
        if (it != pendingSends.end()) {
            pendingSends.erase(it);
            totalReceived++;
        }
        printf("Nack %d for message id %d from address %d\n",
               payload.empty() ? 0 : payload[0], packetReferenceNumber, sourceAddress);
        CheckSendStateAdvance();
        return;
    }

    if (PacketIsDiscover(packetId)) {
        if (deviceState == DeviceState::Discovering && payload.size() > 0) {
            // payload[0]: 1=parent/has children, 0=leaf/child module
            // For children, set parent to the first discovered module
            uint8_t parentAddress;
            if (payload[0] == 1 || discoveredAddresses.empty()) {
                parentAddress = 0xFF;  // This is a parent module
            } else {
                parentAddress = *discoveredAddresses.begin();  // Child of first parent
            }
            if (discoveredAddresses.find(sourceAddress) == discoveredAddresses.end()) {
                OnModuleDiscovered(sourceAddress, parentAddress);
            }
            // After first response, use a shorter timeout to wait for more modules.
            if (!discoveryComplete) {
                discoveryComplete = true;
                discoverStartTime = wpi::Now();
            }
        }
        return;
    } else if (PacketIsAck(packetId) &&
               packetReferenceNumber == MESSAGE_FTDI_RESET_CONTROL) {
        configuredFtdiReset = true;
        deviceState = DeviceState::Ready;
        discoverStartTime = 0;
        printf("Device initialization complete\n");
        return;
    } else if (PacketIsQueryInterface(packetId)) {
        if (deviceState == DeviceState::ConfiguringInterface && !packetInterfaceId.has_value()) {
            packetInterfaceId = ReadUint16(payload);
            if (isUartConnection) {
                // No FTDI chip on internal UART — skip straight to Ready
                deviceState = DeviceState::Ready;
                printf("UART device initialization complete (no FTDI)\n");
            } else {
                deviceState = DeviceState::ConfiguringFtdi;
            }
            discoverStartTime = 0;
        }
        return;
    }

    if (outstandingMessages > 0) outstandingMessages--;
    {
        auto it = std::find_if(pendingSends.begin(), pendingSends.end(),
            [&](const PendingSend& p) {
                return p.dest == sourceAddress && p.msgNum == packetReferenceNumber;
            });
        if (it != pendingSends.end()) {
            pendingSends.erase(it);
            totalReceived++;
        }
    }

    // Handle module-specific responses
    auto module = GetModule(sourceAddress);
    if (!module) {
        printf("Received packet from unknown module address %d\n", sourceAddress);
        CheckSendStateAdvance();
        return;
    }

    // Process the message based on its type
    switch (packetReferenceNumber) {
        case MESSAGE_MODULE_STATUS:
            haveModuleStatus = true;
            break;
        case MESSAGE_BULK_INPUT: {
            haveBulk = true;

            if (module->HasMotors() && payload.size() >= 26) {
                module->GetMotor(0).SetEncoder(ReadInt32(payload.subspan(1)), ReadInt16(payload.subspan(18)));
                module->GetMotor(1).SetEncoder(ReadInt32(payload.subspan(5)), ReadInt16(payload.subspan(20)));
                module->GetMotor(2).SetEncoder(ReadInt32(payload.subspan(9)), ReadInt16(payload.subspan(22)));
                module->GetMotor(3).SetEncoder(ReadInt32(payload.subspan(13)), ReadInt16(payload.subspan(24)));
            }
            break;
        }
        case MESSAGE_BATTERY_VOLTAGE: {
            haveBattery = true;
            double battery = ReadInt16(payload) / 1000.0;
            module->lastBattery = battery;
            module->batteryVoltagePublisher.Set(battery);
            break;
        }
        default:
            break;
    }

    CheckSendStateAdvance();
}

void LynxUsbDevice::CheckSendStateAdvance() {
    if (pendingWrites.empty() && outstandingMessages == 0) {
        sendState = SendState::ReadyToSend;
    }
}

void LynxUsbDevice::Recover() {
    printf("Recover: outstanding=%d pending=%zu sendState=%d\n",
           outstandingMessages, pendingWrites.size(), static_cast<int>(sendState));
    auto now = wpi::Now();
    printf("Recover: %zu unanswered sends (totalSent=%lu totalReceived=%lu diff=%ld):\n",
           pendingSends.size(), (unsigned long)totalSent,
           (unsigned long)totalReceived,
           (long)totalSent - (long)totalReceived);
    for (auto& p : pendingSends) {
        printf("  unanswered: dest=%d msgNum=%d packetTypeId=0x%04X age_ms=%lu\n",
               p.dest, p.msgNum, p.packetTypeId,
               (unsigned long)((now - p.sentAt) / 1000));
    }
    pendingSends.clear();
    stateMachine.Reset();
    outstandingMessages = 0;
    writeBuffer.clear();
    pendingWrites.clear();
    currentCount = 0;
    sendState = SendState::ReadyToSend;
}

// Communication method implementations
void LynxUsbDevice::SendBatteryRequest(uint8_t moduleAddress) {
    if (!packetInterfaceId.has_value()) return;
    
    uint16_t packetId = *packetInterfaceId + 7;
    uint8_t buffer[2] = {BATTERY_ADC, 0};
    SendPacket(moduleAddress, MESSAGE_BATTERY_VOLTAGE, packetId, buffer);
}

void LynxUsbDevice::SendBulkInput(uint8_t moduleAddress) {
    if (!packetInterfaceId.has_value()) return;
    
    uint16_t packetId = *packetInterfaceId + 0;
    SendPacket(moduleAddress, MESSAGE_BULK_INPUT, packetId, {});
}

void LynxUsbDevice::SendKeepAlive(uint8_t moduleAddress) {
    SendPacket(moduleAddress, MESSAGE_KEEP_ALIVE, KEEP_ALIVE_ID, {});
}

void LynxUsbDevice::GetModuleStatus(uint8_t moduleAddress) {
    uint8_t clear = 1;
    SendPacket(moduleAddress, MESSAGE_MODULE_STATUS, MODULE_STATUS_ID,
               std::span<const uint8_t>{&clear, 1});
}

void LynxUsbDevice::SendMotorConstantPower(uint8_t moduleAddress, uint8_t channel, double power) {
    if (!packetInterfaceId.has_value()) return;
    
    power = std::clamp(power, -1.0, 1.0);
    uint16_t packetId = *packetInterfaceId + 15;
    int16_t adjustedPowerLevel = (int16_t)(power * POWER_CONVERSION);

    uint8_t buffer[3] = {channel, (uint8_t)(adjustedPowerLevel),
                         (uint8_t)(adjustedPowerLevel >> 8)};

    SendPacket(moduleAddress, MESSAGE_MOTOR_SET_CONSTANT_POWER_0 + channel, packetId, buffer);
}

void LynxUsbDevice::SendMotorEnable(uint8_t moduleAddress, uint8_t channel, bool enable) {
    if (!packetInterfaceId.has_value()) return;
    
    uint16_t packetId = *packetInterfaceId + 10;
    uint8_t enableVal = enable ? 1 : 0;
    uint8_t buffer[2] = {channel, enableVal};
    SendPacket(moduleAddress, MESSAGE_MOTOR_ENABLE_0 + channel, packetId, buffer);
}

void LynxUsbDevice::SendServoPulseWidth(uint8_t moduleAddress, uint8_t channel, uint16_t pulseWidth) {
    if (!packetInterfaceId.has_value()) return;
    
    if (pulseWidth == 0) return;
    
    uint16_t packetId = *packetInterfaceId + 33;
    uint8_t buffer[3] = {channel, (uint8_t)(pulseWidth), (uint8_t)(pulseWidth >> 8)};
    SendPacket(moduleAddress, MESSAGE_SERVO_PULSE_WIDTH_0 + channel, packetId, buffer);
}

void LynxUsbDevice::SendServoEnable(uint8_t moduleAddress, uint8_t channel, bool enable) {
    if (!packetInterfaceId.has_value()) return;
    
    uint16_t packetId = *packetInterfaceId + 35;
    uint8_t enableVal = enable ? 1 : 0;
    uint8_t buffer[2] = {channel, enableVal};
    SendPacket(moduleAddress, MESSAGE_SERVO_ENABLE_0 + channel, packetId, buffer);
}

// I2C method implementations
void LynxUsbDevice::SendI2CWriteMultipleBytes(uint8_t moduleAddress, uint8_t channel, uint8_t deviceAddress, const std::vector<uint8_t>& data) {
    if (!packetInterfaceId.has_value() || data.empty()) return;
    
    uint16_t packetId = *packetInterfaceId + 38;
    std::vector<uint8_t> buffer;
    buffer.push_back(channel);
    buffer.push_back(deviceAddress);
    buffer.push_back(static_cast<uint8_t>(data.size()));
    buffer.insert(buffer.end(), data.begin(), data.end());
    
    SendPacket(moduleAddress, MESSAGE_I2C_WRITE_MULTIPLE_BYTES, packetId, buffer);
}

void LynxUsbDevice::SendI2CReadMultipleBytes(uint8_t moduleAddress, uint8_t channel, uint8_t deviceAddress, uint8_t numBytes) {
    if (!packetInterfaceId.has_value()) return;
    
    uint16_t packetId = *packetInterfaceId + 40;
    uint8_t buffer[3] = {channel, deviceAddress, numBytes};
    SendPacket(moduleAddress, MESSAGE_I2C_READ_MULTIPLE_BYTES, packetId, buffer);
}

void LynxUsbDevice::SendI2CConfigureChannel(uint8_t moduleAddress, uint8_t channel, uint8_t speedCode) {
    if (!packetInterfaceId.has_value()) return;
    
    uint16_t packetId = *packetInterfaceId + 43;
    uint8_t buffer[2] = {channel, speedCode};
    SendPacket(moduleAddress, MESSAGE_I2C_CONFIGURE_CHANNEL, packetId, buffer);
}

void LynxUsbDevice::SendI2CBlockReadConfig(uint8_t moduleAddress, uint8_t channel, uint8_t deviceAddress, uint8_t startRegister, uint8_t numBytes, uint8_t interval) {
    if (!packetInterfaceId.has_value()) return;
    
    uint16_t packetId = *packetInterfaceId + 50;
    uint8_t buffer[5] = {channel, deviceAddress, startRegister, numBytes, interval};
    SendPacket(moduleAddress, MESSAGE_I2C_BLOCK_READ_CONFIG, packetId, buffer);
}

void LynxUsbDevice::SendMotorMode(uint8_t moduleAddress, uint8_t channel, bool floatVal) {
    if (!packetInterfaceId.has_value()) return;
    
    uint16_t packetId = *packetInterfaceId + 8;
    uint8_t doFloat = floatVal ? 1 : 0;
    uint8_t cmdPayload[3] = {channel, 0, doFloat};
    SendPacket(moduleAddress, MESSAGE_MOTOR_SET_RUN_MODE_0 + channel, packetId, cmdPayload);
}

void LynxUsbDevice::SendMotorCurrentRequest(uint8_t moduleAddress, uint8_t channel) {
    if (!packetInterfaceId.has_value()) return;
    
    uint16_t packetId = *packetInterfaceId + 7;
    uint8_t adcChannel = MOTOR_0_ADC + channel;
    uint8_t buffer[2] = {adcChannel, 0};
    SendPacket(moduleAddress, MESSAGE_MOTOR_GET_CURRENT_0 + channel, packetId, buffer);
}

void LynxUsbDevice::SendEncoderResetRequest(uint8_t moduleAddress, uint8_t channel) {
    if (!packetInterfaceId.has_value()) return;
    
    uint16_t packetId = *packetInterfaceId + 14;
    uint8_t buffer[1] = {channel};
    SendPacket(moduleAddress, MESSAGE_MOTOR_RESET_ENCODER_0 + channel, packetId, buffer);
}

void LynxUsbDevice::SendServoConfiguration(uint8_t moduleAddress, uint8_t channel, uint16_t framePeriod) {
    if (!packetInterfaceId.has_value()) return;
    
    if (framePeriod <= 1) return;
    
    uint16_t packetId = *packetInterfaceId + 31;
    uint8_t buffer[3] = {channel, (uint8_t)(framePeriod), (uint8_t)(framePeriod >> 8)};
    SendPacket(moduleAddress, MESSAGE_SERVO_CONFIGURATION_0 + channel, packetId, buffer);
}