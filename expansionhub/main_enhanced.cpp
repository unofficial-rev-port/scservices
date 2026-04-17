#if defined(__linux__) && defined(MRC_DAEMON_BUILD)
#include <signal.h>
#endif
#include <stdio.h>

#include "version.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include <filesystem>

#include <wpi/net/EventLoopRunner.hpp>
#include <wpi/net/uv/Poll.hpp>
#include <wpi/net/uv/FsEvent.hpp>
#include <wpi/net/uv/Timer.hpp>

#include "wpi/nt/NetworkTableInstance.hpp"
#include "wpi/nt/RawTopic.hpp"
#include "wpi/nt/IntegerTopic.hpp"
#include "wpi/nt/DoubleTopic.hpp"

#include "systemd/sd-device.h"

#include "SerialPort.h"

#include <deque>

#include "wpi/nt/BooleanTopic.hpp"

#include <wpi/util/timestamp.hpp>

#include "ReceiveStateMachine.h"
#include "MessageNumbers.h"

#include "wpi/math/controller/PIDController.hpp"
#include "wpi/math/controller/SimpleMotorFeedforward.hpp"

#include <wpi/units/length.hpp>
#include <wpi/units/velocity.hpp>
#include <wpi/units/voltage.hpp>
#include <wpi/units/acceleration.hpp>

#include "ExpansionHubNtState.h"
#include "LynxUsbDevice.h"
#include "EnabledState.h"
#include "SystemDUsbMonitor.h"

#define NUM_USB_BUSES 4
#define INTERNAL_UART_BUS 4
#define NUM_TOTAL_BUSES (NUM_USB_BUSES + 1)
#define INTERNAL_UART_PATH "/dev/ttyS1"

// EXPERIMENT 2026-04-16:
//   Phase 1 (DEBUG_ONLY_MODULE_2=1): proved module 2 is reachable post-
//   discovery. With strict serial sends (MAX_NUM_OUTSTANDING_MESSAGES=1) and
//   *only* dest=2 traffic, response rate was ~99.9%, no Recover events.
//   So neither "no relay" nor pure first-tick burst is the root cause.
//
//   Phase 2 (DEBUG_ONLY_MODULE_2=0 here, MAX_NUM_OUTSTANDING_MESSAGES=1
//   still): re-enable dest=173 traffic but keep strict-serial sends. This
//   isolates "interleaving dest=2 with dest=173" as the variable. If dest=2
//   responses now drop again, the parent's RS485 forwarding is being
//   starved by local-handle-the-packet work whenever dest=173 traffic is
//   mixed in — and the fix is per-destination outstanding tracking, not
//   raising or lowering the global cap.
#define DEBUG_ONLY_MODULE_2 0

struct LynxBusState {
    uint64_t lastLoop = wpi::util::Now();

    int socketHandle{-1};

    const wpi::nt::NetworkTableInstance* ntInstance;

    unsigned busId{0};

    std::unique_ptr<eh::LynxUsbDevice> currentDevice;

    ~LynxBusState() {
        if (socketHandle != -1) {
            close(socketHandle);
        }
    }

    void OnUpdate(bool canEnable);

    void SendCommands(bool canEnable, bool deviceReset);

    bool StartUvLoop(unsigned bus, const wpi::nt::NetworkTableInstance& ntInst,
                     wpi::net::uv::Loop& loop);

    void OnDeviceAdded(std::unique_ptr<eh::LynxUsbDevice> device);

    void OnDeviceRemoved(std::string_view port);
};

static uint64_t debugPrintCounter = 0;

void LynxBusState::SendCommands(bool canEnable, bool deviceReset) {
    if (!currentDevice) return;

    for (auto& module : currentDevice->GetModules()) {
        if (!module) continue;

        uint8_t moduleAddress = module->moduleInfo.address;

#if DEBUG_ONLY_MODULE_2
        if (moduleAddress != 2) continue;
#endif

        // Periodic debug: print state every ~2 seconds (assuming 12ms loop)
        if (debugPrintCounter % 166 == 0) {
            auto pw = module->GetServo(0).pulseWidthSubscriber.Get(1500);
            double motorSetpoint = module->HasMotors() ? module->GetMotor(0).setpointSubscriber.Get(0) : -999;
            printf("[bus%d/mod%d] canEnable=%d servo0_pw=%ld motor0_sp=%.3f bat=%.2f\n",
                   busId, moduleAddress, canEnable ? 1 : 0, (long)pw, motorSetpoint, module->lastBattery);
        }
        debugPrintCounter++;

        if (deviceReset) {
            // Reset subscribers for all motors if device was reset
            if (module->HasMotors()) {
                for (int i = 0; i < 4; i++) {
                    module->GetMotor(i).enabledSubscriber.ForceReset();
                    module->GetMotor(i).floatOn0Subscriber.ForceReset();
                }
            }

            // Reset subscribers for all servos
            for (int i = 0; i < 6; i++) {
                module->GetServo(i).enabledSubscriber.ForceReset();
                module->GetServo(i).framePeriodSubscriber.ForceReset();
            }
        }

        // Send motor commands if this module supports motors
        if (module->HasMotors()) {
            for (int i = 0; i < 4; i++) {
                auto [power, mode] = module->GetMotor(i).ComputeMotorPower(module->lastBattery);
                currentDevice->SendMotorConstantPower(moduleAddress, i, power);
            }

            // Get motor currents
            for (int i = 0; i < 4; i++) {
                currentDevice->SendMotorCurrentRequest(moduleAddress, i);
            }

            // Send motor configuration
            for (int i = 0; i < 4; i++) {
                auto sendFloatOn0 = module->GetMotor(i).floatOn0Subscriber.Get();
                if (sendFloatOn0.has_value()) {
                    currentDevice->SendMotorMode(moduleAddress, i, *sendFloatOn0);
                    module->GetMotor(i).floatOn0Subscriber.Ack();
                }
            }

            for (int i = 0; i < 4; i++) {
                auto sendEnable = module->GetMotor(i).enabledSubscriber.GetWithCanEnable(canEnable);
                if (sendEnable.has_value()) {
                    currentDevice->SendMotorEnable(moduleAddress, i, *sendEnable);
                    module->GetMotor(i).enabledSubscriber.Ack();
                }
            }
        }

        // Send servo commands (all modules have servos)
        for (int i = 0; i < 6; i++) {
            auto servoConfig = module->GetServo(i).framePeriodSubscriber.Get();
            if (servoConfig.has_value()) {
                currentDevice->SendServoConfiguration(moduleAddress, i, *servoConfig);
                module->GetServo(i).framePeriodSubscriber.Ack();
            }
        }

        for (int i = 0; i < 6; i++) {
            currentDevice->SendServoPulseWidth(
                moduleAddress, i, module->GetServo(i).pulseWidthSubscriber.Get(1500));
        }

        for (int i = 0; i < 6; i++) {
            auto sendEnable = module->GetServo(i).enabledSubscriber.GetWithCanEnable(canEnable);
            if (sendEnable.has_value()) {
                currentDevice->SendServoEnable(moduleAddress, i, *sendEnable);
                module->GetServo(i).enabledSubscriber.Ack();
            }
        }

        // Handle I2C for modules that support it
        if (module->HasI2C()) {
            for (int i = 0; i < NUM_I2C_CHANNELS; i++) {
                auto& channel = module->GetI2CChannel(i);
                
                // Configure channel speed if needed
                auto speedCode = channel.speedCodeSubscriber.Get();
                if (speedCode.has_value()) {
                    currentDevice->SendI2CConfigureChannel(moduleAddress, i, *speedCode);
                    channel.speedCodeSubscriber.Ack();
                }

                // Configure block read if needed
                auto blockAddr = channel.blockReadAddressSubscriber.Get();
                auto blockReg = channel.blockReadRegisterSubscriber.Get();
                auto blockBytes = channel.blockReadBytesSubscriber.Get();
                auto blockInterval = channel.blockReadIntervalSubscriber.Get();

                if (blockAddr.has_value() && blockReg.has_value() &&
                    blockBytes.has_value() && blockInterval.has_value() &&
                    *blockAddr != 0 && *blockBytes != 0) {
                    currentDevice->SendI2CBlockReadConfig(
                        moduleAddress, i, *blockAddr, *blockReg, *blockBytes, *blockInterval);
                    channel.blockReadAddressSubscriber.Ack();
                    channel.blockReadRegisterSubscriber.Ack();
                    channel.blockReadBytesSubscriber.Ack();
                    channel.blockReadIntervalSubscriber.Ack();
                }
            }
        }
    }

    currentDevice->Flush();
}

void LynxBusState::OnDeviceAdded(std::unique_ptr<eh::LynxUsbDevice> device) {
    currentDevice = std::move(device);
    currentDevice->SetNtInstance(ntInstance);

    printf("Lynx USB device added\n");
}

void LynxBusState::OnDeviceRemoved(std::string_view path) {
    if (currentDevice && path == currentDevice->SerialPath()) {
        printf("Lynx USB device removed\n");
        currentDevice.reset();
    }
}

void LynxBusState::OnUpdate(bool canEnable) {
    if (!currentDevice) {
        return;
    }

    if (!currentDevice->HasFinishedInitialization()) {
        currentDevice->RunDiscoverySteps();
        return;
    }

    auto now = wpi::util::Now();
    auto delta = now - lastLoop;

    bool allowSend = currentDevice->AllowSend();

    if (!allowSend && delta < 5000000) {
        printf("Skipping due to outstanding\n");
        return;
    } else if (!allowSend && delta >= 5000000) {
        printf("5 second timeout. Attempting to recover\n");
        currentDevice->Recover();
    }

    lastLoop = now;

    if (delta > 23000) {
        printf("Delta time %lu\n", delta);
    }

    currentDevice->StartTransaction(canEnable);

    // Send initial commands to all modules
    for (auto& module : currentDevice->GetModules()) {
        if (!module) continue;

        uint8_t moduleAddress = module->moduleInfo.address;

#if DEBUG_ONLY_MODULE_2
        if (moduleAddress != 2) continue;
#endif

        // Send keep alive (most important)
        currentDevice->SendKeepAlive(moduleAddress);
        currentDevice->GetModuleStatus(moduleAddress);

        // Handle encoder resets for modules with motors
        if (module->HasMotors()) {
            for (int i = 0; i < 4; i++) {
                auto reset = module->GetMotor(i).resetEncoderSubscriber.GetAtomic(false);
                if (reset.time != module->GetMotor(i).lastResetTime) {
                    module->GetMotor(i).lastResetTime = reset.time;
                    module->GetMotor(i).doReset = true;
                }

                if (module->GetMotor(i).doReset) {
                    currentDevice->SendEncoderResetRequest(moduleAddress, i);
                }
            }
        }

        // Request telemetry
        currentDevice->SendBatteryRequest(moduleAddress);
        currentDevice->SendBulkInput(moduleAddress);
    }

    // Send all the control commands
    SendCommands(canEnable, false);
}

static void OnDeviceRemoved(
    std::array<LynxBusState, NUM_TOTAL_BUSES>& states,
    const std::string& devPath) {
    std::string_view path = devPath;

    for (auto&& i : states) {
        i.OnDeviceRemoved(path);
    }
}

static void OnDeviceAdded(wpi::net::uv::Loop& loop,
                          std::array<LynxBusState, NUM_TOTAL_BUSES>& states,
                          int busNum, const std::string& devPath) {
    if (states[busNum].currentDevice != nullptr) {
        printf("Received duplicate bus, likely race condition\n");
        return;
    }

    int ret = eh::OpenRhspSerialPort(devPath.c_str());
    if (ret < 0) {
        printf("OpenRhspSerialPort failed %d\n", ret);
        return;
    }

    auto device = std::make_unique<eh::LynxUsbDevice>();
    bool isInit = device->Initialize(loop, ret, devPath, busNum);
    printf("initialized %d\n", isInit ? 1 : 0);

    device->RunDiscoverySteps();

    states[busNum].OnDeviceAdded(std::move(device));
}

bool LynxBusState::StartUvLoop(unsigned bus,
                               const wpi::nt::NetworkTableInstance& ntInst,
                               wpi::net::uv::Loop& loop) {
    if (bus >= NUM_TOTAL_BUSES) {
        return false;
    }

    busId = bus;
    ntInstance = &ntInst;

    return true;
}

int main() {
    // Force line-buffered stdout so journald sees diagnostic output promptly
    // instead of waiting for the default 8KB block buffer to fill.
    setvbuf(stdout, nullptr, _IOLBF, 0);

    printf("Starting Enhanced ExpansionHub Daemon\n");
    printf("\tSupports: RS485 chaining, I2C devices, Servo hubs\n");
    printf("\tBuild Hash: %s\n", MRC_GetGitHash());
    printf("\tBuild Timestamp: %s\n", MRC_GetBuildTimestamp());

#if defined(__linux__) && defined(MRC_DAEMON_BUILD)
    sigset_t signal_set;
    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGTERM);
    sigaddset(&signal_set, SIGINT);
    sigprocmask(SIG_BLOCK, &signal_set, nullptr);
#endif

    eh::EnabledState enabledState;

    if (!enabledState.Initialize()) {
        printf("Failed to open control data.\n");
        return -1;
    }

    std::array<LynxBusState, NUM_TOTAL_BUSES> states;
    eh::SystemDUsbMonitor usbMonitor{
        [&states](wpi::net::uv::Loop& loop, int busNum, const std::string& devPath) {
            OnDeviceAdded(loop, states, busNum, devPath);
        },
        [&states](const std::string& devPath) {
            OnDeviceRemoved(states, devPath);
        }};

    auto ntInst = wpi::nt::NetworkTableInstance::Create();
    ntInst.SetServer({"localhost"}, 6810);
    ntInst.StartClient("LynxExpansionHubDaemon");

    wpi::net::EventLoopRunner loopRunner;

    struct LoopStorage {
        std::array<LynxBusState, NUM_TOTAL_BUSES>* hubStates;
        wpi::net::uv::Loop* loop;
    } loopStorage{
        .hubStates = &states,
        .loop = nullptr,
    };

    bool success = false;
    loopRunner.ExecSync([&success, &states, &loopStorage, &ntInst,
                         &usbMonitor, &enabledState](wpi::net::uv::Loop& loop) {
        loopStorage.loop = &loop;
        for (size_t i = 0; i < states.size(); i++) {
            success = states[i].StartUvLoop(i, ntInst, loop);
            if (!success) {
                return;
            }
        }

        auto usbMonResult = usbMonitor.Initialize(&loop);
        if (!usbMonResult) {
            printf("SystemDUsbMonitor::Initialize failed\n");
            success = false;
            return;
        }

        auto poll = wpi::net::uv::Poll::Create(loop, usbMonitor.GetFd());
        if (!poll) {
            printf("Poll create failed\n");
            success = false;
            return;
        }

        poll->pollEvent.connect(
            [&usbMonitor](int flags) { usbMonitor.HandleEvent(); });

        poll->Start(UV_READABLE);

        auto sendTimer = wpi::net::uv::Timer::Create(loop);

        sendTimer->timeout.connect([&states, &enabledState]() {
            bool system_watchdog = enabledState.IsEnabled();

            static uint64_t watchdogPrintCounter = 0;
            if (watchdogPrintCounter % 500 == 0) {
                printf("system_watchdog=%d\n", system_watchdog ? 1 : 0);
            }
            watchdogPrintCounter++;

            for (auto&& dev : states) {
                dev.OnUpdate(system_watchdog);
            }
        });

        wpi::units::millisecond_t millis = eh::PidConstants::Period;
        uint64_t rawMillis = static_cast<uint64_t>(millis.value());

        sendTimer->Start(wpi::net::uv::Timer::Time{rawMillis},
                         wpi::net::uv::Timer::Time{rawMillis});

        usbMonitor.DoInitialCheck();

        // Open internal UART connection to onboard Lynx module
        int uartFd = eh::OpenRhspSerialPort(INTERNAL_UART_PATH);
        if (uartFd >= 0) {
            auto uartDevice = std::make_unique<eh::LynxUsbDevice>();
            bool uartInit = uartDevice->Initialize(loop, uartFd, INTERNAL_UART_PATH, INTERNAL_UART_BUS, true);
            printf("Internal UART (%s) initialized: %d\n", INTERNAL_UART_PATH, uartInit ? 1 : 0);
            if (uartInit) {
                states[INTERNAL_UART_BUS].OnDeviceAdded(std::move(uartDevice));
            }
        } else {
            printf("Internal UART (%s) not available: %d\n", INTERNAL_UART_PATH, uartFd);
        }
    });

    if (!success) {
        loopRunner.Stop();
        return -1;
    }

    {
#if defined(__linux__) && defined(MRC_DAEMON_BUILD)
        int sig = 0;
        sigwait(&signal_set, &sig);
#else
        (void)getchar();
#endif
    }
    ntInst.StopClient();
    wpi::nt::NetworkTableInstance::Destroy(ntInst);

    loopRunner.Stop();

    return 0;
}