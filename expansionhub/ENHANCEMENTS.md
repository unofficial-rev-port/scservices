# ExpansionHub Service Enhancements

This document describes the enhancements made to the ExpansionHub service to support RS485 chaining, I2C devices, and servo hubs.

## Overview

The enhanced ExpansionHub service now supports:

1. **RS485 Chaining**: Multiple Lynx devices connected via RS485 daisy-chain
2. **I2C Device Support**: Full I2C sensor and device integration
3. **Servo Hub Support**: REV Servo Hub (servo-only Lynx modules)
4. **Multi-Module Management**: Unified interface for managing different module types

## Key Components

### New Files Added

1. **LynxModuleTypes.h/cpp**: Defines different Lynx module types and capabilities
2. **I2CNtState.h/cpp**: NetworkTables integration for I2C channels
3. **LynxModuleNtState.h/cpp**: Per-module state management supporting different module types
4. **LynxUsbDevice.h/cpp**: Enhanced USB device handler supporting multiple modules
5. **main_enhanced.cpp**: Enhanced main application supporting the new architecture

### Enhanced Files

1. **MessageNumbers.h**: Added I2C message constants
2. **MotorNtState.h/cpp**: Added module-address-aware initialization
3. **ServoNtState.h/cpp**: Added module-address-aware initialization
4. **CMakeLists.txt**: Updated to include new source files

## Architecture Changes

### Module Discovery and Management

The new architecture supports automatic discovery of chained Lynx modules:

- **Discovery Process**: Broadcasts discovery packets to find all connected modules
- **Address Management**: Tracks module addresses and parent-child relationships
- **Module Type Detection**: Identifies Control Hubs, Expansion Hubs, and Servo Hubs
- **Dynamic Configuration**: Adapts functionality based on detected module capabilities

### Multi-Module Communication

- **Per-Module Addressing**: All commands are sent to specific module addresses
- **Chained Device Support**: Handles RS485-connected devices properly
- **Bulk Operations**: Efficiently manages multiple modules on the same USB connection

### I2C Integration

The enhanced service provides full I2C support:

- **4 I2C Channels**: Supports up to 4 I2C channels per module
- **Block Read Configuration**: Configurable block read operations for sensors
- **Speed Control**: Configurable I2C bus speeds
- **Status Monitoring**: Real-time I2C transaction status

### NetworkTables Structure

The new NetworkTables structure organizes devices hierarchically:

```
/ExpansionHub/
  ├── 0/                    # Bus 0
  │   ├── Module_1/         # Module at address 1
  │   │   ├── Status/       # Module status and info
  │   │   ├── Motor_0/      # Motor 0 (if supported)
  │   │   ├── Motor_1/      # Motor 1 (if supported)
  │   │   ├── Servo_0/      # Servo 0
  │   │   ├── Servo_1/      # Servo 1
  │   │   └── I2C_0/        # I2C Channel 0 (if supported)
  │   │       ├── Config/   # Configuration
  │   │       └── Status/   # Status and data
  │   └── Module_2/         # Module at address 2 (if chained)
  └── 1/                    # Bus 1 (if present)
```

## Module Type Support

### Control Hub / Expansion Hub
- **Motors**: 4 DC motor ports with encoders
- **Servos**: 6 servo/PWM ports  
- **I2C**: 4 I2C channels with full functionality
- **Sensors**: Built-in IMU, voltage monitoring

### Servo Hub
- **Motors**: None (servo-only device)
- **Servos**: 6 servo/PWM ports
- **I2C**: None
- **Features**: Lightweight, cost-effective servo control

## I2C Features

### Channel Configuration
```cpp
// Configure I2C channel speed
/ExpansionHub/0/Module_1/I2C_0/Config/SpeedCode = 0  // 100kHz
/ExpansionHub/0/Module_1/I2C_0/Config/SpeedCode = 1  // 400kHz
```

### Block Read Setup
```cpp
// Configure automatic sensor reading
/ExpansionHub/0/Module_1/I2C_0/Config/BlockReadAddress = 0x20    // Device address
/ExpansionHub/0/Module_1/I2C_0/Config/BlockReadRegister = 0x00   // Start register
/ExpansionHub/0/Module_1/I2C_0/Config/BlockReadBytes = 6         // Bytes to read
/ExpansionHub/0/Module_1/I2C_0/Config/BlockReadInterval = 20     // Interval (ms)
```

### Status Monitoring
```cpp
// Read I2C status and data
int status = /ExpansionHub/0/Module_1/I2C_0/Status/Status;  // Transaction status
byte[] data = /ExpansionHub/0/Module_1/I2C_0/Status/Data;   // Received data
```

## RS485 Chaining

The enhanced service supports RS485 daisy-chaining:

1. **Auto-Discovery**: Automatically finds all modules in the chain
2. **Parent-Child Tracking**: Maintains chain hierarchy information
3. **Address Management**: Ensures proper addressing for chained devices
4. **Error Handling**: Robust error recovery for chain communication issues

## Usage Examples

### Basic Setup
Replace the old main.cpp with main_enhanced.cpp and rebuild:

```bash
cd scservices/expansionhub
mv main.cpp main_original.cpp
mv main_enhanced.cpp main.cpp
make
```

### Configuring I2C Sensors

```cpp
// Example: Configure a color sensor on I2C channel 0
ntInstance.GetTable("/ExpansionHub/0/Module_1/I2C_0/Config")
    ->GetEntry("BlockReadAddress").SetDouble(0x29);    // Color sensor address
ntInstance.GetTable("/ExpansionHub/0/Module_1/I2C_0/Config")
    ->GetEntry("BlockReadRegister").SetDouble(0x14);   // Color data register
ntInstance.GetTable("/ExpansionHub/0/Module_1/I2C_0/Config")
    ->GetEntry("BlockReadBytes").SetDouble(8);         // 8 bytes of color data
ntInstance.GetTable("/ExpansionHub/0/Module_1/I2C_0/Config")
    ->GetEntry("BlockReadInterval").SetDouble(50);     // 50ms intervals
```

### Reading Sensor Data

```cpp
// Read color sensor data
auto dataEntry = ntInstance.GetTable("/ExpansionHub/0/Module_1/I2C_0/Status")
    ->GetEntry("Data");
std::vector<uint8_t> colorData = dataEntry.GetRaw({});
```

## Benefits

1. **Expanded Hardware Support**: Supports more REV hardware configurations
2. **Better Scalability**: Can handle larger robot configurations with multiple hubs
3. **I2C Integration**: Direct support for I2C sensors without external bridges
4. **Cost Optimization**: Servo hubs provide cost-effective servo control
5. **Future-Proofing**: Architecture supports future REV hardware additions

## Migration Guide

### From Original Service

1. **Backup**: Save your current main.cpp as main_original.cpp
2. **Update**: Replace with main_enhanced.cpp and rename to main.cpp
3. **Rebuild**: Recompile the service
4. **Test**: Verify basic functionality with existing hardware
5. **Configure**: Add I2C or chained device configurations as needed

### NetworkTables Changes

The enhanced service uses a different NetworkTables structure. Update your client code to use the new hierarchical paths:

**Old**: `/rhsp/0/motor0/setpoint`
**New**: `/ExpansionHub/0/Module_1/Motor_0/setpoint`

## Troubleshooting

### Common Issues

1. **Module Not Discovered**: Check RS485 wiring and termination
2. **I2C Not Working**: Verify device addresses and pull-up resistors
3. **Performance Issues**: Reduce I2C block read frequency for multiple channels

### Debug Information

The enhanced service provides detailed logging:
- Module discovery events
- I2C transaction status
- Chain communication errors
- Performance metrics

## Future Enhancements

Planned improvements include:
- **Hot-plug Support**: Dynamic module addition/removal
- **Advanced I2C**: Support for more I2C protocols and addressing modes  
- **Diagnostics**: Enhanced monitoring and diagnostic capabilities
- **Configuration UI**: Web-based configuration interface