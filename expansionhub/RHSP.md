# RHSP
RHSP is the REV Hub Serial Protocol. Is the protocol that the Lynx and other REV hubs communicate with. RHSP runs over serial. Between hubs it runs over RS485. For the most part it is half duplex. Peripheral devices only output if they receive a packet from their controller device. The whole protocol is the little endian, so it is easy to serialize on the little endian ARM MCU that runs the Lynx.

## Resources
https://github.com/OpenFTC/Extracted-RC  
https://github.com/REVrobotics/REV-Hub-Serial-Protocol-Analyzer-For-Saleae/blob/main/HighLevelAnalyzer.py  
https://github.com/REVrobotics/node-rhsplib/tree/main/packages/rhsplib/librhsp  

## Terminology
- Hub = a Lynx or other REV hub
- Controller = either librhsp or the FTC Robot Controller App
- Parent (Hub) = the hub that is connected to the controller via UART 0
- Child (Hub) = a hub that is connected to the parent device via RS485 on UART 1
- Command = a packet send by the controller to a hub
- Command ID = the id number of a command
- Reply = a packet send in response to a command from controller by a hub
- Interface = a hub specific set of commands
- Valid Packet = a packet that has the correct format, but may have an invalid command or payload

## Timeout
If a hub hasn't received a packet in 2500 ms it goes into timeout mode. In this mode the led will indicate the timeout event to the user and all motion will be stopped by going into failsafe mode. Any valid packet a hub receives will reset its timeout even if the packet gets nacked. Invalid packets, including ones with bad checksums or ones that don't have the magic number, will not reset the timeout.

## Packet format

```
[0] = 0x44
[1] = 0x4B
[2..3] (uint16_t) packet Size
[4] = Dest address
[5] = Src address
[6] = Message number
[7] = Reference number
[8..9] (uint16_t) command ID
[10..10 + Payload size - 1] = payload
[10 + Payload size] = checksum
```

#### Src address
This is always zero for the controller. <!-- It may not if the parent hub messes with the src address when repeating packets --> It is `0xFF` for broadcast to all hubs.

#### Message number
- Message number should never equal zero. It starts at one. If it overflows to zero it should be reset to one <!-- TEST: message number overflowing -->
- Any response packets should use the same message number as their request

#### Reference number
- Indicates which packet the child device is responding to. Must be the same as the message number in any response packets
<!-- TEST: What happens to the reference number when the message number jumps around wildly -->
<!-- - On the stock firmware it goes up by one whenever a packet is received with a higher Message Number than the last packet -->

#### Command ID
- Indicates the purpose of the packet
- Bit 15 indicates if the packet is a response (1) or a command (0). 
   - A response is where the hub sends back data to the controller with the same command id except for bit 15. 
      - An example is the `GET_MODULE_STATUS` command. 
   - An `ACK` or` NACK` is a command not a response because it doesn't have the same command id as the initial command.

#### Payload
- 512 byte max size in librhsp

#### Checksum
- Last eight bits of the sum of all bytes of the packet except the checksum
- If the checksum is incorrect no response should be sent

#### Example Packet
This is an example of a keep alive packet.  
`44 4B 0B 00 AD 00 04 00 04 7F CE`

## Common Commands
All commands are assumed to be issued from the perspective of the controller. If there is no payload section listed then the command doesn't have a payload. If there is no mention of a response payload then the hub sends an ACK back as a response.

### ACK = 0x7f01
Used to signify that a command was successful  
Only ever a reply
#### Payload
```
[0] (bool) Attention Needed - I assume this is set when any of the status bits are set in the 'GET_MODULE_STATUS' command
```

### NACK = 0x7f02
Used to signify that a command was unsuccessful  
Only ever a reply
#### Payload
```
[0] NACK Code
```

NACK Codes:
```
// 0-9 are for params
0 = "Parameter #0 out of range"
1 = "Parameter #1 out of range"
2 = "Parameter #2 out of range"
3 = "Parameter #3 out of range"
4 = "Parameter #4 out of range"
5 = "Parameter #5 out of range"
6 = "Parameter #6 out of range"
7 = "Parameter #7 out of range"
8 = "Parameter #8 out of range"
9 = "Parameter #9 out of range"

// 10-19 are for GPIO output
10 = "GPIO #0 not configured for output"
11 = "GPIO #1 not configured for output"
12 = "GPIO #2 not configured for output"
13 = "GPIO #3 not configured for output"
14 = "GPIO #4 not configured for output"
15 = "GPIO #5 not configured for output"
16 = "GPIO #6 not configured for output"
17 = "GPIO #7 not configured for output"
18 = "No GPIO pins configured for output"
// NACK 19 is reserved

// 20-29 are for GPIO input
20 = "GPIO #0 not configured for input"
21 = "GPIO #1 not configured for input"
22 = "GPIO #2 not configured for input"
23 = "GPIO #3 not configured for input"
24 = "GPIO #4 not configured for input"
25 = "GPIO #5 not configured for input"
26 = "GPIO #6 not configured for input"
27 = "GPIO #7 not configured for input"
28 = "No GPIO pins configured for input"
// NACK 29 is reserved

// 30-39 are for servos
30 = "Servo not fully configured before enabled"
31 = "Battery voltage too low to run servo."
// NACKs 32-39 are reserved

// 40-49 are for i2c
40 = "I2C Master Busy (command rejected)"
41 = "I2C Operation in progress (poll again for completion status)"
42 = "I2C No results pending"
43 = "I2C Query mismatch (query doesn’t match last operation)"
44 = "I2C Timeout SDA stuck"
45 = "I2C Timeout SCK stuck"
46 = "I2C Timeout"
// NACKs 47-49 are reserved

// 50-59 are for motors
50 = "Motor not fully configured for selected mode before enabled"
51 = "Command not valid for selected motor mode"
52 = "Battery voltage too low to run motor."
// NACKs 53-59 are reserved

//There are a few codes at the end for debugging
253 = "Command implementation pending (software diagnostic; command is known and properly delivered but implementation is not complete)"
254 = "Command routing error (software diagnostic; command is known but not handled by receiving subsystem)"
255 = "Packet Type ID unknown (protocol failure; no unknown commands should be sent if discovery was performed properly)"
```

### GET_MODULE_STATUS = 0x7f03
Gets the status of a module
#### Payload 
```
[0] (bool) Clear status after response
```

#### Response Payload
For the Lynx at least, maybe not the Servo Hub. After device reset only the Device Reset bit should be set. The Battery Low bit is set when the battery is below 7 volts. It triggers a failsafe and also indicates on the LED.
```
[0] (uint8_t) Module Status
   [0] Keep Alive Timeout
   [1] Device Reset - device has come up from reset
   [2] Fail Safe - should be on when the battery level is too low to run, when there is a keep alive timeout 
   [3] Controller Over Temp
   [4] Battery Low
   [5] HIB Fault
   [6..7] Reserved 
[1] (uint8_t) Motor Status
   [0] Motor 0 lost counts 
   [1] Motor 1 lost counts
   [2] Motor 2 lost counts
   [3] Motor 3 lost counts
   [4] Motor 0 driver overheat
   [5] Motor 1 driver overheat
   [6] Motor 2 driver overheat
   [7] Motor 3 driver overheat
```

### KEEP_ALIVE = 0x7f04
Used to prevent the hub from timing out and going into fail safe mode

### FAIL_SAFE = 0x7f05
Tells the hub to disable all motion like an e-stop

### SET_NEW_MODULE_ADDRESS = 0x7f06
Tells a hub to change its address
#### Payload
```
[0] New address
```

### QUERY_INTERFACE = 0x7f07
Checks if a hub supports a specific interface
#### Payload
```
[0 .. strlen() + 1] Interface name as a null terminated string (hence the added one to strlen())
```

#### Response Payload
```
[0..1] (uint16_t) First Command ID. The ID number that the interface starts at.
[2..3] (uint16_t) Number of commands in interface
```

### START_DOWNLOAD = 0x7f08
### DOWNLOAD_CHUNK = 0x7f09
### SET_MODULE_LED_COLOR = 0x7f0a
Sets the color of the RGB LED on the Lynx

#### Payload
```
[0] Red value
[1] Green value
[2] Blue value
```

### GET_MODULE_LED_COLOR = 0x7f0b
Gets the color of the multicolor LED on the Lynx. It returns the currently showing color even if that is part of a pattern

#### Response Payload
```
[0] Red value
[1] Green value
[2] Blue value
```

### SET_MODULE_LED_PATTERN = 0x7f0c
Sends a list of times and colors for the LED to follow. The packet may contain up to 16 steps. To terminate the packet before step 15 a step of all zeros is sent.

#### Step
```
[0] Tenths of seconds for this step
[1] Blue 
[2] Green
[3] Red
```

#### Payload
```
[0..3] Step 0
[4..7] Step 1
...
[60..63] Step 15
```

### GET_MODULE_LED_PATTERN = 0x7f0d
Gets the current pattern being displayed. May not transmit all 16 steps
<!-- TEST: Does this pattern include the low power or connected one? -->
<!-- TEST: Doesn't seem to work on stock firmware with librhsp. Check RC app -->

#### Response Payload
See `Step` from SET_MODULE_LED_PATTERN
```
[0..3] Step 0
[4..7] Step 1
...
[60..63] Step 15
```

### DEBUG_LOG_LEVEL = 0x7f0e
### DISCOVERY = 0x7f0f
This packet is sent by the controller when it wants to find all of the hubs that are connected. First the controller sends a discovery packet to the broadcast address. The parent hub then sends back its address by replying to the discovery packet, but changing the src address from the broadcast address to the one of the parent hub. The parent hub also then proceeds to send out a discovery packet to every possible child address (0-254, but not the address of the parent (verify) over RS485 and forwards their responses back to the controller.

No Payload

#### Response payload
```
[0] (bool) Packet from parent
```

Packet from parent indicates if the parent is the originator of the packet or if it was retransmitted after being received from RS485

## DEKA Interface Commands
All commands are offset from the command ID offset value provided by a QUERY_INTERFACE command. The offset is typically 0x1000 for this interface.
Commands 0x19 - 0x1E are unimplemented by the stock firmware and librhsp, but they are implemented by the RC app. Maybe they are for the servo hub? Commands 0x36 - 0x39 are unimplemented by the stock firmware and the RC app. They are only implemented by librhsp.

### GET_BULK_INPUT_DATA = 0x00
### SET_SINGLE_DIO_OUTPUT = 0x01
### SET_ALL_DIO_OUTPUTS = 0x02
### SET_DIO_DIRECTION = 0x03
### GET_DIO_DIRECTION = 0x04
### GET_SINGLE_DIO_INPUT = 0x05
### GET_ALL_DIO_INPUTS = 0x06
### GET_ADC = 0x07
Gets the current value of one of the ADC channels
#### Payload
```
[0] Channel to read
[1] (bool) rawMode
```

Available channels:
```
User 0 = 0x0
User 1 = 0x1
User 2 = 0x2
User 3 = 0x3
GPIO current = 0x4
I2C current = 0x5
Servo current = 0x6
Battery current = 0x7
Motor 0 current = 0x8
Motor 1 current = 0x9
Motor 2 current = 0xA
Motor 3 current = 0xB
Five volts = 0xC
Battery volts = 0xD
Controller temperature = 0xE
```

Rawmode indicates if the reply should be a raw ADC measurement or converted into units. The units are millivolts, milliamps, or deci-degC (0.1 degrees C) as appropriate

#### Response Payload
```
[0-1] (int16) ADC value
```

### SET_MOTOR_CHANNEL_MODE = 0x08
### GET_MOTOR_CHANNEL_MODE = 0x09
### SET_MOTOR_CHANNEL_ENABLE = 0x0A
Enables or disables a motor channel
#### Payload
```
[0] Motor Channel
[1] (bool) Enabled
```

### GET_MOTOR_CHANNEL_ENABLE = 0x0B
Get if a motor channel is enabled or disabled
#### Payload
```
[0] Motor Channel
```

#### Response Payload
```
[0] (bool) Enabled
```

### SET_MOTOR_CHANNEL_CURRENT_ALERT_LEVEL = 0x0C
### GET_MOTOR_CHANNEL_CURRENT_ALERT_LEVEL = 0x0D
### RESET_MOTOR_ENCODER = 0x0E
Resets the encoder position back to zero
#### Payload
```
[0] Motor Channel
```

### SET_MOTOR_CONSTANT_POWER = 0x0F
Sets the power of a motor. In some modes this must be positive.
#### Payload
```
[0] Motor Channel
[1..2] (int16_t) Power
```

### GET_MOTOR_CONSTANT_POWER = 0x10
Gets the power of a motor
#### Payload
```
[0] Motor Channel
```

#### Response Payload
```
[0..1] (int16_t) Power
```

### SET_MOTOR_TARGET_VELOCITY = 0x11
### GET_MOTOR_TARGET_VELOCITY = 0x12
### SET_MOTOR_TARGET_POSITION = 0x13
### GET_MOTOR_TARGET_POSITION = 0x14
### IS_MOTOR_AT_TARGET = 0x15
### GET_MOTOR_ENCODER_POSITION = 0x16
Gets the encoder position
#### Payload
```
[0] Motor Channel
```

### Response Payload
```
[0..3] (int32_t) counts
```

### SET_MOTOR_PID_CONTROL_LOOP_COEFFICIENTS = 0x17
### GET_MOTOR_PID_CONTROL_LOOP_COEFFICIENTS = 0x18
### SET_PWM_CONFIGURATION = 0x19
### GET_PWM_CONFIGURATION = 0x1A
### SET_PWM_PULSE_WIDTH = 0x1B
### GET_PWM_PULSE_WIDTH = 0x1C
### SET_PWM_ENABLE = 0x1D
### GET_PWM_ENABLE = 0x1E
### SET_SERVO_CONFIGURATION = 0x1F
### GET_SERVO_CONFIGURATION = 0x20
### SET_SERVO_PULSE_WIDTH = 0x21
### GET_SERVO_PULSE_WIDTH = 0x22
### SET_SERVO_ENABLE = 0x23
### GET_SERVO_ENABLE = 0x24
### I2C_WRITE_SINGLE_BYTE = 0x25
### I2C_WRITE_MULTIPLE_BYTES = 0x26
### I2C_READ_SINGLE_BYTE = 0x27
### I2C_READ_MULTIPLE_BYTES = 0x28
### I2C_READ_STATUS_QUERY = 0x29
### I2C_WRITE_STATUS_QUERY = 0x2A
### I2C_CONFIGURE_CHANNEL = 0x2B
### PHONE_CHARGE_CONTROL = 0x2C
Enable phone charging. This is no longer supported by any competition legal phones

#### Payload 
```
[0] (bool) Enable
```

### PHONE_CHARGE_QUERY = 0x2D
Check if phone charging is enabled.

#### Response Payload 
```
[0] (bool) Enabled
```

### INJECT_DATA_LOG_HINT = 0x2E
### I2C_CONFIGURE_QUERY = 0x2F
### READ_VERSION_STRING = 0x30
Gets the version of the hub

#### Response Payload
```
[0] String length
[1..String Length] The version string. Typically looks like "HW: 20, Maj: 1, Min: 8, Eng: 2". Should not be null terminated
   HW is the hardware revision. 20 means 2.0
   Maj is the major version (1.x.x) in a semantic version
   Min is the minor version (x.8.x) in a semantic version
   Eng is the patch version (x.x.2) in a semantic version
```


### FTDI_RESET_CONTROL = 0x31
### FTDI_RESET_QUERY = 0x32
### SET_MOTOR_PIDF_CONTROL_LOOP_COEFFICIENTS = 0x33
### I2C_WRITE_READ_MULTIPLE_BYTES = 0x34
### GET_MOTOR_PIDF_CONTROL_LOOP_COEFFICIENTS = 0x35
### I2C_TRANSACTION = 0x36
### I2C_QUERY_TRANSACTION = 0x37
### SET_BULK_OUTPUT_DATA = 0x38


### READ_VERSION = 0x39
Gets the version of a hub firmware in a binary format. Unsupported on the stock Lynx FW <!-- TEST: is it actually unsupported? -->

