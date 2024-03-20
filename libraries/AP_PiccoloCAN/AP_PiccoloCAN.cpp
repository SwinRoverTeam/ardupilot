/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Oliver Walters / Currawong Engineering Pty Ltd
 */


#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#include "AP_PiccoloCAN.h"

#if HAL_PICCOLO_CAN_ENABLE

#include <AP_Param/AP_Param.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/AP_Common.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_CANManager/AP_CANManager.h>

#include <AP_EFI/AP_EFI_Currawong_ECU.h>

#include <stdio.h>

// Protocol files for the Velocity ESC
#include <AP_PiccoloCAN/piccolo_protocol/ESCVelocityProtocol.h>
#include <AP_PiccoloCAN/piccolo_protocol/ESCPackets.h>

// Protocol files for the CBS servo
#include <AP_PiccoloCAN/piccolo_protocol/ServoProtocol.h>
#include <AP_PiccoloCAN/piccolo_protocol/ServoPackets.h>

extern const AP_HAL::HAL& hal;

#if HAL_CANMANAGER_ENABLED
#define debug_can(level_debug, fmt, args...) do { AP::can().log_text(level_debug, "PiccoloCAN", fmt, ##args); } while (0)
#else
#define debug_can(level_debug, fmt, args...)
#endif

// table of user-configurable Piccolo CAN bus parameters
const AP_Param::GroupInfo AP_PiccoloCAN::var_info[] = {

    // @Param: ESC_BM
    // @DisplayName: ESC channels
    // @Description: Bitmask defining which ESC (motor) channels are to be transmitted over Piccolo CAN
    // @Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32
    // @User: Advanced
    AP_GROUPINFO("ESC_BM", 1, AP_PiccoloCAN, _esc_bm, 0xFFFF),

    // @Param: ESC_RT
    // @DisplayName: ESC output rate
    // @Description: Output rate of ESC command messages
    // @Units: Hz
    // @User: Advanced
    // @Range: 1 500
    AP_GROUPINFO("ESC_RT", 2, AP_PiccoloCAN, _esc_hz, PICCOLO_MSG_RATE_HZ_DEFAULT),

    // @Param: SRV_BM
    // @DisplayName: Servo channels
    // @Description: Bitmask defining which servo channels are to be transmitted over Piccolo CAN
    // @Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15, 15: Servo 16
    // @User: Advanced
    AP_GROUPINFO("SRV_BM", 3, AP_PiccoloCAN, _srv_bm, 0xFFFF),

    // @Param: SRV_RT
    // @DisplayName: Servo command output rate
    // @Description: Output rate of servo command messages
    // @Units: Hz
    // @User: Advanced
    // @Range: 1 500
    AP_GROUPINFO("SRV_RT", 4, AP_PiccoloCAN, _srv_hz, PICCOLO_MSG_RATE_HZ_DEFAULT),
#if AP_EFI_CURRAWONG_ECU_ENABLED
    // @Param: ECU_ID
    // @DisplayName: ECU Node ID
    // @Description: Node ID to send ECU throttle messages to. Set to zero to disable ECU throttle messages. Set to 255 to broadcast to all ECUs.
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("ECU_ID", 5, AP_PiccoloCAN, _ecu_id, PICCOLO_CAN_ECU_ID_DEFAULT),

    // @Param: ECU_RT
    // @DisplayName: ECU command output rate
    // @Description: Output rate of ECU command messages
    // @Units: Hz
    // @User: Advanced
    // @Range: 1 500
    AP_GROUPINFO("ECU_RT", 6, AP_PiccoloCAN, _ecu_hz, PICCOLO_MSG_RATE_HZ_DEFAULT),
#endif
    AP_GROUPEND
};

AP_PiccoloCAN::AP_PiccoloCAN()
{
    AP_Param::setup_object_defaults(this, var_info);

    debug_can(AP_CANManager::LOG_INFO, "PiccoloCAN: constructed\n\r");
}

AP_PiccoloCAN *AP_PiccoloCAN::get_pcan(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CAN::Protocol::PiccoloCAN) {
        return nullptr;
    }

    return static_cast<AP_PiccoloCAN*>(AP::can().get_driver(driver_index));
}

bool AP_PiccoloCAN::add_interface(AP_HAL::CANIface* can_iface) {
    if (_can_iface != nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: Multiple Interface not supported\n\r");
        return false;
    }

    _can_iface = can_iface;

    if (_can_iface == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: CAN driver not found\n\r");
        return false;
    }

    if (!_can_iface->is_initialized()) {
        debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: Driver not initialized\n\r");
        return false;
    }

    if (!_can_iface->set_event_handle(&sem_handle)) {
        debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: Cannot add event handle\n\r");
        return false;
    }
    return true;
}

// initialize PiccoloCAN bus
void AP_PiccoloCAN::init(uint8_t driver_index, bool enable_filters)
{
    _driver_index = driver_index;

    debug_can(AP_CANManager::LOG_DEBUG, "PiccoloCAN: starting init\n\r");

    if (_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: already initialized\n\r");
        return;
    }
    // start calls to loop in separate thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_PiccoloCAN::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_MAIN, 1)) {
        debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: couldn't create thread\n\r");
        return;
    }

    _initialized = true;

    snprintf(_thread_name, sizeof(_thread_name), "PiccoloCAN_%u", driver_index);

    debug_can(AP_CANManager::LOG_DEBUG, "PiccoloCAN: init done\n\r");
}

// loop to send output to CAN devices in background thread
void AP_PiccoloCAN::loop()
{
    AP_HAL::CANFrame txFrame {};
    AP_HAL::CANFrame rxFrame {};

    uint16_t esc_tx_counter = 0;
    uint16_t servo_tx_counter = 0;
#if AP_EFI_CURRAWONG_ECU_ENABLED
    uint16_t ecu_tx_counter = 0;
#endif

    // CAN Frame ID components
    uint8_t frame_id_group;     // Piccolo message group
    uint16_t frame_id_device;   // Device identifier

    while (true) {

        if (!_initialized) {
            debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: not initialized\n\r");
            hal.scheduler->delay_microseconds(10000);
            continue;
        }

        // Calculate the output rate for ESC commands
        _esc_hz.set(constrain_int16(_esc_hz, PICCOLO_MSG_RATE_HZ_MIN, PICCOLO_MSG_RATE_HZ_MAX));

        uint16_t escCmdRateMs = 1000 / _esc_hz;

        // Calculate the output rate for servo commands
        _srv_hz.set(constrain_int16(_srv_hz, PICCOLO_MSG_RATE_HZ_MIN, PICCOLO_MSG_RATE_HZ_MAX));

        uint16_t servoCmdRateMs = 1000 / _srv_hz;
#if AP_EFI_CURRAWONG_ECU_ENABLED
        _ecu_hz.set(constrain_int16(_ecu_hz, PICCOLO_MSG_RATE_HZ_MIN, PICCOLO_MSG_RATE_HZ_MAX));

        uint16_t ecuCmdRateMs = 1000 / _ecu_hz;
#endif
        uint64_t timeout = AP_HAL::micros64() + 250ULL;

        // 1ms loop delay
        hal.scheduler->delay_microseconds(1000);

        // Transmit ESC commands at regular intervals
        if (esc_tx_counter++ > escCmdRateMs) {
            esc_tx_counter = 0;
            send_esc_messages();
        }

        // Transmit servo commands at regular intervals
        if (servo_tx_counter++ > servoCmdRateMs) {
            servo_tx_counter = 0;
            send_servo_messages();
        }

#if AP_EFI_CURRAWONG_ECU_ENABLED
        // Transmit ecu throttle commands at regular intervals
        if (ecu_tx_counter++ > ecuCmdRateMs) {
            ecu_tx_counter = 0;
            send_ecu_messages();
        }
#endif

        // Look for any message responses on the CAN bus
        while (read_frame(rxFrame, timeout)) {

            handle_esc_message(rxFrame);

            // Extract group and device ID values from the frame identifier
            frame_id_group = (rxFrame.id >> 24) & 0x1F;
            frame_id_device = (rxFrame.id >> 8) & 0xFF;

            // Only accept extended messages
            if ((rxFrame.id & AP_HAL::CANFrame::FlagEFF) == 0) {
                continue;
            }

            switch (PiccoloCAN_MessageGroup(frame_id_group)) {
            // ESC messages exist in the ACTUATOR group
            case PiccoloCAN_MessageGroup::ACTUATOR:

                switch (PiccoloCAN_ActuatorType(frame_id_device)) {
                case PiccoloCAN_ActuatorType::SERVO:
                    //handle_servo_message(rxFrame);
                    break;
                case PiccoloCAN_ActuatorType::ESC:
                    
                    break;
                default:
                    // Unknown actuator type
                    break;
                }

                break;
            case PiccoloCAN_MessageGroup::ECU_OUT:
            #if AP_EFI_CURRAWONG_ECU_ENABLED
                handle_ecu_message(rxFrame);
            #endif
                break;
            default:
                break;
            }
        }
    }
}

// write frame on CAN bus, returns true on success
bool AP_PiccoloCAN::write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout)
{
    if (!_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: Driver not initialized for write_frame\n\r");
        return false;
    }

    bool read_select = false;
    bool write_select = true;
    
    bool ret =  _can_iface->select(read_select, write_select, &out_frame, timeout);

    if (!ret || !write_select) {
        return false;
    }

    return (_can_iface->send(out_frame, timeout, AP_HAL::CANIface::AbortOnError) == 1);
}

// read frame on CAN bus, returns true on succses
bool AP_PiccoloCAN::read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout)
{
    if (!_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "PiccoloCAN: Driver not initialized for read_frame\n\r");
        return false;
    }
    bool read_select = true;
    bool write_select = false;
    bool ret = _can_iface->select(read_select, write_select, nullptr, timeout);

    if (!ret || !read_select) {
        // No frame available
        return false;
    }

    uint64_t time;
    AP_HAL::CANIface::CanIOFlags flags {};

    return (_can_iface->receive(recv_frame, time, flags) == 1);
}

// called from SRV_Channels
void AP_PiccoloCAN::update()
{
    uint64_t timestamp = AP_HAL::micros64();

    /* Read out the servo commands from the channel mixer */
    for (uint8_t ii = 0; ii < PICCOLO_CAN_MAX_NUM_SERVO; ii++) {

        if (is_servo_channel_active(ii)) {

            uint16_t output = 0;

            SRV_Channel::Aux_servo_function_t function = SRV_Channels::channel_function(ii);

            if (SRV_Channels::get_output_pwm(function, output)) {
                _servos[ii].command = output;
                _servos[ii].newCommand = true;
            }
        }
    }

    /* Read out the ESC commands from the channel mixer */
    for (uint8_t ii = 0; ii < PICCOLO_CAN_MAX_NUM_ESC; ii++) {

        if (is_esc_channel_active(ii)) {

            uint16_t output = 0;
            
            SRV_Channel::Aux_servo_function_t motor_function = SRV_Channels::get_motor_function(ii);

            if (SRV_Channels::get_output_pwm(motor_function, output)) {
                _escs[ii].command = output;
                _escs[ii].newCommand = true;
            }
        }
    }

#if AP_EFI_CURRAWONG_ECU_ENABLED
    if (_ecu_id != 0) {
        _ecu_info.command = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
        _ecu_info.newCommand = true;
    }
#endif // AP_EFI_CURRAWONG_ECU_ENABLED

    AP_Logger *logger = AP_Logger::get_singleton();

    // Push received telemetry data into the logging system
    if (logger && logger->logging_enabled()) {

        WITH_SEMAPHORE(_telem_sem);

        for (uint8_t ii = 0; ii < PICCOLO_CAN_MAX_NUM_SERVO; ii++) {

            AP_PiccoloCAN_Servo &servo = _servos[ii];

            if (servo.newTelemetry) {
                union {
                    Servo_ErrorBits_t ebits;
                    uint8_t errors;
                } err;
                err.ebits = servo.status.statusA.errors;
                logger->Write_ServoStatus(
                    timestamp,
                    ii,
                    servo.position(),                       // Servo position (represented in microsecond units)
                    servo.current() * 0.01f,                // Servo force (actually servo current, 0.01A per bit)
                    servo.speed(),                          // Servo speed (degrees per second)
                    servo.dutyCycle(),                      // Servo duty cycle (absolute value as it can be +/- 100%)
                    uint16_t(servo.commandedPosition()),    // Commanded position
                    servo.voltage(),                        // Servo voltage
                    servo.current(),                        // Servo current
                    servo.temperature(),                    // Servo temperature
                    servo.temperature(),                    // 
                    err.errors
                );

                servo.newTelemetry = false;
            }
        }
    }
}

#if HAL_GCS_ENABLED
// send ESC telemetry messages over MAVLink
void AP_PiccoloCAN::send_esc_telemetry_mavlink(uint8_t mav_chan)
{
    // Arrays to store ESC telemetry data
    uint8_t temperature[4] {};
    uint16_t voltage[4] {};
    uint16_t rpm[4] {};
    uint16_t count[4] {};
    uint16_t current[4] {};
    uint16_t totalcurrent[4] {};

    bool dataAvailable = false;

    uint8_t idx = 0;

    WITH_SEMAPHORE(_telem_sem);

    for (uint8_t ii = 0; ii < PICCOLO_CAN_MAX_NUM_ESC; ii++) {

        // Calculate index within storage array
        idx = (ii % 4);

        AP_PiccoloCAN_ESC &esc = _escs[idx];

        // Has the ESC been heard from recently?
        if (is_esc_present(ii)) {
            dataAvailable = true;

            // Provide the maximum ESC temperature in the telemetry stream
            temperature[idx] = esc.temperature();   // Convert to C
            voltage[idx] = esc.voltage() * 10;      // Convert to cV
            current[idx] = esc.current() * 10;      // Convert to cA
            totalcurrent[idx] = 0;
            rpm[idx] = esc.rpm();
            count[idx] = 0;
        } else {
            temperature[idx] = 0;
            voltage[idx] = 0;
            current[idx] = 0;
            totalcurrent[idx] = 0;
            rpm[idx] = 0;
            count[idx] = 0;
        }

        // Send ESC telemetry in groups of 4
        if ((ii % 4) == 3) {

            if (dataAvailable) {
                if (!HAVE_PAYLOAD_SPACE((mavlink_channel_t) mav_chan, ESC_TELEMETRY_1_TO_4)) {
                    continue;
                }

                switch (ii) {
                case 3:
                    mavlink_msg_esc_telemetry_1_to_4_send((mavlink_channel_t) mav_chan, temperature, voltage, current, totalcurrent, rpm, count);
                    break;
                case 7:
                    mavlink_msg_esc_telemetry_5_to_8_send((mavlink_channel_t) mav_chan, temperature, voltage, current, totalcurrent, rpm, count);
                    break;
                case 11:
                    mavlink_msg_esc_telemetry_9_to_12_send((mavlink_channel_t) mav_chan, temperature, voltage, current, totalcurrent, rpm, count);
                    break;
                case 15:
                    mavlink_msg_esc_telemetry_13_to_16_send((mavlink_channel_t) mav_chan, temperature, voltage, current, totalcurrent, rpm, count);
                    break;
                default:
                    break;
                }
            }

            dataAvailable = false;
        }
    }
}
#endif

// send servo messages over CAN
void AP_PiccoloCAN::send_servo_messages(void)
{
    AP_HAL::CANFrame txFrame {};

    uint64_t timeout = AP_HAL::micros64() + 1000ULL;

    // No servos are selected? Don't send anything!
    if (_srv_bm == 0x00) {
        return;
    }

    bool send_cmd = false;
    int16_t cmd[4] {};
    uint8_t idx;

    // Transmit bulk command packets to 4x servos simultaneously
    for (uint8_t ii = 0; ii < PICCOLO_CAN_MAX_GROUP_SERVO; ii++) {

        send_cmd = false;

        for (uint8_t jj = 0; jj < 4; jj++) {
            
            idx = (ii * 4) + jj;

            // Set default command value if an output field is unused
            cmd[jj] = 0x7FFF;

            // Skip servo if the output is not enabled
            if (!is_servo_channel_active(idx)) {
                continue;
            }

            /* Check if the servo is enabled.
             * If it is not enabled, send an enable message.
             */

            if (!is_servo_present(idx) || !is_servo_enabled(idx)) {
                // Servo is not enabled
                encodeServo_EnablePacket(&txFrame);
                txFrame.id |= (idx + 1);
                write_frame(txFrame, timeout);
            } else if (_servos[idx].newCommand) {
                // A new command is provided
                send_cmd = true;
                cmd[jj] = _servos[idx].command;
                _servos[idx].newCommand = false;
            }
        }

        if (send_cmd) {
            encodeServo_MultiPositionCommandPacket(
                &txFrame,
                cmd[0],
                cmd[1],
                cmd[2],
                cmd[3],
                (PKT_SERVO_MULTI_COMMAND_1 + ii)
            );

            // Broadcast the command to all servos
            txFrame.id |= 0xFF;

            write_frame(txFrame, timeout);
        }
    }
}

void AP_PiccoloCAN::process_mavlink(const mavlink_message_t &msg)
{
    // decode packet
    mavlink_command_long_t packet;
    mavlink_msg_command_long_decode(&msg, &packet);

    switch (packet.command)
    {
    case MAV_CMD_DO_SET_SERVO:
        {
        int code = 0xF6;
        int speed = 0x100;
        int acc = 0x3;
        int dir = 0;
        // read in arm positions and speed (degs)
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "1:%f 2:%f 3:%f", packet.param2, packet.param3, packet.param4);
        if ((int)packet.param2 == 1) {
            //Move arm forward
            dir = 0x0;
        }
        else if ((int)packet.param2 == -1) {
            //Move arm backwards
            dir = 0x80;
        } else {
            //Stop arm
            dir = 0x80;
            speed = 0x0;
        }

        uint64_t timeout = AP_HAL::micros64() + 10000ULL;

        // calculate crc for motor drivers
        int crc = (int)packet.param1 + code + dir + (speed >> 8) + (speed & 0xFF) + acc;
        crc = crc & 0xFF;

        // make message
        int buf[5];
        buf[0] = code;
        buf[1] =  dir + (speed >> 8);
        buf[2] = speed & 0xFF;
        buf[3] = acc;
        buf[4] = crc;

        AP_HAL::CANFrame frame = AP_HAL::CANFrame((uint32_t)packet.param1 + 0x301, (const uint8_t*)buf, 5, false);
        write_frame(frame, timeout);

        break;
        }

    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
        {
        // go to next cam
        if ((int)packet.param1 == 1) {
            current_cam++;
        } else {
            current_cam--;
        }

        break;
        }
    }
}

void AP_PiccoloCAN::send_esc_messages(void)
{
    uint64_t timeout = AP_HAL::micros64() + 10000ULL;

    // write motor drive commands
    uint8_t buf[8] = {0};
    buf[0] = (uint8_t)(hal.rcin->read(2) / 10) - 60;
    buf[2] = (uint8_t)(hal.rcin->read(1) / 10) - 60;

    AP_HAL::CANFrame frame = AP_HAL::CANFrame(0x002, buf, 8, false);
    write_frame(frame, timeout);
}

// interpret a servo message received over CAN
bool AP_PiccoloCAN::handle_servo_message(AP_HAL::CANFrame &frame)
{
    //Add code here
    // The servo address is the lower byte of the frame ID
    uint8_t addr = frame.id & 0xFF;

    // Ignore servo with an invalid node ID
    if (addr == 0x00) {
        return false;
    }

    // Subtract to get the address in memory
    addr -= 1;

    // Maximum number of servos allowed
    if (addr >= PICCOLO_CAN_MAX_NUM_SERVO) {
        return false;
    }

    // Pass the CAN frame off to the specific servo
    return _servos[addr].handle_can_frame(frame);
}

bool AP_PiccoloCAN::handle_esc_message(AP_HAL::CANFrame &frame)
{
    uint32_t std_id = frame.id & AP_HAL::CANFrame::MaskStdID;
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "got something from %lu", std_id);

    if (std_id != 0x307)
    {
        return false;
    }

    bool finalMsg = false;

    if (frame.data[0] < 128) {
        finalMsg = true;
    }

    frame.data[0] = frame.data[0] & 0x7F;

    int i = 0;
    for (; i < 8; i++) {
        if (frame.data[i] == 0x0) {
            break;
        }
        fullMsg[i + bytes_read] = frame.data[i];
    }
    bytes_read += i;

    if (finalMsg) {
        fullMsg[bytes_read] = 0;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s", fullMsg);
        bytes_read = 0;
    }

    return true;
}

#if AP_EFI_CURRAWONG_ECU_ENABLED
void AP_PiccoloCAN::send_ecu_messages(void)
{
    AP_HAL::CANFrame txFrame {};

    const uint64_t timeout = AP_HAL::micros64() + 1000ULL;

    // No ECU node id set, don't send anything
    if (_ecu_id == 0) {
        return;
    }

    if (_ecu_info.newCommand) {
        encodeECU_ThrottleCommandPacket(&txFrame, _ecu_info.command);
        txFrame.id |= (uint8_t) _ecu_id;

        _ecu_info.newCommand = false;

        write_frame(txFrame, timeout);
    }
}

bool AP_PiccoloCAN::handle_ecu_message(AP_HAL::CANFrame &frame)
{
    // Get the ecu instance
    AP_EFI_Currawong_ECU* ecu = AP_EFI_Currawong_ECU::get_instance();
    if (ecu != nullptr) {
        return ecu->handle_message(frame);
    }
    return false;
}
#endif // AP_EFI_CURRAWONG_ECU_ENABLED

/**
 * Check if a given servo channel is "active" (has been configured for Piccolo control output)
 */
bool AP_PiccoloCAN::is_servo_channel_active(uint8_t chan)
{
    // First check if the particular servo channel is enabled in the channel mask
    if (((_srv_bm >> chan) & 0x01) == 0x00) {
        return false;
    }

    SRV_Channel::Aux_servo_function_t function = SRV_Channels::channel_function(chan);

    // Ignore if the servo channel does not have a function assigned
    if (function <= SRV_Channel::k_none) {
        return false;
    }

    // Ignore if the assigned function is a motor function
    if (SRV_Channel::is_motor(function)) {
        return false;
    }

    // We can safely say that the particular servo channel is active
    return true;
}

/**
 * Check if a given ESC channel is "active" (has been configured for Piccolo control output)
 */
bool AP_PiccoloCAN::is_esc_channel_active(uint8_t chan)
{
    // First check if the particular ESC channel is enabled in the channel mask
    if (((_esc_bm >> chan) & 0x01) == 0x00) {
        return false;
    }

    // Check if a motor function is assigned for this motor channel
    SRV_Channel::Aux_servo_function_t motor_function = SRV_Channels::get_motor_function(chan);

    if (SRV_Channels::function_assigned(motor_function)) {
        return true;
    }

    return false;
}


/**
 * Determine if a servo is present on the CAN bus (has telemetry data been received)
 */
bool AP_PiccoloCAN::is_servo_present(uint8_t chan, uint64_t timeout_ms)
{
    if (chan >= PICCOLO_CAN_MAX_NUM_SERVO) {
        return false;
    }

    return _servos[chan].is_connected(timeout_ms);
}


/**
 * Determine if an ESC is present on the CAN bus (has telemetry data been received)
 */
bool AP_PiccoloCAN::is_esc_present(uint8_t chan, uint64_t timeout_ms)
{
    if (chan >= PICCOLO_CAN_MAX_NUM_ESC) {
        return false;
    }

    return _escs[chan].is_connected(timeout_ms);
}


/**
 * Check if a given servo is enabled
 */
bool AP_PiccoloCAN::is_servo_enabled(uint8_t chan)
{
    if (chan >= PICCOLO_CAN_MAX_NUM_SERVO) {
        return false;
    }

    // If the servo is not present, we cannot determine if it is enabled or not
    if (!is_servo_present(chan)) {
        return false;
    }

    return _servos[chan].is_enabled();
}


/**
 * Check if a given ESC is enabled (both hardware and software enable flags)
 */
bool AP_PiccoloCAN::is_esc_enabled(uint8_t chan)
{
    if (chan >= PICCOLO_CAN_MAX_NUM_ESC) {
        return false;
    }

    // If the ESC is not present, we cannot determine if it is enabled or not
    if (!is_esc_present(chan)) {
        return false;
    }

    return _escs[chan].is_enabled();
}


bool AP_PiccoloCAN::pre_arm_check(char* reason, uint8_t reason_len)
{
    // Check that each required servo is present on the bus
    for (uint8_t ii = 0; ii < PICCOLO_CAN_MAX_NUM_SERVO; ii++) {

        if (is_servo_channel_active(ii)) {

            if (!is_servo_present(ii)) {
                snprintf(reason, reason_len, "Servo %u not detected", ii + 1);
                return false;
            }
        }
    }

    // Check that each required ESC is present on the bus
    for (uint8_t ii = 0; ii < PICCOLO_CAN_MAX_NUM_ESC; ii++) {

        // Skip any ESC channels where the motor channel is not enabled
        if (is_esc_channel_active(ii)) {

            if (!is_esc_present(ii)) {
                snprintf(reason, reason_len, "ESC %u not detected", ii + 1);
                return false;
            }

            if (_escs[ii].is_hw_inhibited()) {
                snprintf(reason, reason_len, "ESC %u is hardware inhibited", (ii + 1));
                return false;
            }
        }
    }

    return true;
}


#endif // HAL_PICCOLO_CAN_ENABLE
