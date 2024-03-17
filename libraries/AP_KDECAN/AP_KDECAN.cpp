/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * AP_KDECAN.cpp
 *
 *      Author: Francisco Ferreira and Tom Pittenger
 */

#include "AP_KDECAN.h"

#if AP_KDECAN_ENABLED
#include <stdio.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>    // for MIN,MAX

extern const AP_HAL::HAL& hal;

#define AP_KDECAN_DEBUG 0

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_KDECAN::var_info[] = {

    // @Param: NPOLE
    // @DisplayName: Number of motor poles
    // @Description: Sets the number of motor poles to calculate the correct RPM value
    AP_GROUPINFO("NPOLE", 1, AP_KDECAN, _num_poles, DEFAULT_NUM_POLES),

    AP_GROUPEND
};

AP_KDECAN::AP_KDECAN()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_KDECAN must be singleton");
    }
#endif
    _singleton = this;
}

void AP_KDECAN::init()
{
    if (_driver != nullptr) {
        // only allow one instance
        return;
    }

    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        if (CANSensor::get_driver_type(i) == AP_CAN::Protocol::KDECAN) {
            _driver = new AP_KDECAN_Driver();
            return;
        }
    }
}

void AP_KDECAN::update()
{
    if (_driver == nullptr) {
        return;
    }
    _driver->update((uint8_t)_num_poles.get());
}

AP_KDECAN_Driver::AP_KDECAN_Driver() : CANSensor("KDECAN")
{
    register_driver(AP_CAN::Protocol::KDECAN);

    // start thread for receiving and sending CAN frames. Tests show we use about 640 bytes of stack
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_KDECAN_Driver::loop, void), "kdecan", 2048, AP_HAL::Scheduler::PRIORITY_CAN, 0);
}

// parse inbound frames
void AP_KDECAN_Driver::handle_frame(AP_HAL::CANFrame &frame)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "got frame with %i", frame.id && frame.MaskStdID);
}

void AP_KDECAN_Driver::update(const uint8_t num_poles)
{
}

void AP_KDECAN_Driver::loop()
{
}

bool AP_KDECAN_Driver::send_packet_uint16(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_ms, const uint16_t data)
{
    const uint16_t data_be16 = htobe16(data);
    return send_packet(address, dest_id, timeout_ms, (uint8_t*)&data_be16, 2);
}

bool AP_KDECAN_Driver::send_packet(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_ms, const uint8_t *data, const uint8_t data_len)
{
    // broadcast telemetry request frame
    const frame_id_t id {
        {
            .object_address = address,
            .destination_id = dest_id,
            .source_id = AUTOPILOT_NODE_ID,
            .priority = 0,
            .unused = 0
        }
    };

    AP_HAL::CANFrame frame = AP_HAL::CANFrame((id.value | AP_HAL::CANFrame::FlagEFF), data, data_len, false);

    const uint64_t timeout_us = uint64_t(timeout_ms) * 1000UL;
    return write_frame(frame, timeout_us);
}

// singleton instance
AP_KDECAN *AP_KDECAN::_singleton;

namespace AP {
AP_KDECAN *kdecan()
{
    return AP_KDECAN::get_singleton();
}
};

#endif // AP_KDECAN_ENABLED

