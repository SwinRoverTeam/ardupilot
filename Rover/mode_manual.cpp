#include "Rover.h"

void ModeManual::_exit()
{
    // clear lateral when exiting manual mode
    g2.motors.set_lateral(0);
}

void ModeManual::update()
{
    float desired_steering, desired_throttle, desired_lateral,
        desired_roll, desired_pitch;

    // grab channel positions
    get_pilot_desired_roll_and_pitch(desired_roll, desired_pitch);
    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
    get_pilot_desired_lateral(desired_lateral);

    uint8_t buf[8] = {0};
    buf[0] = (uint8_t)desired_throttle;
    buf[1] = (uint8_t)desired_pitch;
    

    // create CAN message
    static AP_HAL::CANFrame frame = AP_HAL::CANFrame(1, buf, 8, false);
    hal.can[0]->send(frame, 1, 1);

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "thrtl: %f, ptch: %f", desired_throttle, desired_pitch);

    /*
    // apply manual steering expo
    desired_steering = 4500.0 * input_expo(desired_steering / 4500, g2.manual_steering_expo);

    // if vehicle is balance bot, calculate actual throttle required for balancing
    if (rover.is_balancebot()) {
        rover.balancebot_pitch_control(desired_throttle);
    }

    // walking robots support roll, pitch and walking_height
    
    get_pilot_desired_walking_height(desired_walking_height);
    g2.motors.set_roll(desired_roll);
    g2.motors.set_pitch(desired_pitch);
    g2.motors.set_walking_height(desired_walking_height);

    // set sailboat sails
    float desired_mainsail;
    float desired_wingsail;
    float desired_mast_rotation;
    g2.sailboat.get_pilot_desired_mainsail(desired_mainsail, desired_wingsail, desired_mast_rotation);
    g2.motors.set_mainsail(desired_mainsail);
    g2.motors.set_wingsail(desired_wingsail);
    g2.motors.set_mast_rotation(desired_wingsail);

    // copy RC scaled inputs to outputs
    g2.motors.set_throttle(desired_throttle);
    g2.motors.set_steering(desired_steering, (g2.manual_options & ManualOptions::SPEED_SCALING));
    g2.motors.set_lateral(desired_lateral);
    */
}
