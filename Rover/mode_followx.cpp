#include "Rover.h"
#include <AP_Follow/AP_Follow.h>
#include <ctype.h>
#include <stdio.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>



// initialize follow mode
bool ModeFollowx::_enter()
{
  
    if (!g2.follow.enabled()) {
        return false;
    }

    // initialise speed to waypoint speed
    _desired_speed = g2.wp_nav.get_default_speed();

    return true;
    gcs().send_text(MAV_SEVERITY_CRITICAL,'Mode Follow X active')
}

// exit handling
void ModeFollowx::_exit()
{
    g2.follow.clear_offsets_if_required();
}

void ModeFollowx::update()
{
    
    // stop vehicle if no speed estimate
    float speed;
    if (!attitude_control.get_forward_speed(speed)) {
        // no valid speed so stop
        g2.motors.set_throttle(0.0f);
        g2.motors.set_steering(0.0f);
        return;
    }

    Vector3f dist_vec;  // vector to lead vehicle
    Vector3f dist_vec_offs; // vector to lead vehicle + offset
    Vector3f vel_of_target; // velocity of lead vehicle

    // if no target simply stop the vehicle
    if (!g2.follow.get_target_dist_and_vel_ned(dist_vec, dist_vec_offs, vel_of_target)) {  
        _reached_destination = true;
        stop_vehicle();
        return;
    }
    float distance = safe_sqrt(sq(dist_vec.x) + sq(dist_vec.y)); // compute distance wrt target
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Target distance is %.2f", (float) distance);

    // calculate desired velocity vector
    Vector2f desired_velocity_ne;
    const float kp = g2.follow.get_pos_p().kP();
    desired_velocity_ne.x = vel_of_target.x // desired speed is longitudinal target speed (x speed)
    
    // calculate target heading 
    float target_heading_deg;
    if (!g2.follow.get_target_heading_deg(target_heading_deg)) {
        return;
    }
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Target heading is %.2f", (float) target_heading_deg);


    // if desired  x velocity is zero stop vehicle
    if (is_zero(desired_velocity_ne.x) ) {
        _reached_destination = true;
        stop_vehicle();
        return;
    }

    // we have not reached the target
    _reached_destination = false;

    // scale desired velocity to stay within horizontal speed limit
    float desired_speed = safe_sqrt(sq(desired_velocity_ne.x));
    if (!is_zero(desired_speed) && (desired_speed > _desired_speed)) {
        const float scalar_xy = _desired_speed / desired_speed;
        desired_velocity_ne *= scalar_xy;
        desired_speed = _desired_speed;
    }
    
    // calculate vehicle heading
    const float desired_yaw_cd = wrap_180_cd(atan2f( desired_velocity_ne.x) * DEGX100);

    // run steering and throttle controllers
    calc_steering_to_heading(desired_yaw_cd);
    calc_throttle(desired_speed, true);
    
    void Write(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, ...);

    AP::logger().Write("DIST_TEST", "Target_distx,Target_disty,Target_distance,Target_heading",
                   "mmmd", // units: meters, meters,meters , degrees
                   "0000", // multiplier *1
                   "ffff", // format: float
                   AP_HAL::micros64(),
                   (double)dist_vec.x,
                   (double)dist_vec.y,
                   (double)distance,
                   (double)target_heading_deg);
}


// return desired heading (in degrees) for reporting to ground station (NAV_CONTROLLER_OUTPUT message)
float ModeFollowx::wp_bearing() const
{
    return g2.follow.get_bearing_to_target();
}

// return distance (in meters) to destination
float ModeFollowx::get_distance_to_destination() const
{
    return g2.follow.get_distance_to_target();
}

// set desired speed in m/s
bool ModeFollowx::set_desired_speed(float speed)
{
    if (is_negative(speed)) {
        return false;
    }
    _desired_speed = speed;
    return true;
}
