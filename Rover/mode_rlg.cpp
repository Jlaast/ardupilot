#include "Rover.h"

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif

void setup();
void loop();


/* setup UART at 115200 ---------------------------------------- */
static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == NULL)
    {
        // that UART doesn't exist on this platform
        return;
    }
    ///begin(baudrate,Rx,Tx)
    uart->begin(115200);
}

void setup(void)
{
/* Setup UartC ------------- */
   hal.scheduler->delay(1000); //Ensure that the uartA can be initialized
   setup_uart(hal.serial(1), "SERIAL1");
}
   char c = 0;
   static char buf[16];
   static unsigned int i;

void ModeRLG::update()
{

    while(hal.serial(1)->available() > 0 && c != '\n' && i < sizeof(buf)){
      c = hal.serial(1)->read();
      buf[i++] = c;
      //printf("Hello on UART %s at %.3f seconds\n",name, (double)(AP_HAL::millis() * 0.001f));
      gcs().send_text(MAV_SEVERITY_CRITICAL, "UART1 %c ",  c);
   }


    // get speed forward
    float speed, desired_steering;
    if (!attitude_control.get_forward_speed(speed)) {
        float desired_throttle;
        // convert pilot stick input into desired steering and throttle
        get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);

        // if vehicle is balance bot, calculate actual throttle required for balancing
        if (rover.is_balancebot()) {
            rover.balancebot_pitch_control(desired_throttle);
        }

        // no valid speed, just use the provided throttle
        g2.motors.set_throttle(desired_throttle);
    } else {
        float desired_speed;
        // convert pilot stick input into desired steering and speed
        get_pilot_desired_steering_and_speed(desired_steering, desired_speed);
        calc_throttle(desired_speed, true);
    }

    float steering_out;

    // handle sailboats
    if (!is_zero(desired_steering)) {
        // steering input return control to user
        rover.g2.sailboat.clear_tack();
    }
    if (rover.g2.sailboat.tacking()) {
        // call heading controller during tacking

        steering_out = attitude_control.get_steering_out_heading(rover.g2.sailboat.get_tack_heading_rad(),
                                                                 g2.wp_nav.get_pivot_rate(),
                                                                 g2.motors.limit.steer_left,
                                                                 g2.motors.limit.steer_right,
                                                                 rover.G_Dt);
    } else {
        // convert pilot steering input to desired turn rate in radians/sec
        const float target_turn_rate = (desired_steering / 4500.0f) * radians(g2.acro_turn_rate);

        // run steering turn rate controller and throttle controller
        steering_out = attitude_control.get_steering_out_rate(target_turn_rate,
                                                              g2.motors.limit.steer_left,
                                                              g2.motors.limit.steer_right,
                                                              rover.G_Dt);
    }

    set_steering(steering_out * 4500.0f);
}

bool ModeRLG::requires_velocity() const
{
    return !g2.motors.have_skid_steering();
}

// sailboats in acro mode support user manually initiating tacking from transmitter
void ModeRLG::handle_tack_request()
{
    //rover.g2.sailboat.handle_tack_request_acro();
}
