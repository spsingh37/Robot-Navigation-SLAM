#include <stdio.h>
#include <stdint.h>
#include <mbot/motor/motor.h>
#include <mbot/encoder/encoder.h>
#include <rc/math/filter.h>
#include <pico/stdlib.h>
#include <hardware/adc.h>
#include <mbot/fram/fram.h>
#include <mbot/defs/mbot_params.h>
#include <mbot_lcm_msgs_serial.h>

#define INT_16_MAX 32000
#define MAINLOOP_DELAY_MS 40

static mbot_params_t params;

serial_mbot_encoders_t mbot_encoders = {0};
serial_mbot_motor_vel_t mbot_motor_vel = {0};
serial_twist2D_t mbot_vel_cmd = {0};

static uint64_t global_utime = 0;

static rc_filter_t filter_L = RC_FILTER_INITIALIZER;
static rc_filter_t filter_R = RC_FILTER_INITIALIZER;

// Helper function to use slope + intercept from calibration to generate a PWM command.
float _calibrated_pwm_from_vel_cmd(float vel_cmd, int motor_idx)
{
    if (vel_cmd > 0.0)
    {
        return (vel_cmd * params.slope_pos[motor_idx]) + params.itrcpt_pos[motor_idx];
    }
    else if (vel_cmd < 0.0)
    {
        return (vel_cmd * params.slope_neg[motor_idx]) + params.itrcpt_neg[motor_idx];
    }
    return 0.0;
}

void mbot_read_encoders(serial_mbot_encoders_t *encoders)
{
    int64_t delta_time = global_utime - encoders->utime;
    encoders->utime = global_utime;
    encoders->delta_time = delta_time;

    encoders->ticks[params.mot_right] = mbot_encoder_read_count(params.mot_right);
    encoders->delta_ticks[params.mot_right] = mbot_encoder_read_delta(params.mot_right);
    encoders->ticks[params.mot_left] = mbot_encoder_read_count(params.mot_left);
    encoders->delta_ticks[params.mot_left] = mbot_encoder_read_delta(params.mot_left);
}

void mbot_calculate_motor_vel(serial_mbot_encoders_t encoders, serial_mbot_motor_vel_t *motor_vel)
{
    float conversion = (1.0 / params.gear_ratio) * (1.0 / params.encoder_resolution) * 1E6f * 2.0 * 3.14159;
    motor_vel->velocity[params.mot_left] = params.encoder_polarity[params.mot_left] * (conversion / encoders.delta_time) * encoders.delta_ticks[params.mot_left];
    motor_vel->velocity[params.mot_right] = params.encoder_polarity[params.mot_right] * (conversion / encoders.delta_time) * encoders.delta_ticks[params.mot_right];
}

int main(){
    //----------------------Initialization
    stdio_init_all();
    // set master clock to 250MHz (if unstable set SYS_CLOCK to 125Mhz)
    if (!set_sys_clock_khz(125000, true))
    {
        printf("ERROR mbot_init_pico: cannot set system clock\n");
        return MBOT_ERROR;
    };

    sleep_ms(2000); // quick sleep so we can catch the bootup process in terminal
    printf("Initializing!\n");

    mbot_motor_init(0);
    mbot_motor_init(2);
    mbot_encoder_init();

    if (mbot_init_fram())
    {
        printf("ERROR: FRAM chip failed to initialize\n");
        return -1;
    }
    //Get params from FRAM to use calibration
    mbot_read_fram(0, sizeof(params), &params);
    global_utime = to_us_since_boot(get_absolute_time());

    //Set tunable values here (kP, kD)
    float kP = 1.5, kI = 1.5, kD = 1E-9; //0.3 spd: 1.5,1.5,1E-9
    mbot_vel_cmd.vx = 0.2; //should not go beyond 0.4 (set 9.563)
    mbot_vel_cmd.vy = 0;
    mbot_vel_cmd.wz = 0;

    
    if(rc_filter_pid(&filter_L, kP, kI, kD, MAINLOOP_DELAY_MS / 250.0, MAINLOOP_DELAY_MS / 1000.0))
    {
        return -1; //Failed to create filter
    }
    if (rc_filter_pid(&filter_R, kP, kI, kD, MAINLOOP_DELAY_MS / 250.0, MAINLOOP_DELAY_MS / 1000.0))
    {
        return -1; // Failed to create filter
    }

    float filter_cutoff = 15.0; //rad/s, fastest the wheel could/should go
    // max speed during calibration: neg 9.1106/9.4489; pos 10.2867/8.7723 at 4 pips of power
    //                               neg 9.1589/9.4249; pos 10.2545/8.7884 at full battery

    rc_filter_enable_saturation(&filter_L, -filter_cutoff, filter_cutoff); //upper/lower bounding the filter output
    rc_filter_enable_saturation(&filter_R, -filter_cutoff, filter_cutoff);
    int loop_ctr = 0;
    sleep_ms(MAINLOOP_DELAY_MS); //Avoid sudden delta encoder change due to short time between encoder reads

    while(1){
        mbot_read_encoders(&mbot_encoders);
        mbot_calculate_motor_vel(mbot_encoders, &mbot_motor_vel);

        //PID controller
        float vL_goal = (mbot_vel_cmd.vx - DIFF_BASE_RADIUS * mbot_vel_cmd.wz) / DIFF_WHEEL_RADIUS;
        float vR_goal = (-mbot_vel_cmd.vx - DIFF_BASE_RADIUS * mbot_vel_cmd.wz) / DIFF_WHEEL_RADIUS;

        float vL = rc_filter_march(&filter_L, vL_goal - mbot_motor_vel.velocity[params.mot_left]);
        float vR = rc_filter_march(&filter_R, vR_goal - mbot_motor_vel.velocity[params.mot_right]);

        float pwmL = _calibrated_pwm_from_vel_cmd(vL * params.motor_polarity[params.mot_left], params.mot_left);
        float pwmR = _calibrated_pwm_from_vel_cmd(vR * params.motor_polarity[params.mot_right], params.mot_right);
        
        printf("Goal (rad/s): \t[%.3f, %.3f]\n", vL_goal, vR_goal);
        printf("Real (rad/s): \t[%.3f, %.3f]\n", mbot_motor_vel.velocity[params.mot_left], mbot_motor_vel.velocity[params.mot_right]);
        // printf("PID adjusted: \t[%.3f, %.3f]\n", vL, vR);
        // printf("------------------------------------\n\n");

        // printf("%.3f\n", vL);
        loop_ctr++;
        if(loop_ctr > 8){
            loop_ctr = 0;
            // printf("\n");
        }

        mbot_motor_set_duty(params.mot_left, pwmL);
        mbot_motor_set_duty(params.mot_right, pwmR);
        global_utime = to_us_since_boot(get_absolute_time());
        sleep_ms(MAINLOOP_DELAY_MS); //close enough to main loop freq.
    }
    rc_filter_free(&filter_L);
    rc_filter_free(&filter_R);
}