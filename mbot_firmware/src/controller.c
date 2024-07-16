#include "controller.h"

int mbot_init_ctlr(mbot_ctlr_cfg_t ctlr_cfg){
    left_wheel_pid = rc_filter_empty();
    right_wheel_pid = rc_filter_empty();
    back_wheel_pid = rc_filter_empty();
    mbot_vx_pid = rc_filter_empty();
    mbot_vy_pid = rc_filter_empty();
    mbot_wz_pid = rc_filter_empty();
    rc_filter_pid(&left_wheel_pid, ctlr_cfg.left.kp, ctlr_cfg.left.ki, ctlr_cfg.left.kd, ctlr_cfg.left.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&left_wheel_pid, ctlr_cfg.right.kp, ctlr_cfg.right.ki, ctlr_cfg.right.kd, ctlr_cfg.right.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&left_wheel_pid, ctlr_cfg.back.kp, ctlr_cfg.back.ki, ctlr_cfg.back.kd, ctlr_cfg.back.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&left_wheel_pid, ctlr_cfg.vx.kp, ctlr_cfg.vx.ki, ctlr_cfg.vx.kd, ctlr_cfg.vx.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&left_wheel_pid, ctlr_cfg.vy.kp, ctlr_cfg.vy.ki, ctlr_cfg.vy.kd, ctlr_cfg.vy.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&left_wheel_pid, ctlr_cfg.wz.kp, ctlr_cfg.wz.ki, ctlr_cfg.wz.kd, ctlr_cfg.wz.Tf, MAIN_LOOP_PERIOD);
}

int mbot_motor_vel_ctlr(serial_mbot_motor_vel_t vel_cmd, serial_mbot_motor_vel_t vel, serial_mbot_motor_pwm_t &mbot_motor_pwm){
    float right_error = vel_cmd.velocity[0] - vel.velocity[0];
    float left_error = vel_cmd.velocity[1] - vel.velocity[1];
    float back_error = vel_cmd.velocity[2] - vel.velocity[2];
    float right_cmd = rc_filter_march(right_error);
    float left_cmd = rc_filter_march(left_error);
    float back_cmd = rc_filter_march(back_error);
}
