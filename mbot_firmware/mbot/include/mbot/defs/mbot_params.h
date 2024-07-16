#ifndef MBOT_PARAM_DEFS_H
#define MBOT_PARAM_DEFS_H

#define MBOT_ERROR -1
#define MBOT_OK 0
#define COMMS_ERROR 0
#define COMMS_OK 1
#define MBOT_TIMEOUT_US 1000000


#define SYS_CLOCK       125000 //system clock in kHz
#define PWM_FREQ        10000
#define MAIN_LOOP_HZ            25.0 // Hz of control loop
#define MAIN_LOOP_PERIOD        (1.0f / MAIN_LOOP_HZ)


// Hardware Parameters
#define GEAR_RATIO              78.0
#define ENCODER_RES             40.0 // 40.0 for ROB103 encoders

#define DIFFERENTIAL_DRIVE 1
#define OMNI_120_DRIVE 2 // 3 omni wheels spaced 120deg
#define ACKERMAN_DRIVE 3

// MBot Classic Parameters
#define DIFF_WHEEL_DIAMETER          0.0837
#define DIFF_WHEEL_RADIUS            0.04183
#define DIFF_BASE_RADIUS             0.07786
#define DIFF_MOTOR_LEFT_SLOT         0    // Left motor using M0 slot
#define DIFF_MOTOR_RIGHT_SLOT        1    // Right motor using M1 slot
#define UNUSED_DIFF_MOTOR_SLOT       2    // defined for mbot classic, 2 means M2 slot

// MBot Omni Parameters
// #define OMNI_OLD_BASE_RADIUS        0.094     // radius of wheel centers (old bot)
#define OMNI_BASE_RADIUS        0.10843     // Radius of base, from center of base to middle of omni wheels
                                            // Base radius to outer surface on wheel is 0.1227
#define OMNI_WHEEL_RADIUS       0.048       // 0.050 for old wheels
//#define OMNI_MOTOR_ANGLE_LFT (-M_PI / 3.0f)   // Left wheel rotation axis (-60 degrees)
//#define OMNI_MOTOR_ANGLE_BCK (M_PI)           // Back wheel rotation axis (180 degrees)
//#define OMNI_MOTOR_ANGLE_RGT (M_PI / 3.0f)    // Right wheel rotation axis (60 degrees)
#define OMNI_MOTOR_ANGLE_LFT (-M_PI / 6.0f)   // Left wheel velocity angle (-30 degrees)
#define OMNI_MOTOR_ANGLE_BCK (M_PI / 2.0f)           // Back wheel velocity angle (90 degrees)
#define OMNI_MOTOR_ANGLE_RGT (-5.0 * M_PI / 6.0f)    // Right wheel velocity angle (-150 degrees)
#define INV_SQRT3               5.7735026918962575E-1
#define SQRT3                   1.732050807568877

typedef struct mbot_params_t{
    int robot_type;
    float wheel_radius;
    float wheel_base_radius;
    float gear_ratio;
    float encoder_resolution;
    int mot_left;
    int mot_right;
    int mot_back;
    int motor_polarity[3];
    int encoder_polarity[3];
    float slope_pos[3];
    float itrcpt_pos[3];
    float slope_neg[3];
    float itrcpt_neg[3];
} mbot_params_t;



#endif