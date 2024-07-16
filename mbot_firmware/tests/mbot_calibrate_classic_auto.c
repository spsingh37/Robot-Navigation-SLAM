#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <mbot/imu/imu.h>
#include <mbot/motor/motor.h>
#include <mbot/encoder/encoder.h>
#include <mbot/fram/fram.h>

mbot_bhy_data_t mbot_imu_data;
mbot_bhy_config_t mbot_imu_config;

#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

void find_two_smallest(float* arr, int size, int* idx1, int* idx2) {
    *idx1 = 0;
    *idx2 = 1;

    if (fabs(arr[*idx2]) < fabs(arr[*idx1])) {
        // swap
        int temp = *idx1;
        *idx1 = *idx2;
        *idx2 = temp;
    }

    for (int i = 2; i < size; i++) {
        if (fabs(arr[i]) < fabs(arr[*idx1])) {
            *idx2 = *idx1;
            *idx1 = i;
        } else if (fabs(arr[i]) < fabs(arr[*idx2])) {
            *idx2 = i;
        }
    }
}

int find_index_of_max_positive(float* arr, int size) {
    float max_positive = -5000.0;
    int idx = -1;
    for (int i = 0; i < size; i++) {
        if (arr[i] > max_positive && arr[i] > 0) {
            max_positive = arr[i];
            idx = i;
        }
    }
    // Return -1 if no positive number is found
    return idx;
}

void least_squares_fit(float* pwms, float* speeds, int n, float* m, float* b) {
    float sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_xx = 0.0;

    for (int i = 0; i < n; ++i) {
        if (speeds[i] != 0.0) {
            sum_x += speeds[i];
            sum_y += pwms[i];
            sum_xy += speeds[i] * pwms[i];
            sum_xx += speeds[i] * speeds[i];
        }
    }

    *m = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
    *b = (sum_y - *m * sum_x) / n;
}

void print_mbot_params_dd(const mbot_params_t* params) {
    printf("Robot Type: %d\n", params->robot_type);
    printf("Wheel Radius: %f\n", params->wheel_radius);
    printf("Wheel Base Radius: %f\n", params->wheel_base_radius);
    printf("Gear Ratio: %f\n", params->gear_ratio);
    printf("Encoder Resolution: %f\n", params->encoder_resolution);
    printf("Motor Left: %d\n", params->mot_left);
    printf("Motor Right: %d\n", params->mot_right);

    printf("Motor Polarity: %d %d\n", params->motor_polarity[MOTOR_LEFT], params->motor_polarity[MOTOR_RIGHT]);
    printf("Encoder Polarity: %d %d\n", params->encoder_polarity[MOTOR_LEFT], params->encoder_polarity[MOTOR_RIGHT]);

    printf("Positive Slope: %f %f\n", params->slope_pos[MOTOR_LEFT], params->slope_pos[MOTOR_RIGHT]);
    printf("Positive Intercept: %f %f\n", params->itrcpt_pos[MOTOR_LEFT], params->itrcpt_pos[MOTOR_RIGHT]);

    printf("Negative Slope: %f %f\n", params->slope_neg[MOTOR_LEFT], params->slope_neg[MOTOR_RIGHT]);
    printf("Negative Intercept: %f %f\n", params->itrcpt_neg[MOTOR_LEFT], params->itrcpt_neg[MOTOR_RIGHT]);
}

int main() {
    mbot_params_t params;
    params.robot_type = DIFFERENTIAL_DRIVE;
    params.gear_ratio = GEAR_RATIO;
    params.encoder_resolution = ENCODER_RES;
    params.wheel_base_radius = DIFF_BASE_RADIUS;
    params.wheel_radius = DIFF_WHEEL_RADIUS;
    stdio_init_all();
    printf("\n\n\nInitializing...\n");
    bi_decl(bi_program_description("This will calibrate an MBot and print a diagnostic report"));
    mbot_motor_init(MOTOR_LEFT);
    mbot_motor_init(MOTOR_RIGHT);
    mbot_encoder_init();
    mbot_init_fram();
    printf("\nWaiting for 5 seconds...\n");
    sleep_ms(5000);
    // find encoder polarity
    printf("\nTesting Encoder Polarity...\n");
    mbot_motor_set_duty(MOTOR_LEFT, 0.2);
    mbot_motor_set_duty(MOTOR_RIGHT, 0.2);
    for(int i=0; i<5; i++){
        printf("E0: %d , E1: %d\n", mbot_encoder_read_delta(MOTOR_LEFT), mbot_encoder_read_delta(MOTOR_RIGHT));
        sleep_ms(100);
    }
    mbot_motor_set_duty(MOTOR_LEFT, 0.0);
    mbot_motor_set_duty(MOTOR_RIGHT, 0.0);
    params.encoder_polarity[MOTOR_LEFT] = (mbot_encoder_read_count(MOTOR_LEFT)>0) ? 1 : -1;
    params.encoder_polarity[MOTOR_RIGHT] = (mbot_encoder_read_count(MOTOR_RIGHT)>0) ? 1 : -1;
    printf("\nENC0 POL: %d , ENC1 POL: %d\n", params.encoder_polarity[MOTOR_LEFT], params.encoder_polarity[MOTOR_RIGHT]);
    
    
    printf("\nTesting Motor Polarity...\n");
    mbot_imu_config = mbot_imu_default_config();
    mbot_imu_init(&mbot_imu_data, mbot_imu_config);
    float accel_x0 = 0.0;
    float accel_y0 = 0.0;
    sleep_ms(1000);
    for(int i=0; i<100; i++){
        accel_x0 += mbot_imu_data.accel[0]/100.0;
        accel_y0 += mbot_imu_data.accel[1]/100.0;
        sleep_ms(20);
    }
    printf("\nTesting Motor Polarity...\n");
    printf("\nACCEL baseline | x0: %f y0: %f\n\n", accel_x0, accel_y0);


    // find motor polarity
    float gyro_z[4] = {0, 0, 0, 0};
    float accel_x[4] = {0, 0, 0, 0};
    float accel_y[4] = {0, 0, 0, 0};

    float spd = 0.4; //Motor duty used for calibration
    float motor_duties[4][2] = {
        {spd, spd},
        {-spd, spd},
        {spd, -spd},
        {-spd, -spd}
    };

    for (int i = 0; i < 4; i++) {
        mbot_motor_set_duty(MOTOR_LEFT, motor_duties[i][0]);
        mbot_motor_set_duty(MOTOR_RIGHT, motor_duties[i][1]);

        for (int j = 0; j < 25; j++) {
            gyro_z[i] += mbot_imu_data.gyro[2];
            accel_x[i] += mbot_imu_data.accel[0] - accel_x0;
            accel_y[i] += mbot_imu_data.accel[1] - accel_y0;
            sleep_ms(20);
        }

        mbot_motor_set_duty(MOTOR_LEFT, 0.0);
        mbot_motor_set_duty(MOTOR_RIGHT, 0.0);
        printf("Gyro: %f , Accel X: %f , Accel Y: %f\n", gyro_z[i], accel_x[i], accel_y[i]);
        sleep_ms(500);
    }
    int idx1, idx2, fwd_idx, rot_idx;
    find_two_smallest(gyro_z, 4, &idx1, &idx2); //Find the two segments without rotation
    printf("2 smallest: (%d, %d)\n", idx1, idx2);
    rot_idx = find_index_of_max_positive(gyro_z, 4); //Find largest positive rotation (+wz)

    if(accel_x[idx1] < 0.0){fwd_idx = idx1;}
    else if(accel_x[idx2] < 0.0){fwd_idx = idx2;} //Find no rotation max -x acceleration (+vx)
    else{printf("ERROR, No Negative Acceleration\n");}
    printf("rotidx, fwdidx = %d, %d\n", rot_idx, fwd_idx);
    switch(fwd_idx) {
    case 0:
        params.motor_polarity[MOTOR_LEFT] = 1;
        params.motor_polarity[MOTOR_RIGHT] = -1;
        break;
    case 1:
        params.motor_polarity[MOTOR_LEFT] = -1;
        params.motor_polarity[MOTOR_RIGHT] = -1;
        break;
    case 2:
        params.motor_polarity[MOTOR_LEFT] = 1;
        params.motor_polarity[MOTOR_RIGHT] = 1;
        break;
    case 3:
        params.motor_polarity[MOTOR_LEFT] = -1;
        params.motor_polarity[MOTOR_RIGHT] = 1;
        break;
    default:
        printf("ERROR: Invalid index\n");
    }
    //Adjust Polarity for positive readings
    params.encoder_polarity[MOTOR_LEFT] *= params.motor_polarity[MOTOR_LEFT];
    params.encoder_polarity[MOTOR_RIGHT] *= params.motor_polarity[MOTOR_RIGHT];

    for(int i=0; i<4; i++){
        motor_duties[i][0] *= params.motor_polarity[MOTOR_LEFT];
        motor_duties[i][1] *= params.motor_polarity[MOTOR_RIGHT];
    }
    // if(motor_duties[rot_idx][0] > 0.0){
    //     params.mot_right = 0;
    //     params.mot_left = 2;
    // }
    // else if(motor_duties[rot_idx][1] > 0.0){
    //     params.mot_right = 2;
    //     params.mot_left = 0;
    // }
    params.mot_left = MOTOR_LEFT;
    params.mot_right = MOTOR_RIGHT;
    printf("Motor Polarity: (%d, %d)  Left ID: %d, Right ID: %d\n", params.motor_polarity[MOTOR_LEFT], params.motor_polarity[MOTOR_RIGHT], params.mot_left, params.mot_right);

    int enc_right;
    int enc_left;
    int mot_right = params.mot_right;
    int mot_left = params.mot_left;

    printf("Driving Forward...\n");
    mbot_encoder_read_delta(mot_right);
    mbot_encoder_read_delta(mot_left);
    mbot_motor_set_duty(mot_right, params.motor_polarity[mot_right]*-0.2);
    mbot_motor_set_duty(mot_left, params.motor_polarity[mot_left]*0.2);
    sleep_ms(500);
    mbot_motor_set_duty(mot_right, 0.0);
    mbot_motor_set_duty(mot_left, 0.0);
    enc_right = params.encoder_polarity[mot_right] * mbot_encoder_read_delta(mot_right);
    enc_left = params.encoder_polarity[mot_left] * mbot_encoder_read_delta(mot_left);
    printf("Encoder Readings: R: %d, L: %d, \n", enc_right, enc_left);
    sleep_ms(500);

    printf("Driving Backward...\n");
    mbot_encoder_read_delta(mot_right);
    mbot_encoder_read_delta(mot_left);
    mbot_motor_set_duty(mot_right, params.motor_polarity[mot_right]*0.2);
    mbot_motor_set_duty(mot_left, params.motor_polarity[mot_left]*-0.2);
    sleep_ms(500);
    mbot_motor_set_duty(mot_right, 0.0);
    mbot_motor_set_duty(mot_left, 0.0);
    enc_right = params.encoder_polarity[mot_right] * mbot_encoder_read_delta(mot_right);
    enc_left = params.encoder_polarity[mot_left] * mbot_encoder_read_delta(mot_left);
    printf("Encoder Readings: R: %d, L: %d, \n", enc_right, enc_left);
    sleep_ms(500);

    printf("Turning Positive (CCW)...\n");
    mbot_encoder_read_delta(mot_right);
    mbot_encoder_read_delta(mot_left);
    mbot_motor_set_duty(mot_right, params.motor_polarity[mot_right]*-0.2);
    mbot_motor_set_duty(mot_left, params.motor_polarity[mot_left]*-0.2);
    sleep_ms(500);
    mbot_motor_set_duty(mot_right, 0.0);
    mbot_motor_set_duty(mot_left, 0.0);
    enc_right = params.encoder_polarity[mot_right] * mbot_encoder_read_delta(mot_right);
    enc_left = params.encoder_polarity[mot_left] * mbot_encoder_read_delta(mot_left);
    printf("Encoder Readings: R: %d, L: %d, \n", enc_right, enc_left);
    sleep_ms(500);

    printf("Turning Negative (CW)...\n");
    mbot_encoder_read_delta(mot_right);
    mbot_encoder_read_delta(mot_left);
    mbot_motor_set_duty(mot_right, params.motor_polarity[mot_right]*0.2);
    mbot_motor_set_duty(mot_left, params.motor_polarity[mot_left]*0.2);
    sleep_ms(500);
    mbot_motor_set_duty(mot_right, 0.0);
    mbot_motor_set_duty(mot_left, 0.0);
    enc_right = params.encoder_polarity[mot_right] * mbot_encoder_read_delta(mot_right);
    enc_left = params.encoder_polarity[mot_left] * mbot_encoder_read_delta(mot_left);
    printf("Encoder Readings: R: %d, L: %d, \n", enc_right, enc_left);
    sleep_ms(500);

    printf("Measuring Motor Calibration...\n");
    
    // Turn CCW
    int num_points = 20;
    float dt = 0.5;
    float wheel_speed_right[num_points+1];
    float wheel_speed_left[num_points+1];
    float duty_right[num_points+1];
    float duty_left[num_points+1];
    float conv = (2 * M_PI)/(params.gear_ratio * params.encoder_resolution);
    printf("Measuring CCW...\n");
    mbot_encoder_read_delta(mot_right);
    mbot_encoder_read_delta(mot_left);
    
    for(int i = 0; i <= num_points; i++){
        
        float d = i * 1.0/(float)num_points;
        mbot_motor_set_duty(mot_right, params.motor_polarity[mot_right] * -d);
        mbot_motor_set_duty(mot_left, params.motor_polarity[mot_left] * -d);
        sleep_ms(dt * 1000);
        duty_right[i] = -d;
        duty_left[i] = -d;
        wheel_speed_right[i] = conv * params.encoder_polarity[mot_right] * mbot_encoder_read_delta(mot_right) / dt;
        wheel_speed_left[i] = conv * params.encoder_polarity[mot_left] * mbot_encoder_read_delta(mot_left) / dt;
        printf("duty: %f, right: %f, left: %f\n", duty_right[i], wheel_speed_right[i], wheel_speed_left[i]);
    }
    
    int n = sizeof(duty_right) / sizeof(duty_right[0]);
    float m_rn, b_rn, m_ln, b_ln;
    least_squares_fit(duty_right, wheel_speed_right, n, &m_rn, &b_rn);
    least_squares_fit(duty_left, wheel_speed_left, n, &m_ln, &b_ln);
    
    //slow down

    mbot_motor_set_duty(mot_right, params.motor_polarity[mot_right] * -0.8);
    mbot_motor_set_duty(mot_left, params.motor_polarity[mot_left] * -0.8);
    sleep_ms(300);
    mbot_motor_set_duty(mot_right, params.motor_polarity[mot_right] * -0.5);
    mbot_motor_set_duty(mot_left, params.motor_polarity[mot_left] * -0.5);
    sleep_ms(300);
    mbot_motor_set_duty(mot_right, 0.0);
    mbot_motor_set_duty(mot_left, 0.0);
    printf("\n\n");
    

    //Turn CW
    sleep_ms(500);
    printf("Measuring CW...\n");
    mbot_encoder_read_delta(mot_right);
    mbot_encoder_read_delta(mot_left);
    for(int i = 0; i <= num_points; i++){
        float d = i * 1.0/(float)num_points;
        mbot_motor_set_duty(mot_right, params.motor_polarity[mot_right] * d);
        mbot_motor_set_duty(mot_left, params.motor_polarity[mot_left] * d);
        sleep_ms(dt * 1000);
        duty_right[i] = d;
        duty_left[i] = d;
        wheel_speed_right[i] = conv * params.encoder_polarity[mot_right] * mbot_encoder_read_delta(mot_right) / dt;
        wheel_speed_left[i] = conv * params.encoder_polarity[mot_left] * mbot_encoder_read_delta(mot_left) / dt;
        printf("duty: %f, right: %f, left: %f\n", duty_right[i], wheel_speed_right[i], wheel_speed_left[i]);
    }

    //slow down
    mbot_motor_set_duty(mot_right, params.motor_polarity[mot_right] * 0.8);
    mbot_motor_set_duty(mot_left, params.motor_polarity[mot_left] * 0.8);
    sleep_ms(300);
    mbot_motor_set_duty(mot_right, params.motor_polarity[mot_right] * 0.5);
    mbot_motor_set_duty(mot_left, params.motor_polarity[mot_left] * 0.5);
    sleep_ms(300);
    mbot_motor_set_duty(mot_right, 0.0);
    mbot_motor_set_duty(mot_left, 0.0);

    float m_rp, b_rp, m_lp, b_lp;
    least_squares_fit(duty_right, wheel_speed_right, n, &m_rp, &b_rp);
    least_squares_fit(duty_left, wheel_speed_left, n, &m_lp, &b_lp);
    
    params.slope_pos[mot_right] = m_rp;
    params.slope_pos[mot_left] = m_lp;
    params.slope_neg[mot_right] = m_rn;
    params.slope_neg[mot_left] = m_ln;
    params.itrcpt_pos[mot_right] = b_rp;
    params.itrcpt_pos[mot_left] = b_lp;
    params.itrcpt_neg[mot_right] = b_rn;
    params.itrcpt_neg[mot_left] = b_ln;

    printf("Right Motor Calibration: \n");
    printf("m_rp: %f\n", m_rp);
    printf("b_rp: %f\n", b_rp);
    printf("m_rn: %f\n", m_rn);
    printf("b_rn: %f\n", b_rn);
    printf("Left Motor Calibration: \n");
    printf("m_lp: %f\n", m_lp);
    printf("b_lp: %f\n", b_lp);
    printf("m_ln: %f\n", m_ln);
    printf("b_ln: %f\n", b_ln);

    
    mbot_write_fram(0, sizeof(params), &params);
    mbot_params_t written;
    mbot_read_fram(0, sizeof(written), &written);
    printf("\nParameters stored in FRAM (%d bytes): \n", sizeof(written));
    print_mbot_params_dd(&written);

    printf("\nDone!\n");
}