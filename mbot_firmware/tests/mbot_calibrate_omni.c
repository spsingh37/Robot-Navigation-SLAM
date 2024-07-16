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

void print_mbot_params_omni(const mbot_params_t* params) {
    printf("Robot Type: %d\n", params->robot_type);
    printf("Wheel Radius: %f\n", params->wheel_radius);
    printf("Wheel Base: %f\n", params->wheel_base_radius);
    printf("Gear Ratio: %f\n", params->gear_ratio);
    printf("Encoder Resolution: %f\n", params->encoder_resolution);
    printf("Motor Left: %d\n", params->mot_left);
    printf("Motor Right: %d\n", params->mot_right);
    printf("Motor Back: %d\n", params->mot_back);
    printf("Motor Polarity: %d %d %d\n", params->motor_polarity[0], params->motor_polarity[1], params->motor_polarity[2]);
    printf("Encoder Polarity: %d %d %d\n", params->encoder_polarity[0], params->encoder_polarity[1], params->encoder_polarity[2]);
    printf("Positive Slope: %f %f %f\n", params->slope_pos[0], params->slope_pos[1], params->slope_pos[2]);
    printf("Positive Intercept: %f %f %f\n", params->itrcpt_pos[0], params->itrcpt_pos[1], params->itrcpt_pos[2]);
    printf("Negative Slope: %f %f %f\n", params->slope_neg[0], params->slope_neg[1], params->slope_neg[2]);
    printf("Negative Intercept: %f %f %f\n", params->itrcpt_neg[0], params->itrcpt_neg[1], params->itrcpt_neg[2]);
}

int main() {
    mbot_params_t params;
    params.robot_type = OMNI_120_DRIVE;
    params.gear_ratio = GEAR_RATIO;
    params.encoder_resolution = ENCODER_RES;
    params.wheel_base_radius = OMNI_BASE_RADIUS;
    params.wheel_radius = OMNI_WHEEL_RADIUS;
    stdio_init_all();
    sleep_ms(5000); // quick sleep so we can catch the bootup process in terminal
    printf("\n\n\nInitializing...\n");
    bi_decl(bi_program_description("This will calibrate an MBot and print a diagnostic report"));
    mbot_motor_init(0);
    mbot_motor_init(1);
    mbot_motor_init(2);
    mbot_encoder_init();
    mbot_init_fram();
    printf("\nWaiting for 3 seconds...\n");

    //Since the IMU starts giving us data after ~2 seconds empirically, might as well initialize it now.
    mbot_imu_config = mbot_imu_default_config();
    int imu_init_status = mbot_imu_init(&mbot_imu_data, mbot_imu_config);
    sleep_ms(3000);

    
    /*************************************************
     * find encoder polarity relative to positive motor PWM
     *************************************************/
    printf("\nTesting Encoder Polarity...\n");
    mbot_motor_set_duty(0, 0.3);
    mbot_motor_set_duty(1, 0.3);
    mbot_motor_set_duty(2, 0.3);
    for(int i=0; i<5; i++){
        printf("E0: %d , E1: %d, E2: %d\n", mbot_encoder_read_delta(0), mbot_encoder_read_delta(1), mbot_encoder_read_delta(2));
        sleep_ms(100);
    }
    mbot_motor_set_duty(0, 0.0);
    mbot_motor_set_duty(1, 0.0);
    mbot_motor_set_duty(2, 0.0);
    params.encoder_polarity[0] = (mbot_encoder_read_count(0)>0) ? 1 : -1;
    params.encoder_polarity[1] = (mbot_encoder_read_count(1)>0) ? 1 : -1;
    params.encoder_polarity[2] = (mbot_encoder_read_count(2)>0) ? 1 : -1;
    
    //move back to start
    mbot_motor_set_duty(0, -0.3);
    mbot_motor_set_duty(1, -0.3);
    mbot_motor_set_duty(2, -0.3);
    sleep_ms(500);
    mbot_motor_set_duty(0, 0.0);
    mbot_motor_set_duty(1, 0.0);
    mbot_motor_set_duty(2, 0.0);
    sleep_ms(500);

    /*************************************************
     * find motor polarity relative to IMU, this is the
     * control signal to turn the robot CCW
     *************************************************/
    printf("\nTesting Motor Polarity...\n");
    if(imu_init_status == 0){
        // IMU available
        //  find motor polarity
        float gyro_z[3] = {0, 0, 0};
        // Definitions for motor duties for each case
        float motor_duties[3][3] = {
            {0.4, 0.0, 0.0},
            {0.0, 0.4, 0.0},
            {0.0, 0.0, 0.4},
        };
        for (int i = 0; i < 3; i++)
        {
            mbot_motor_set_duty(0, motor_duties[i][0]);
            mbot_motor_set_duty(1, motor_duties[i][1]);
            mbot_motor_set_duty(2, motor_duties[i][2]);
            // measure gyro to determine polarity of motor
            for (int j = 0; j < 25; j++)
            {
                gyro_z[i] += mbot_imu_data.gyro[2];
                sleep_ms(20);
            }
            // Stop
            mbot_motor_set_duty(0, 0.0);
            mbot_motor_set_duty(1, 0.0);
            mbot_motor_set_duty(2, 0.0);
            sleep_ms(100);

            mbot_motor_set_duty(0, -motor_duties[i][0]);
            mbot_motor_set_duty(1, -motor_duties[i][1]);
            mbot_motor_set_duty(2, -motor_duties[i][2]);
            for (int j = 0; j < 25; j++)
            {
                gyro_z[i] += -mbot_imu_data.gyro[2];
                sleep_ms(20);
            }

            mbot_motor_set_duty(0, 0.0);
            mbot_motor_set_duty(1, 0.0);
            mbot_motor_set_duty(2, 0.0);
            printf("Gyro: %ff\n", gyro_z[i]);
            sleep_ms(500);
        }

        for (int i = 0; i < 3; i++)
        {
            if (gyro_z[i] > 0.0)
            {
                params.motor_polarity[i] = -1;
            }
            else
            {
                params.motor_polarity[i] = 1;
            }
            params.encoder_polarity[i] *= params.motor_polarity[i];
        }
    }else{
        //Failed to init IMU - do manual motor calibration
        printf("\nNo IMU available - doing interactive motor calibration\n");
        printf("Press any character to start calibration.\n");
        printf("Pay attention to what direction the wheel spun!\n\n");
        getchar();

        for(int motor = 0; motor < 3; ++motor){
            printf("Running motor %d...\n", motor);
            while(1){
                mbot_motor_set_duty(motor, 0.4);
                sleep_ms(1000);
                mbot_motor_set_duty(motor, 0);
                printf("Which direction did the wheel spin?\n");
                printf("Enter 0 for Clockwise, 1 for CounterClockwise, or r for Retry.\n");
                char input = getchar();
                printf("Entered: [%c]\n", input);
                while(input != '0' && input != '1' && input != 'r'){
                    printf("Unrecognized command [%c], please input '0', '1', or 'r'.\n", input);
                    input = getchar();
                }
                if(input == '0'){
                    params.motor_polarity[motor] = -1;
                    break;
                }else if(input == '1'){
                    params.motor_polarity[motor] = 1;
                    break;
                }
                //Else, retry the process
            }
            params.encoder_polarity[motor] *= params.motor_polarity[motor];
        }
    }

    printf("\nEncoder Polarity: (%d, %d, %d)\n", params.encoder_polarity[0], params.encoder_polarity[1], params.encoder_polarity[2]);
    printf("Motor Polarity: (%d, %d, %d)\n", params.motor_polarity[0], params.motor_polarity[1], params.motor_polarity[2]);


    /*************************************************
     * find motor positions relative to IMU
     *************************************************/
    // Now we know the polarities, we can drive pairs of motors 
    // and measure the acceleration to find the placement
    // float motor_duties[3][3] = {
    //     {params.motor_polarity[0] * 0.5, params.motor_polarity[1] * -0.5, 0.0},
    //     {0.0, params.motor_polarity[1] * 0.5, params.motor_polarity[2] * -0.5},
    //     {params.motor_polarity[0] * 0.5, 0.0, params.motor_polarity[2] * -0.5},
    // };
    // float accel_x0 = 0.0;
    // float accel_y0 = 0.0;
    // float accel_x[3] = {0, 0, 0};
    // float accel_y[3] = {0, 0, 0};
    // sleep_ms(500);
    // for(int i=0; i<100; i++){
    //     accel_x0 += mbot_imu_data.accel[0]/100.0;
    //     accel_y0 += mbot_imu_data.accel[1]/100.0;
    //     sleep_ms(20);
    // }
    // printf("\nACCEL | x0: %f y0: %f\n\n", accel_x0, accel_y0);

    // for (int i = 0; i < 3; i++) {
    //     mbot_motor_set_duty(0, motor_duties[i][0]);
    //     mbot_motor_set_duty(1, motor_duties[i][1]);
    //     mbot_motor_set_duty(2, motor_duties[i][2]);

    //     // measure gyro to determine polarity of motor
    //     for (int j = 0; j < 25; j++) {
    //         accel_x[i] += mbot_imu_data.accel[0];
    //         accel_y[i] += mbot_imu_data.accel[1];
    //         sleep_ms(20);
    //     }
    //     mbot_motor_set_duty(0, 0.0);
    //     mbot_motor_set_duty(1, 0.0);
    //     mbot_motor_set_duty(2, 0.0);
    //     printf("Accel: X: %f Y: %f\n", accel_x[i], accel_y[i]);
    //     sleep_ms(500);
    // }

    params.mot_right = 0;
    params.mot_left = 2;
    params.mot_back = 1;
    


    /*************************************************
     * Test Driving FWD, BCK, LEFT, RIGHT, CCW, CW
     *************************************************/
    int enc_right;
    int enc_left;
    int enc_back;
    int mot_right = params.mot_right;
    int mot_left = params.mot_left;
    int mot_back = params.mot_back;
    printf("Driving Forward...\n");
    mbot_encoder_read_delta(mot_right);
    mbot_encoder_read_delta(mot_left);
    mbot_encoder_read_delta(mot_back);
    mbot_motor_set_duty(mot_right, params.motor_polarity[mot_right]*-0.3);
    mbot_motor_set_duty(mot_left, params.motor_polarity[mot_left]*0.3);
    mbot_motor_set_duty(mot_back, params.motor_polarity[mot_back]*0.0);
    sleep_ms(500);
    mbot_motor_set_duty(mot_right, 0.0);
    mbot_motor_set_duty(mot_left, 0.0);
    mbot_motor_set_duty(mot_back, 0.0);
    enc_right = params.encoder_polarity[mot_right] * mbot_encoder_read_delta(mot_right);
    enc_left = params.encoder_polarity[mot_left] * mbot_encoder_read_delta(mot_left);
    enc_back = params.encoder_polarity[mot_back] * mbot_encoder_read_delta(mot_back);
    printf("Encoder Readings: R: %d, L: %d, B: %d\n", enc_right, enc_left, enc_back);
    sleep_ms(500);

    printf("Driving Backward...\n");
    mbot_encoder_read_delta(mot_right);
    mbot_encoder_read_delta(mot_left);
    mbot_encoder_read_delta(mot_back);
    mbot_motor_set_duty(mot_right, params.motor_polarity[mot_right]*0.3);
    mbot_motor_set_duty(mot_left, params.motor_polarity[mot_left]*-0.3);
    mbot_motor_set_duty(mot_back, params.motor_polarity[mot_back]*0.0);
    sleep_ms(500);
    mbot_motor_set_duty(mot_right, 0.0);
    mbot_motor_set_duty(mot_left, 0.0);
    mbot_motor_set_duty(mot_back, 0.0);
    enc_right = params.encoder_polarity[mot_right] * mbot_encoder_read_delta(mot_right);
    enc_left = params.encoder_polarity[mot_left] * mbot_encoder_read_delta(mot_left);
    enc_back = params.encoder_polarity[mot_back] * mbot_encoder_read_delta(mot_back);
    printf("Encoder Readings: R: %d, L: %d, B: %d\n", enc_right, enc_left, enc_back);
    sleep_ms(500);

    printf("Driving Right...\n");
    mbot_encoder_read_delta(mot_right);
    mbot_encoder_read_delta(mot_left);
    mbot_encoder_read_delta(mot_back);
    mbot_motor_set_duty(mot_right, params.motor_polarity[mot_right]*0.2);
    mbot_motor_set_duty(mot_left, params.motor_polarity[mot_left]*0.2);
    mbot_motor_set_duty(mot_back, params.motor_polarity[mot_back]*-0.4);
    sleep_ms(500);
    mbot_motor_set_duty(mot_right, 0.0);
    mbot_motor_set_duty(mot_left, 0.0);
    mbot_motor_set_duty(mot_back, 0.0);
    enc_right = params.encoder_polarity[mot_right] * mbot_encoder_read_delta(mot_right);
    enc_left = params.encoder_polarity[mot_left] * mbot_encoder_read_delta(mot_left);
    enc_back = params.encoder_polarity[mot_back] * mbot_encoder_read_delta(mot_back);
    printf("Encoder Readings: R: %d, L: %d, B: %d\n", enc_right, enc_left, enc_back);
    sleep_ms(500);

    printf("Driving Left...\n");
    mbot_encoder_read_delta(mot_right);
    mbot_encoder_read_delta(mot_left);
    mbot_encoder_read_delta(mot_back);
    mbot_motor_set_duty(mot_right, params.motor_polarity[mot_right]*-0.2);
    mbot_motor_set_duty(mot_left, params.motor_polarity[mot_left]*-0.2);
    mbot_motor_set_duty(mot_back, params.motor_polarity[mot_back]*0.4);
    sleep_ms(500);
    mbot_motor_set_duty(mot_right, 0.0);
    mbot_motor_set_duty(mot_left, 0.0);
    mbot_motor_set_duty(mot_back, 0.0);
    enc_right = params.encoder_polarity[mot_right] * mbot_encoder_read_delta(mot_right);
    enc_left = params.encoder_polarity[mot_left] * mbot_encoder_read_delta(mot_left);
    enc_back = params.encoder_polarity[mot_back] * mbot_encoder_read_delta(mot_back);
    printf("Encoder Readings: R: %d, L: %d, B: %d\n", enc_right, enc_left, enc_back);
    sleep_ms(500);

   printf("Turning CCW...\n");
    mbot_encoder_read_delta(mot_right);
    mbot_encoder_read_delta(mot_left);
    mbot_encoder_read_delta(mot_back);
    mbot_motor_set_duty(mot_right, params.motor_polarity[mot_right]*-0.3);
    mbot_motor_set_duty(mot_left, params.motor_polarity[mot_left]*-0.3);
    mbot_motor_set_duty(mot_back, params.motor_polarity[mot_back]*-0.3);
    sleep_ms(500);
    mbot_motor_set_duty(mot_right, 0.0);
    mbot_motor_set_duty(mot_left, 0.0);
    mbot_motor_set_duty(mot_back, 0.0);
    enc_right = params.encoder_polarity[mot_right] * mbot_encoder_read_delta(mot_right);
    enc_left = params.encoder_polarity[mot_left] * mbot_encoder_read_delta(mot_left);
    enc_back = params.encoder_polarity[mot_back] * mbot_encoder_read_delta(mot_back);
    printf("Encoder Readings: R: %d, L: %d, B: %d\n", enc_right, enc_left, enc_back);
    sleep_ms(500);

   printf("Turning CW...\n");
    mbot_encoder_read_delta(mot_right);
    mbot_encoder_read_delta(mot_left);
    mbot_encoder_read_delta(mot_back);
    mbot_motor_set_duty(mot_right, params.motor_polarity[mot_right]*0.3);
    mbot_motor_set_duty(mot_left, params.motor_polarity[mot_left]*0.3);
    mbot_motor_set_duty(mot_back, params.motor_polarity[mot_back]*0.3);
    sleep_ms(500);
    mbot_motor_set_duty(mot_right, 0.0);
    mbot_motor_set_duty(mot_left, 0.0);
    mbot_motor_set_duty(mot_back, 0.0);
    enc_right = params.encoder_polarity[mot_right] * mbot_encoder_read_delta(mot_right);
    enc_left = params.encoder_polarity[mot_left] * mbot_encoder_read_delta(mot_left);
    enc_back = params.encoder_polarity[mot_back] * mbot_encoder_read_delta(mot_back);
    printf("Encoder Readings: R: %d, L: %d, B: %d\n", enc_right, enc_left, enc_back);
    sleep_ms(500);


    /*************************************************
     * Measure Calibration Params by turning in place
     *************************************************/

    printf("Measuring Motor Calibration...\n");
    // Turn CCW
    int num_points = 20;
    float dt = 0.5;
    float wheel_speed_right[num_points+1];
    float wheel_speed_left[num_points+1];
    float wheel_speed_back[num_points+1];
    float duty_right[num_points+1];
    float duty_left[num_points+1];
    float duty_back[num_points+1];
    float conv = (2 * M_PI)/(params.gear_ratio * params.encoder_resolution);
    printf("Measuring CCW...\n");
    mbot_encoder_read_delta(mot_right);
    mbot_encoder_read_delta(mot_left);
    mbot_encoder_read_delta(mot_back);
    
    for(int i = 0; i <= num_points; i++){
        
        float d = i * 1.0/(float)num_points;
        mbot_motor_set_duty(mot_right, params.motor_polarity[mot_right] * -d);
        mbot_motor_set_duty(mot_left,  params.motor_polarity[mot_left]  * -d);
        mbot_motor_set_duty(mot_back,  params.motor_polarity[mot_back]  * -d);
        sleep_ms(dt * 1000);
        duty_right[i] = -d;
        duty_left[i] = -d;
        duty_back[i] = -d;
        wheel_speed_right[i] = conv * mbot_encoder_read_delta(mot_right) / dt;
        wheel_speed_left[i] = conv * mbot_encoder_read_delta(mot_left) / dt;
        wheel_speed_back[i] = conv * mbot_encoder_read_delta(mot_back) / dt;
        printf("duty: %f, right: %f, left: %f, back: %f\n", duty_right[i], wheel_speed_right[i], wheel_speed_left[i], wheel_speed_back[i]);
    }
    
    int n = sizeof(duty_right) / sizeof(duty_right[0]);
    float m_rn, b_rn, m_ln, b_ln, m_bn, b_bn;
    least_squares_fit(duty_right, wheel_speed_right, n, &m_rn, &b_rn);
    least_squares_fit(duty_left, wheel_speed_left, n, &m_ln, &b_ln);
    least_squares_fit(duty_back, wheel_speed_back, n, &m_bn, &b_bn);
    
    //slow down
    
    mbot_motor_set_duty(mot_right, params.motor_polarity[mot_right] * -0.8);
    mbot_motor_set_duty(mot_left, params.motor_polarity[mot_left] * -0.8);
    mbot_motor_set_duty(mot_back, params.motor_polarity[mot_back] * -0.8);
    sleep_ms(300);
    mbot_motor_set_duty(mot_right, params.motor_polarity[mot_right] * -0.4);
    mbot_motor_set_duty(mot_left, params.motor_polarity[mot_left] * -0.4);
    mbot_motor_set_duty(mot_back, params.motor_polarity[mot_back] * -0.4);
    sleep_ms(300);
    mbot_motor_set_duty(mot_right, 0.0);
    mbot_motor_set_duty(mot_left, 0.0);
    mbot_motor_set_duty(mot_back, 0.0);
    printf("\n\n");
    

    //Turn CW
    sleep_ms(500);
    printf("Measuring CW...\n");
    mbot_encoder_read_delta(mot_right);
    mbot_encoder_read_delta(mot_left);
    mbot_encoder_read_delta(mot_back);
    
    for(int i = 0; i <= num_points; i++){
        
        float d = i * 1.0/(float)num_points;
        mbot_motor_set_duty(mot_right, params.motor_polarity[mot_right] * d);
        mbot_motor_set_duty(mot_left, params.motor_polarity[mot_left] * d);
        mbot_motor_set_duty(mot_back, params.motor_polarity[mot_back] * d);
        sleep_ms(dt * 1000);
        duty_right[i] = d;
        duty_left[i] = d;
        duty_back[i] = d;
        wheel_speed_right[i] = conv * mbot_encoder_read_delta(mot_right) / dt;
        wheel_speed_left[i] = conv * mbot_encoder_read_delta(mot_left) / dt;
        wheel_speed_back[i] = conv * mbot_encoder_read_delta(mot_back) / dt;
        printf("duty: %f, right: %f, left: %f, back: %f\n", duty_right[i], wheel_speed_right[i], wheel_speed_left[i], wheel_speed_back[i]);
    }

     //slow down
    
    mbot_motor_set_duty(mot_right, params.motor_polarity[mot_right] * 0.8);
    mbot_motor_set_duty(mot_left, params.motor_polarity[mot_left] * 0.8);
    mbot_motor_set_duty(mot_back, params.motor_polarity[mot_back] * 0.8);
    sleep_ms(300);
    mbot_motor_set_duty(mot_right, params.motor_polarity[mot_right] * 0.4);
    mbot_motor_set_duty(mot_left, params.motor_polarity[mot_left] * 0.4);
    mbot_motor_set_duty(mot_back, params.motor_polarity[mot_back] * 0.4);
    sleep_ms(300);
    mbot_motor_set_duty(mot_right, 0.0);
    mbot_motor_set_duty(mot_left, 0.0);
    mbot_motor_set_duty(mot_back, 0.0);
    printf("\n\n");

    float m_rp, b_rp, m_lp, b_lp, m_bp, b_bp;
    least_squares_fit(duty_right, wheel_speed_right, n, &m_rp, &b_rp);
    least_squares_fit(duty_left, wheel_speed_left, n, &m_lp, &b_lp);
    least_squares_fit(duty_back, wheel_speed_back, n, &m_bp, &b_bp);
    
    params.slope_pos[mot_right] = m_rp;
    params.slope_pos[mot_left] = m_lp;
    params.slope_pos[mot_back] = m_bp;
    params.slope_neg[mot_right] = m_rn;
    params.slope_neg[mot_left] = m_ln;
    params.slope_neg[mot_back] = m_bn;
    params.itrcpt_pos[mot_right] = b_rp;
    params.itrcpt_pos[mot_left] = b_lp;
    params.itrcpt_pos[mot_back] = b_bp;
    params.itrcpt_neg[mot_right] = b_rn;
    params.itrcpt_neg[mot_left] = b_ln;
    params.itrcpt_neg[mot_back] = b_bn;

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
    printf("Back Motor Calibration: \n");
    printf("m_bp: %f\n", m_bp);
    printf("b_bp: %f\n", b_bp);
    printf("m_bn: %f\n", m_bn);
    printf("b_bn: %f\n", b_bn);

    printf("Writing to FRAM...\n");
    mbot_write_fram(0, sizeof(params), &params);

    printf("Reading from written FRAM...\n");
    mbot_params_t written;
    mbot_read_fram(0, sizeof(written), &written);

    printf("\nParameters stored in FRAM (%d bytes): \n", sizeof(written));
    print_mbot_params_omni(&written);

    printf("\nDone!\n");
}