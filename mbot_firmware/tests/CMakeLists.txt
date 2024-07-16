include_directories(/usr/local/include)

#mbot_dmp_test
add_executable(mbot_dmp_test
  mbot_dmp_test.c
)
target_link_libraries(mbot_dmp_test
  pico_stdlib
  pico_multicore
  hardware_pwm
  hardware_sync
  hardware_i2c
  hardware_flash
  rclib
  comms
)
pico_enable_stdio_usb(mbot_dmp_test 1)
pico_enable_stdio_uart(mbot_dmp_test 0)
pico_add_extra_outputs(mbot_dmp_test)

#mbot_calibrate_accel
add_executable(mbot_calibrate_accel
  mbot_calibrate_accel.c
)
target_link_libraries(mbot_calibrate_accel
  pico_stdlib
  pico_multicore
  hardware_pwm
  hardware_sync
  hardware_i2c
  hardware_flash
  rclib
  comms
)
pico_enable_stdio_usb(mbot_calibrate_accel 1)
pico_enable_stdio_uart(mbot_calibrate_accel 0)
pico_add_extra_outputs(mbot_calibrate_accel)

#mbot_fake_calibration
add_executable(mbot_fake_calibration
  mbot_fake_calibration.c
)
target_link_libraries(mbot_fake_calibration
  pico_stdlib
  pico_multicore
  hardware_pwm
  hardware_sync
  hardware_i2c
  hardware_flash
  rclib
  comms
)
pico_enable_stdio_usb(mbot_fake_calibration 1)
pico_enable_stdio_uart(mbot_fake_calibration 0)
pico_add_extra_outputs(mbot_fake_calibration)


#mbot_calibrate_gyro
add_executable(mbot_calibrate_gyro
  mbot_calibrate_gyro.c
)
target_link_libraries(mbot_calibrate_gyro
  pico_stdlib
  pico_multicore
  hardware_pwm
  hardware_sync
  hardware_i2c
  hardware_flash
  rclib
  comms
)
pico_enable_stdio_usb(mbot_calibrate_gyro 1)
pico_enable_stdio_uart(mbot_calibrate_gyro 0)
pico_add_extra_outputs(mbot_calibrate_gyro)

#MATRIX-TESTS
add_executable(rc_matrix_test
rc_matrix_test.c
)
target_link_libraries(rc_matrix_test
  pico_stdlib
  rclib
)
pico_enable_stdio_usb(rc_matrix_test 1)
pico_add_extra_outputs(rc_matrix_test)


#FRAM-TEST
add_executable(mbot_fram_test
  mbot_fram_test.c
)
target_link_libraries(mbot_fram_test
  pico_stdlib
  hardware_i2c
  rclib
)
pico_enable_stdio_usb(mbot_fram_test 1)
pico_enable_stdio_uart(mbot_fram_test 0)
pico_add_extra_outputs(mbot_fram_test)

#BHY-TEST
add_executable(mbot_bhy_test
  mbot_bhy_test.c
)
target_link_libraries(mbot_bhy_test
  pico_stdlib
  hardware_i2c
  rclib
)
pico_enable_stdio_usb(mbot_bhy_test 1)
pico_enable_stdio_uart(mbot_bhy_test 0)
pico_add_extra_outputs(mbot_bhy_test)

#BARO-TEST
add_executable(mbot_barometer_test
  mbot_barometer_test.c
)
target_link_libraries(mbot_barometer_test
  mbotlib
  pico_stdlib
  hardware_i2c
  rclib
)
pico_enable_stdio_usb(mbot_barometer_test 1)
pico_enable_stdio_uart(mbot_barometer_test 0)
pico_add_extra_outputs(mbot_barometer_test)

#MOTOR-TEST
add_executable(mbot_motor_test
  mbot_motor_test.c
)
target_link_libraries(mbot_motor_test
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  rclib
)
pico_enable_stdio_usb(mbot_motor_test 1)
pico_enable_stdio_uart(mbot_motor_test 0)
pico_add_extra_outputs(mbot_motor_test)

#SERVO-TEST
add_executable(mbot_servo_test
  mbot_servo_test.c
)
target_link_libraries(mbot_servo_test
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  rclib
)
pico_enable_stdio_usb(mbot_servo_test 1)
pico_enable_stdio_uart(mbot_servo_test 0)
pico_add_extra_outputs(mbot_servo_test)

#READ-MOTOR-ADC-TEST
add_executable(mbot_read_motor_adc_test
  mbot_read_motor_adc_test.c
)
target_link_libraries(mbot_read_motor_adc_test
  pico_stdlib
  hardware_pwm
  rclib
)
pico_enable_stdio_usb(mbot_read_motor_adc_test 1)
pico_enable_stdio_uart(mbot_read_motor_adc_test 0)
pico_add_extra_outputs(mbot_read_motor_adc_test)

#MOTOR-ANALYZE
add_executable(mbot_motor_analyze
  mbot_motor_analyze.c
)
target_link_libraries(mbot_motor_analyze
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  hardware_adc
  rclib
)
pico_enable_stdio_usb(mbot_motor_analyze 1)
pico_enable_stdio_uart(mbot_motor_analyze 0)
pico_add_extra_outputs(mbot_motor_analyze)

#mbot_calibrate_gyro
add_executable(mbot_closed_loop_ctrler_test
  mbot_closed_loop_ctrler_test.c
)
target_link_libraries(mbot_closed_loop_ctrler_test
  pico_stdlib
  pico_multicore
  hardware_pwm
  hardware_sync
  hardware_i2c
  hardware_flash
  rclib
  comms
)
pico_enable_stdio_usb(mbot_closed_loop_ctrler_test 1)
pico_enable_stdio_uart(mbot_closed_loop_ctrler_test 0)
pico_add_extra_outputs(mbot_closed_loop_ctrler_test)

#I control tests
add_executable(mbot_current_control_test
  mbot_current_control_test.c
)
target_link_libraries(mbot_current_control_test
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  hardware_adc
  rclib
)
pico_enable_stdio_usb(mbot_current_control_test 1)
pico_enable_stdio_uart(mbot_current_control_test 0)
pico_add_extra_outputs(mbot_current_control_test)


#USB serial test
add_executable(mbot_usb_serial_test
  mbot_usb_serial_test.c
)
target_link_libraries(mbot_usb_serial_test
  pico_stdlib
  
)
pico_enable_stdio_usb(mbot_usb_serial_test 1)
pico_enable_stdio_uart(mbot_usb_serial_test 0)
pico_add_extra_outputs(mbot_usb_serial_test)

#Encoder tests
add_executable(mbot_encoder_test
  mbot_encoder_test.c
)
target_link_libraries(mbot_encoder_test
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  rclib
)
pico_enable_stdio_usb(mbot_encoder_test 1)
pico_enable_stdio_uart(mbot_encoder_test 0)
pico_add_extra_outputs(mbot_encoder_test)

#interrupt test
add_executable(pico_interrupt_test
pico_interrupt_test.c
)
target_link_libraries(pico_interrupt_test
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  rclib
)
pico_enable_stdio_usb(pico_interrupt_test 1)
pico_enable_stdio_uart(pico_interrupt_test 0)
pico_add_extra_outputs(pico_interrupt_test)

#IMU test
add_executable(mbot_imu_test
  mbot_imu_test.c
)
target_link_libraries(mbot_imu_test
  pico_stdlib
  hardware_i2c
  rclib
)
pico_enable_stdio_usb(mbot_imu_test 1)
pico_enable_stdio_uart(mbot_imu_test 0)
pico_add_extra_outputs(mbot_imu_test)

#Motor Calibration
add_executable(mbot_calibrate_classic
  mbot_calibrate_classic.c
)
target_link_libraries(mbot_calibrate_classic
  pico_stdlib
  hardware_i2c
  hardware_pio
  hardware_pwm
  rclib
)
pico_enable_stdio_usb(mbot_calibrate_classic 1)
pico_enable_stdio_uart(mbot_calibrate_classic 0)
pico_add_extra_outputs(mbot_calibrate_classic)

#Motor Calibration
add_executable(mbot_calibrate_omni
  mbot_calibrate_omni.c
)
target_link_libraries(mbot_calibrate_omni
  pico_stdlib
  hardware_i2c
  hardware_pio
  hardware_pwm
  rclib
)
pico_enable_stdio_usb(mbot_calibrate_omni 1)
pico_enable_stdio_uart(mbot_calibrate_omni 0)
pico_add_extra_outputs(mbot_calibrate_omni)