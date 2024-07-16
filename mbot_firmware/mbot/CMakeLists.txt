add_library(mbotlib STATIC
  # MOTOR
  src/motor/motor.c
  # SERVO
  src/servo/servo.c
  # ENCODER
  src/encoder/encoder.c
  # FRAM chip
  src/fram/fram.c
  # BOSCH IMU
  src/imu/bhy_support.c
  src/imu/bhy_uc_driver.c
  src/imu/bhy.c
  src/imu/imu.c
  # BAROMETER
  src/barometer/barometer.c
  # DEFS
  src/defs/mbot_defs.c
  # UTILS
  src/utils/utils.c
)

target_link_libraries(mbotlib
pico_stdlib
pico_multicore
hardware_pwm
hardware_i2c
hardware_pio
hardware_adc
hardware_flash
)

pico_generate_pio_header(mbotlib ${CMAKE_SOURCE_DIR}/mbot/include/mbot/encoder/quadrature_encoder.pio)

target_include_directories(mbotlib PUBLIC
  include
)
