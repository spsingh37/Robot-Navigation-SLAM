#MBOT
add_executable(mbot
  mbot.c
  odometry.c
  print_tables.c
)

include_directories(/usr/local/include)

target_link_libraries(mbot
  pico_stdlib
  pico_multicore
  pico_time
  hardware_pwm
  hardware_sync
  hardware_i2c
  hardware_flash
  mbotlib
  rclib
  comms
)
pico_enable_stdio_usb(mbot 1)
pico_enable_stdio_uart(mbot 0)
pico_add_extra_outputs(mbot)
