#include <mbot_lcm_msgs_serial.h>
#include <mbot/defs/mbot_params.h>

/**
 * @brief Calculate the body velocity of a differential drive robot
 * 
 * @param[in] wheel_left_vel  Velocity of the left wheel in rad/s
 * @param[in] wheel_right_vel Velocity of the right wheel in rad/s
 * @param[out] mbot_vel       Pointer to the structure where the calculated body velocity will be stored
 * @return int                Returns 0 on success
 */
int mbot_calculate_diff_body_vel(float wheel_left_vel, float wheel_right_vel, serial_twist2D_t *mbot_vel);

/**
 * @brief Calculate the body velocity of a differential drive robot using an IMU for angular velocity
 * 
 * @param[in] wheel_left_vel  Velocity of the left wheel in rad/s
 * @param[in] wheel_right_vel Velocity of the right wheel in rad/s
 * @param[in] imu             IMU data
 * @param[out] mbot_vel       Pointer to the structure where the calculated body velocity will be stored
 * @return int                Returns 0 on success
 */
int mbot_calculate_diff_body_vel_imu(float wheel_left_vel, float wheel_right_vel, serial_mbot_imu_t imu, serial_twist2D_t *mbot_vel);

/**
 * @brief Calculate the body velocity of an omnidirectional (Kiwi) robot
 * 
 * @param[in] wheel_left_vel  Velocity of left wheel in rad/s
 * @param[in] wheel_right_vel Velocity of right wheelin rad/s
 * @param[in] wheel_back_vel  Velocity of back wheel in rad/s
 * @param[out] mbot_vel   Pointer to the structure where the calculated body velocity will be stored
 * @return int            Returns 0 on success
 */
int mbot_calculate_omni_body_vel(float wheel_left_vel, float wheel_right_vel, float wheel_back_vel, serial_twist2D_t *mbot_vel);

/**
 * @brief Calculate the body velocity of an omnidirectional (Kiwi) robot using an IMU for angular velocity
 * 
 * @param[in] wheel_left_vel  Velocity of left wheel in rad/s
 * @param[in] wheel_right_vel Velocity of right wheelin rad/s
 * @param[in] wheel_back_vel  Velocity of back wheel in rad/s
 * @param[in] imu         IMU data
 * @param[out] mbot_vel   Pointer to the structure where the calculated body velocity will be stored
 * @return int            Returns 0 on success
 */
int mbot_calculate_omni_body_vel_imu(float wheel_left_vel, float wheel_right_vel, float wheel_back_vel, serial_mbot_imu_t imu, serial_twist2D_t *mbot_vel);

/**
 * @brief Calculate the odometry (position and orientation) of the robot based on its body velocity
 * 
 * @param[in] mbot_vel  Current body velocity of the robot
 * @param[in] dt        Time interval over which the velocity is applied
 * @param[out] odometry Pointer to the structure where the calculated odometry will be stored
 * @return int          Returns 0 on success
 */
int mbot_calculate_odometry(serial_twist2D_t mbot_vel,  float dt, serial_pose2D_t *odometry);

