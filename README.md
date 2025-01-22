# PCA9685 Hardware Interface

### `ros2_control` Parameters

| Parameter         | Required | Type | Description                            |
|-------------------|----------|------|----------------------------------------|
| `i2c_device`       | false | string  | I2C bus to use e.g. "/dev/i2c-1"      |
| `i2c_address`      | false | int     | I2C address of the PCA9685 to control |
| `pwm_frequency`    | false | double  | PWM frequency                         |
| `joint/channel`    | true  | integer | Driver channel to output on [0-15]    |
| `joint/pwm_low`    | false | double  | Pulse width of minimum angle / speed  |
| `joint/pwm_high`   | false | double  | Pulse width of maximum angle / speed  |
| `joint/pwm_zero`   | false | double  | Pulse width of zero speed (velocity command only)            |
| `joint/vel_scale`  | false | double  | Rotational speed (rad/s) at pwm_high (velocity command only) |


### Tips

The `pwm_test` executable (`ros2 run pca9685_hardware_interface pwm_test --ros-args -p channel:=0`) can be used to determine PWM limits.

For position servos, zero is set in the middle of the reachable range to follow MoveIt's convention.

If a state interface is specified, it must match the command interface. The reported state is simply the last command.