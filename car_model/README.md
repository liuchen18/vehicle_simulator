# vehicle model 
control command topic:<br>
    steering_cmd - std_msgs/Float64 topic containing the desired steering wheel angle in radians<br>
    brake_cmd - std_msgs/Float64 topic containing the desired brake torque in Newton-meters (Nm)<br>
    throttle_cmd - std_msgs/Float64 topic containing the desired throttle percentage (range 0 to 1)<br>
    gear_cmd - std_msgs/UInt8 topic containing the desired gear (DRIVE = 0, REVERSE = 1)<br>

feedback topic:<br>
    twist - geometry_msgs/TwistStamped containing the current velocity<br>
    gear_state - std_msgs/UInt8 containing the direction :forward or backward((DRIVE = 0, REVERSE = 1))<br>

