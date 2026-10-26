"""BCU3: ten gain slots contain the cascade PID gains."""
import struct
LQI_SCALE_NAMES=('distance','speed','pitch','pitch_rate','yaw','yaw_rate','position_integral','yaw_integral')
WIRE2=struct.Struct("<IIQQQ13I34f")
FIELDS2=['magic', 'sequence', 'sent_us', 'sample_us', 'imu_us', 'cycle', 'dt_us', 'revision', 'mode', 'result', 'fault', 'flags', 'saturated', 'imu_age_us', 'encoder_age_us', 'current_age_a_us', 'current_age_b_us', 'calculate_us', 'pitch', 'roll', 'pitch_rate', 'yaw', 'yaw_rate', 'speed', 'distance', 'target_speed', 'target_pitch', 'command_a', 'command_b', 'iq_a', 'iq_b', 'id_a', 'id_b', 'raw_a', 'raw_b', 'angle_a', 'angle_b', 'bus', 'current_limit_A', 'velocity_integral', 'pitch_integral', 'yaw_integral', 'velocity_kp', 'velocity_ki', 'pitch_kd', 'pitch_kp', 'pitch_ki', 'yaw_kp', 'yaw_ki', 'yaw_kd', 'position_kp', 'pitch_trim_rad']

ATTITUDE_WIRE=struct.Struct('<IIQ19f6IfI')
ATTITUDE_FIELDS=['magic','sequence','time_us','qw','qx','qy','qz','bx','by','bz','roll','pitch','yaw','gyro_x','gyro_y','gyro_z','p_rx','p_ry','p_rz','p_bx','p_by','p_bz','compute_us','gravity_updates','gravity_rejected','rest_updates','confirmed_rest','stack_high_water','nis','crc']
YAW_WIRE=struct.Struct('<IIQ10f3I')
YAW_FIELDS=['magic','sequence','time_us','raw_yaw','encoder_yaw','corrected_yaw','bias','gyro_rate','encoder_rate','corrected_rate','quiet_time','target','target_rate','flags','phase','crc']

IMU_WIRE=struct.Struct('<IIQQI7hH6fI')
IMU_FIELDS=['magic','sequence','time_us','read_began_us','sample_sequence','acc_x_raw','acc_y_raw','acc_z_raw','temperature_raw','gyro_x_raw','gyro_y_raw','gyro_z_raw','reserved','gyro_x','gyro_y','gyro_z','roll_x','pitch_y','yaw_z','crc']
