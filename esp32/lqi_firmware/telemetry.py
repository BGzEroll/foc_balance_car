"""BCU3 wire layout; legacy PID gain slots are reserved zeros in LQI."""
import struct
WIRE2=struct.Struct("<IIQQQ13I34f")
FIELDS2=['magic', 'sequence', 'sent_us', 'sample_us', 'imu_us', 'cycle', 'dt_us', 'revision', 'mode', 'result', 'fault', 'flags', 'saturated', 'imu_age_us', 'encoder_age_us', 'current_age_a_us', 'current_age_b_us', 'calculate_us', 'pitch', 'roll', 'pitch_rate', 'yaw', 'yaw_rate', 'speed', 'distance', 'target_speed', 'target_pitch', 'command_a', 'command_b', 'iq_a', 'iq_b', 'id_a', 'id_b', 'raw_a', 'raw_b', 'angle_a', 'angle_b', 'bus', 'current_limit_A', 'velocity_integral', 'pitch_integral', 'yaw_integral', 'velocity_kp', 'velocity_ki', 'pitch_kd', 'pitch_kp', 'pitch_ki', 'yaw_kp', 'yaw_ki', 'yaw_kd', 'position_kp', 'pitch_trim_rad']
