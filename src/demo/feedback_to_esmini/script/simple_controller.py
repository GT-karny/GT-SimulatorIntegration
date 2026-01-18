import math
import sys
import os

# Ensure OSI bindings are available
try:
    current_dir = os.path.dirname(os.path.abspath(__file__))
    osi3_path = os.path.join(current_dir, 'osi3')
    if osi3_path not in sys.path:
        sys.path.append(osi3_path)
    import osi3.osi_sensorview_pb2 as osi_sv
except ImportError:
    osi_sv = None
    print("[simple_controller] Warning: Could not import osi3 bindings")

class PIDController:
    def __init__(self, kp, ki, kd, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral = 0
        self.prev_error = 0

    def calc(self, target, current, dt):
        error = target - current
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        if self.output_limit:
            output = max(-self.output_limit, min(self.output_limit, output))
            
        return output

class LongitudinalController:
    def __init__(self):
        # PID params should be tuned
        self.pid = PIDController(kp=1.0, ki=0.1, kd=0.05, output_limit=1.0) 

    def update(self, current_speed, target_speed, dt):
        """
        Returns (throttle, brake)
        """
        control_val = self.pid.calc(target_speed, current_speed, dt)
        
        throttle = 0.0
        brake = 0.0
        
        if control_val >= 0:
            throttle = control_val
        else:
            brake = -control_val
            
        print(f"[Longitudinal] Cur: {current_speed:.2f}, Tgt: {target_speed:.2f} -> Thr: {throttle:.2f}, Brk: {brake:.2f}")
        return throttle, brake

class LateralController:
    def __init__(self, wheelbase=3.0, lookahead_dist=5.0):
        self.L = wheelbase
        self.ld = lookahead_dist

    def update(self, ego_pos_x, ego_pos_y, ego_yaw, trajectory_points):
        """
        Pure Pursuit Implementation
        Returns steering angle (-1.0 to 1.0, normalized)
        """
        if not trajectory_points:
            return 0.0

        # 1. Find target point (closest point + lookahead)
        target_pt = None
        
        # Searching for the lookahead point
        for pt in trajectory_points:
            dx = pt.x - ego_pos_x
            dy = pt.y - ego_pos_y
            dist = math.hypot(dx, dy)
            
            if dist >= self.ld:
                target_pt = pt
                break
        
        if target_pt is None:
            target_pt = trajectory_points[-1]

        # 2. Transform to vehicle coordinate
        dx_global = target_pt.x - ego_pos_x
        dy_global = target_pt.y - ego_pos_y
        
        # alpha is the angle to the target point relative to vehicle heading
        alpha = math.atan2(dy_global, dx_global) - ego_yaw
        
        # 3. Calculate steering angle
        # curvature k = 2 * sin(alpha) / Ld
        steer_rad = math.atan2(2.0 * self.L * math.sin(alpha), self.ld)
        
        # Normalize to -1.0 to 1.0 (assuming max steer is approx 0.7 rad)
        max_steer = 0.7
        normalized_steer = max(-1.0, min(1.0, steer_rad / max_steer))
        
        return normalized_steer

class OSIParser:
    def __init__(self, binary_data, prev_timestamp):
        self.valid = False
        self.ego_x = 0
        self.ego_y = 0
        self.ego_yaw = 0
        self.current_speed = 0
        self.target_speed = 0
        self.trajectory_points = []
        self.dt = 0.01
        self.timestamp = prev_timestamp

        if osi_sv is None:
            return

        try:
            sv = osi_sv.SensorView()
            sv.ParseFromString(binary_data)
            
            # Timestamp Calc
            new_timestamp = sv.timestamp.seconds + sv.timestamp.nanos * 1e-9
            self.dt = new_timestamp - prev_timestamp
            if self.dt <= 0: self.dt = 0.01 
            self.timestamp = new_timestamp

            # Extract Ego
            # Find the object that matches the host_vehicle_id
            host_id = sv.host_vehicle_id
            ego = None
            
            for obj in sv.global_ground_truth.moving_object:
                if obj.id.value == host_id.value:
                    ego = obj
                    break
            
            # Fallback if not found (or host_id not set properly), try to pick one that looks like a car or just first
            if ego is None and len(sv.global_ground_truth.moving_object) > 0:
                print(f"[OSIParser] Warning: Host ID {host_id.value} not found in objects. Using first object.")
                ego = sv.global_ground_truth.moving_object[0]

            if ego:
                self.ego_x = ego.base.position.x
                self.ego_y = ego.base.position.y
                self.ego_yaw = ego.base.orientation.yaw
                
                # Current Speed
                ego_vel_vec = ego.base.velocity
                self.current_speed = math.sqrt(ego_vel_vec.x**2 + ego_vel_vec.y**2 + ego_vel_vec.z**2)
                
                # Extract Trajectory
                future_traj = ego.future_trajectory
                if len(future_traj) > 0:
                    self.trajectory_points = [p.position for p in future_traj]
                    
                    # Target Speed Calculation
                    # Calculate from distance between points (assuming dt=0.5s)
                    if len(future_traj) > 1:
                        dx = future_traj[1].position.x - future_traj[0].position.x
                        dy = future_traj[1].position.y - future_traj[0].position.y
                        dist = math.hypot(dx, dy)
                        self.target_speed = dist / 0.5
                    else:
                         # Fallback if only 1 point
                         self.target_speed = 0.0
                    
                else:
                    print("[OSIParser] No future trajectory found")
                
                # Debug Logging
                # print(f"[OSIParser] Valid. ID: {ego.id.value}, HostID: {host_id.value}, CurSpd: {self.current_speed:.2f}, TgtSpd: {self.target_speed:.2f}")
                self.valid = True
            else:
                print("[OSIParser] No moving objects found")

        except Exception as e:
            print(f"[OSIParser] Exception: {e}")
            self.valid = False

    def get_debug_info(self):
        return f"Spd: {self.current_speed:.2f}/{self.target_speed:.2f}"
