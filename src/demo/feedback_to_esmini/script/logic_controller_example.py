try:
    import sys
    import os
    
    # [Workaround] Add osi3 package directory to sys.path
    current_dir = os.path.dirname(os.path.abspath(__file__))
    osi3_path = os.path.join(current_dir, 'osi3')
    if osi3_path not in sys.path:
        sys.path.append(osi3_path)

    import osi3.osi_sensorview_pb2 as osi_sv
    
    # Import our custom controller logic
    from simple_controller import LongitudinalController, LateralController, OSIParser
    
    print("[GT-DriveController] logic_controller_example loaded")
    
except ImportError as e:
    print(f"[GT-DriveController] Error importing modules: {e}")


class Controller:
    def __init__(self):
        print("[GT-DriveController] Initializing Trajectory Follower Controller")
        self.long_ctrl = LongitudinalController()
        # lookahead_dist can be tuned
        self.lat_ctrl = LateralController(wheelbase=3.0, lookahead_dist=12.0)
        self.prev_timestamp = 0.0

    def update_control(self, binary_data):
        # Default safe outputs
        throttle = 0.0
        brake = 1.0
        steering = 0.0
        drive_mode = 1 # Drive
        osi_output_bytes = binary_data
        
        # 1. Parse OSI Data
        parser = OSIParser(binary_data, self.prev_timestamp)
        self.prev_timestamp = parser.timestamp
        
        if not parser.valid:
            return [throttle, brake, steering, drive_mode, osi_output_bytes]

        # 2. Lateral Control (Steering)
        steering = self.lat_ctrl.update(
            parser.ego_x, 
            parser.ego_y, 
            parser.ego_yaw, 
            parser.trajectory_points
        )
        
        # 3. Longitudinal Control (Throttle/Brake)
        throttle, brake = self.long_ctrl.update(
            parser.current_speed, 
            parser.target_speed, 
            parser.dt
        )
        
        return [throttle, brake, steering, drive_mode, osi_output_bytes]
