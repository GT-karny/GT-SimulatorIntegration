try:
    import sys
    import os
    
    # [Workaround] Add osi3 package directory to sys.path
    # The generated protobuf files use flat imports (e.g. 'import osi_common_pb2')
    # so their containing directory must be in sys.path.
    current_dir = os.path.dirname(os.path.abspath(__file__))
    osi3_path = os.path.join(current_dir, 'osi3')
    if osi3_path not in sys.path:
        sys.path.append(osi3_path)

    import osi3.osi_sensorview_pb2 as osi_sv
    print("[GT-DriveController] OSI bindings imported successfully")
except ImportError as e:
    # Fallback or error logging if osi3 not available in environment
    print(f"[GT-DriveController] Error: Could not import osi3.osi_sensorview_pb2: {e}")
    # Print sys.path for debugging
    print(f"[GT-DriveController] sys.path: {sys.path}")
    osi_sv = None

class Controller:
    def __init__(self):
        print("[GT-DriveController] Initialized Python Controller")

    def update_control(self, binary_data):
        """
        Processes OSI SensorView data and returns control outputs.
        """
        # Default outputs
        throttle = 0
        brake = 1
        steering = 0
        drive_mode = 1 # 1: Forward, 0: Neutral, -1: Reverse
        osi_output_bytes = binary_data # Passthrough for now
        
        if osi_sv is None:
            return [throttle, brake, steering, drive_mode, osi_output_bytes]

        try:
            # Deserialize OSI
            sv = osi_sv.SensorView()
            sv.ParseFromString(binary_data)
            
            # TODO: Implement actual logic based on sv content
            
            return [throttle, brake, steering, drive_mode, osi_output_bytes]

        except Exception as e:
            print(f"[GT-DriveController] Error in update_control: {e}")
            return [0.0, 0.0, 0.0, 0, b""]
