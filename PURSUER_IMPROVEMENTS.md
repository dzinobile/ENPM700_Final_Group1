# Pursuer Node Improvements for Targeting Robot0

## Summary
Updated the pursuer node in the `webots_practice` package to more accurately track and pursue robot0 through improved control algorithms and predictive targeting.

## Key Improvements

### 1. **Enhanced Control Parameters**
Added tunable PID-style gains:
- `kp_angular` (default: 3.5): Proportional gain for angular velocity control
- `kp_linear` (default: 0.8): Proportional gain for linear velocity control  
- `min_linear_speed` (default: 0.05): Minimum forward speed to maintain momentum

### 2. **Predictive Target Tracking**
- Tracks previous target positions to estimate velocity
- Applies a prediction factor (0.5) to lead the target
- Compensates for target movement to reduce lag in pursuit

### 3. **Improved Angular Control**
- Increased proportional gain from 1.0 to 3.5 for more responsive turning
- Better normalization of angle errors
- Separate handling for large angle errors (>90 degrees) - rotates in place

### 4. **Smarter Linear Velocity Control**
- Progressive speed adjustment based on distance (proportional control)
- Alignment-based speed reduction when not facing target
- Minimum speed threshold prevents stalling when close to target
- Stops completely when aligned but facing away (>90° error)

### 5. **Higher Control Frequency**
- Reduced control loop period from 100ms to 50ms (20Hz)
- Provides smoother and more responsive control

### 6. **Better State Management**
- Stores previous positions for both pursuer and target
- Enables velocity estimation and predictive behavior
- More robust handling of edge cases

## Technical Details

### Control Algorithm
```
Angular Velocity: ω = clamp(kp_angular * angle_error, -ω_max, ω_max)

Linear Velocity: v = clamp(kp_linear * distance * alignment_factor, v_min, v_max)
where alignment_factor = max(0.2, cos(angle_error))
```

### Special Behaviors
1. **Large Angle Error (|θ| > 90°)**: Rotate in place (v=0)
2. **Near Target (d < pursuit_distance)**: Stop completely
3. **Normal Pursuit**: Progressive speed with alignment correction

## Building
```bash
colcon build --packages-select webots_practice
```

## Usage
The node automatically uses the improved parameters. To further tune:
```bash
ros2 run webots_practice pursuer --ros-args \
  -p robot_id:=1 \
  -p target_robot_id:=0 \
  -p kp_angular:=3.5 \
  -p kp_linear:=0.8 \
  -p pursuit_distance:=0.1
```

## Expected Behavior
- More aggressive and accurate turning toward robot0
- Smoother acceleration/deceleration
- Better tracking of moving targets
- Reduced oscillation and overshooting
- Faster convergence to target
