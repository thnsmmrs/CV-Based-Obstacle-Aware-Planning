# Obstacle Aware Motion Planning ECE 5463 Final Project

Plan and execute **collision-free motion** for a robot from point A → B in a 2D workspace with circular and rectangular obstacles, using sampling-based motion planning, spline trajectories, and PD control.

> Core idea:  
> Use an RRT-style planner in task-space or configuration-space to find a feasible path, smooth it into a spline, run FK/IK to simulate the robot, and track the trajectory with a PD controller. Extensions include spline smoothing for low jerk and dynamic re-planning around moving obstacles.

---

## 1. Project Goals

**Primary goals**

- Plan a path from start → goal while avoiding:
  - Circular obstacles
  - Rectangular obstacles
- Plan in either:
  - **Task space** (end-effector position), or  
  - **Configuration space** (joint angles)
- Simulate and **visualize**:
  - The workspace with obstacles
  - The planned path
  - The robot following the path (via FK/IK)
- Track the motion with a **PD controller** (similar to PA3)

**Stretch / extension goals**

- Use **spline smoothing** (e.g., cubic / quintic splines) to:
  - Reduce acceleration and jerk
  - Improve hardware lifetime and motion quality
- Implement **dynamic obstacle avoidance**:
  - Moving obstacles
  - Re-plan at some fixed frequency (e.g., every N timesteps)
- Integrate more realistic environments:
  - Randomly generated layouts
  - Simple “warehouse-like” maps from online datasets
  - (Optional) Depth camera input to detect obstacles

---

## 2. High-Level Architecture

Pipeline:

1. **Robot & Kinematics**
   - Define a simple manipulator model (e.g., planar 2R/3R or given course robot).
   - Implement **FK** and **IK** for the robot:
     - FK: joint → end-effector pose
     - IK: end-effector pose → joint configurations (closed-form or numeric)

2. **Environment & Obstacles**
   - 2D workspace (x–y plane).
   - Circular obstacles: `(x_c, y_c, r)`
   - Rectangular obstacles: axis-aligned or oriented boxes.
   - Collision checking:
     - For a robot configuration: check if any link intersects an obstacle.
     - For a point in task space: check if it lies inside an obstacle.

3. **Motion Planner (RRT or similar)**
   - Use RRT / RRT-Connect / RRT* to find a collision-free path:
     - Either in joint space (q) or task space (x, y).
   - Output:
     - A list of waypoints from start → goal that avoids obstacles.

4. **Spline Trajectory Generation**
   - Convert the planner waypoints into a **time-parameterized spline**:
     - Piecewise cubic or quintic splines.
     - Optionally enforce velocity and acceleration constraints.
   - Compute:
     - Position, velocity, acceleration (and approximate jerk) along the path.

5. **PD Control**
   - Use a joint-space PD controller:
     - `τ = Kp (q_des − q) + Kd (q̇_des − q̇)`
   - Simulate joint dynamics (simplified, e.g., unit mass / inertia).
   - Show tracking error vs time.

6. **Dynamic Obstacle Avoidance (extension)**
   - Add moving obstacles with known trajectories.
   - At fixed re-plan intervals:
     - Check for predicted collisions.
     - Run planner again from current state → goal.
     - Stitch/plumb a new spline from the new plan.

7. **Visualization**
   - Plot 2D workspace:
     - Obstacles (circles, rectangles)
     - Start, goal
     - RRT tree (optional)
     - Final path and smoothed spline
   - Animate robot arm following the trajectory:
     - Links, joints
     - Collision overlays
   - Plot metrics:
     - Position vs time
     - Velocity, acceleration, jerk vs time
     - Tracking error (PD performance)

---

## 3. Project Requirements

**Technical**

- Motion planning:
  - RRT / RRT-Connect / RRT* (at least one).
- Kinematics:
  - Forward and inverse kinematics for chosen robot.
- Control:
  - PD controller for joint tracking.
- Simulation:
  - Discrete-time simulation of robot trajectory.
- Visualization:
  - Workspace + obstacles + path.
  - Animation of motion (or at least snapshots).

**Language / Tools** (example – adjust as needed)

- Python + NumPy + Matplotlib (or equivalent)  
- OR MATLAB (if we decide to mirror PA2/PA3 environment).
- Version control via Git + GitHub.

We’ll standardize on one stack in early development and keep the codebase consistent.
