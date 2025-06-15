# Mechanical Model and Kinematics Documentation

## Robot Physical Parameters
- Robot Radius (L): 61mm (distance from center to wheel)
- Wheel Radius (r): 27mm
- Wheel Installation Angles (the wheels tangential form the chassis (similar to traditional 3 wheel omnidirectial setup)):
  * Wheel 1 (Front-Right): $\theta_1 = -75^\circ = -\frac{5\pi}{12}$
  * Wheel 2 (Rear): $\theta_2 = 180^\circ = \pi$
  * Wheel 3 (Front-Left): $\theta_3 = 75^\circ = \frac{5\pi}{12}$

## Motor Configuration
- Motor Type: DJI M2006
- Maximum Current: 0.6A
- Motor IDs:
  * ID1 (index 0): Front-right wheel (-75бу)
  * ID2 (index 1): Rear wheel (180бу)
  * ID3 (index 2): Front-left wheel (+75бу)

## Kinematic Model

### Inverse Kinematics
Converts robot velocity to wheel velocities.

#### Matrix Form
$$
\begin{bmatrix} 
\omega_1 \\ 
\omega_2 \\ 
\omega_3
\end{bmatrix} = 
\frac{1}{r}
\begin{bmatrix}
\sin(\theta_1) & \cos(\theta_1) & L \\
\sin(\theta_2) & \cos(\theta_2) & L \\
\sin(\theta_3) & \cos(\theta_3) & L
\end{bmatrix}
\begin{bmatrix}
v_x \\
v_y \\
\omega
\end{bmatrix}
$$

Where:
- $\omega_{1,2,3}$ are wheel angular velocities
- $v_x$ is robot velocity in x direction (right positive)
- $v_y$ is robot velocity in y direction (forward positive)
- $\omega$ is robot angular velocity (clockwise positive)

#### Expanded Form
$$
\begin{align*}
\omega_1 &= \frac{1}{r}(v_x\sin(\theta_1) + v_y\cos(\theta_1) + L\omega) \\
\omega_2 &= \frac{1}{r}(v_x\sin(\theta_2) + v_y\cos(\theta_2) + L\omega) \\
\omega_3 &= \frac{1}{r}(v_x\sin(\theta_3) + v_y\cos(\theta_3) + L\omega)
\end{align*}
$$

### Forward Kinematics
Converts wheel velocities to robot velocity.

#### Matrix Form
$$
\begin{bmatrix}
v_x \\
v_y \\
\omega
\end{bmatrix} =
\frac{r}{3}
\begin{bmatrix}
\sin(\theta_1) & \sin(\theta_2) & \sin(\theta_3) \\
\cos(\theta_1) & \cos(\theta_2) & \cos(\theta_3) \\
1/L & 1/L & 1/L
\end{bmatrix}
\begin{bmatrix}
\omega_1 \\
\omega_2 \\
\omega_3
\end{bmatrix}
$$

#### Expanded Form
$$
\begin{align*}
v_x &= \frac{r}{3}(\omega_1\sin(\theta_1) + \omega_2\sin(\theta_2) + \omega_3\sin(\theta_3)) \\
v_y &= \frac{r}{3}(\omega_1\cos(\theta_1) + \omega_2\cos(\theta_2) + \omega_3\cos(\theta_3)) \\
\omega &= \frac{r}{3L}(\omega_1 + \omega_2 + \omega_3)
\end{align*}
$$

## Velocity and Current Limits

### Robot Velocity Limits
- Maximum Linear Velocity: 1.0 m/s
- Maximum Angular Velocity: 2.0 rad/s

### Current Mapping
Motor current is proportional to wheel velocity:
$$
I = \frac{\omega}{\omega_{max}} \cdot I_{max}
$$

Where:
- $I$ is motor current
- $\omega$ is wheel angular velocity
- $\omega_{max}$ is maximum angular velocity
- $I_{max}$ is maximum current (0.6A)

## Remote Control Mapping
- Channel 2 (decoded[2]): Forward/backward movement ($v_y$)
- Channel 3 (decoded[3]): Left/right movement ($v_x$)
- Channel 0 (decoded[0]): Rotation ($\omega$)
- Channel 4 (decoded[4]): AUX1 button (Reserved for future use)
- Channel 5 (decoded[5]): AUX2 button (Reserved for future use) 