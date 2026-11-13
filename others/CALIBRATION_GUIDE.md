# 🎯 Sensor Calibration & PID Tuning Guide: Micromouse

Achieving reliable navigation requires calibrating three critical subsystems:
1. **Infrared Proximity Distance Sensors**
2. **Wheel Encoder Odometry**
3. **IMU Gyroscope Heading & In-Place 90° / 180° Turns**

---

## 📡 1. IR Proximity Sensor Calibration

Infrared reflectance voltages follow an inverse power response with distance:
$$V(d) \approx \frac{A}{(d + B)^C} + V_{\text{ambient}}$$

### A. Step-by-Step Procedure
1. Place the mouse centered precisely in a standard corridor (walls at $d = 84\text{ mm}$ on Left and Right).
2. Measure nominal center voltages:
   - $V_{\text{nominal\_L}}$
   - $V_{\text{nominal\_R}}$
3. Shift the robot $\pm 10\text{ mm}$ and record the delta to compute the sensitivity coefficient ($K_{\text{sens}} = \frac{\Delta V}{\Delta d}\text{ mV/mm}$).
4. Determine the **Wall Detection Threshold**:
   - If wall is present at $84\text{ mm}$: Reading $\approx 2200$ (12-bit ADC).
   - If corridor is open ($> 250\text{ mm}$): Reading $\approx 350$.
   - **Threshold Cutoff**: Choose midpoint $\sim 900 - 1100$. Any reading above this confirms wall presence.

---

## ⚙️ 2. Encoder Odometry & Scale Factor

For accurate cell transitions ($180\text{ mm}$ per cell step), compute the exact tick multiplier:

$$\text{Counts Per Revolution (CPR)} = \text{Motor Shaft Ticks} \times 4 \times \text{Gear Ratio}$$

### Example Calculation
- 12 CPR magnetic disc with $4\times$ quadrature decoding = $48\text{ counts/turn}$.
- Gear Ratio $= 30:1$.
- Total Counts per wheel rotation $= 48 \times 30 = 1440\text{ counts}$.
- Wheel Diameter $= 32.0\text{ mm} \implies \text{Circumference} = \pi \times 32.0 = 100.53\text{ mm}$.
- **Scale Factor**:
  $$\text{Ticks per mm} = \frac{1440}{100.53} \approx 14.323\text{ ticks/mm}$$
- **1 Cell Movement ($180\text{ mm}$)**:
  $$\text{Target Ticks} = 180 \times 14.323 \approx 2578\text{ ticks}$$

---

## 🔄 3. Gyroscope Yaw Rate & In-Place Turns

The MPU-6050 measures rotational velocity in $^\circ/\text{sec}$.

### Zero-Rate Drift Compensation
During boot-up (first 2 seconds), keep the robot completely stationary:
```cpp
float gyroOffsetZ = 0;
for (int i = 0; i < 1000; i++) {
    gyroOffsetZ += readRawGyroZ();
    delay(2);
}
gyroOffsetZ /= 1000.0;
```

### In-Place 90° Turn Logic
$$\theta(t) = \int_{0}^{t} (\text{GyroZ} - \text{gyroOffsetZ}) \cdot dt$$

```cpp
void turnInPlace(float targetAngleDeg) {
    float currentAngle = 0;
    unsigned long lastTime = micros();

    while (abs(currentAngle - targetAngleDeg) > 0.5) {
        unsigned long now = micros();
        float dt = (now - lastTime) / 1000000.0;
        lastTime = now;

        float rate = (readRawGyroZ() - gyroOffsetZ) * GYRO_SCALE_FACTOR;
        currentAngle += rate * dt;

        float error = targetAngleDeg - currentAngle;
        float motorCommand = Kp_turn * error;
        setMotorSpeeds(-motorCommand, motorCommand);
    }
    stopMotors();
}
```

---

## 🎛️ 4. PID Wall Centering Tuning

1. Set $K_i = 0$ and $K_d = 0$.
2. Gradually increase $K_p$ until the robot stays between walls without oscillating wildly.
3. Increase $K_d$ to dampen rapid oscillations when approaching cell edges or openings.
4. Keep $K_i$ at zero or very small ($< 0.005$) to avoid integrator windup when traversing asymmetric corridors (walls only on one side).
