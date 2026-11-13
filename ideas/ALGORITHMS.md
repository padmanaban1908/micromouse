# 🧠 Micromouse Algorithms & Mathematics

This guide provides the mathematical and algorithmic foundation of the Micromouse system: maze representation, the Floodfill solver, path cost optimization, and motion control kinematics.

---

## 1. Maze Grid Representation

A standard competition maze is a $16 \times 16$ orthogonal grid of cells, indexed with Cartesian coordinates:
- $(0, 0)$ is the South-West start corner (facing North).
- The destination is the $2 \times 2$ central cluster: $\{(7, 7), (7, 8), (8, 7), (8, 8)\}$.

### Wall Bitmask Encoding
Each cell $(x, y)$ stores its wall configuration in an 8-bit byte:

```c
#define WALL_NORTH 0x01  // (1 << 0)
#define WALL_EAST  0x02  // (1 << 1)
#define WALL_SOUTH 0x04  // (1 << 2)
#define WALL_WEST  0x08  // (1 << 3)
#define VISITED    0x10  // (1 << 4)
```

### Symmetric Wall Updates
Walls are shared boundaries between adjacent cells. Whenever a wall is detected:
$$\text{Wall}_{\text{North}}(x, y) \iff \text{Wall}_{\text{South}}(x, y+1)$$
$$\text{Wall}_{\text{East}}(x, y) \iff \text{Wall}_{\text{West}}(x+1, y)$$
$$\text{Wall}_{\text{South}}(x, y) \iff \text{Wall}_{\text{North}}(x, y-1)$$
$$\text{Wall}_{\text{West}}(x, y) \iff \text{Wall}_{\text{East}}(x-1, y)$$

---

## 2. Classical Floodfill Algorithm

Floodfill computes the shortest distance from every cell in the maze to the destination. It operates like an inverse water wave flowing outward from the goal.

### A. Initialization (Manhattan Distance Potential)
Before exploration begins, every cell $(x, y)$ is initialized with its Manhattan distance to the closest center goal cell:
$$D_0(x, y) = \min_{gx \in \{7,8\}, gy \in \{7,8\}} (|x - gx| + |y - gy|)$$

At the goal cells:
$$D(7, 7) = D(7, 8) = D(8, 7) = D(8, 8) = 0$$

### B. Relaxation & Consistency Condition
A cell $u$ is said to be **consistent** if its distance value satisfies:
$$D(u) = 1 + \min_{v \in \text{AccessibleNeighbors}(u)} D(v)$$

If the robot discovers a new wall that blocks access to its lowest neighbor, cell $u$ becomes **inconsistent** ($D(u) \le \min D(v)$).

### C. Queue-Based Floodfill Update
When inconsistency occurs, the maze potential is re-flooded using a FIFO queue:

```c
void floodfill(int targetX, int targetY) {
    // 1. Push inconsistent cell to queue
    queue.push(currentCell);

    while (!queue.isEmpty()) {
        Cell u = queue.pop();
        int minNeighborDist = 255;

        for (Direction dir : {NORTH, EAST, SOUTH, WEST}) {
            if (!hasWall(u, dir)) {
                Cell v = getNeighbor(u, dir);
                minNeighborDist = min(minNeighborDist, dist[v.x][v.y]);
            }
        }

        // If cell distance is not 1 + minNeighbor, update and propagate
        if (dist[u.x][u.y] != minNeighborDist + 1) {
            dist[u.x][u.y] = minNeighborDist + 1;
            for (Direction dir : {NORTH, EAST, SOUTH, WEST}) {
                if (!hasWall(u, dir)) {
                    queue.push(getNeighbor(u, dir));
                }
            }
        }
    }
}
```

---

## 3. Modified Floodfill: Turn Penalty Optimization

In a physical robot, turning takes significantly more time and energy than driving straight. The Modified Floodfill algorithm factors turn costs into the wave propagation:

$$\text{Cost}(u \to v) = 
\begin{cases} 
C_{\text{straight}} & \text{if heading}(u \to v) == \text{current\_heading} \\
C_{\text{straight}} + C_{\text{turn}} & \text{if heading}(u \to v) \ne \text{current\_heading} 
\end{cases}$$

Typically:
- $C_{\text{straight}} = 1.0$ (or 10 units)
- $C_{\text{turn}} = 2.5$ (or 25 units)
- $C_{\text{u-turn}} = 5.0$ (or 50 units)

This discourages "staircase" zig-zag patterns and favors long straightaways where the robot can achieve top speed.

---

## 4. Differential Drive Kinematics & Odometry

Between cells, the robot maintains precise $(X, Y, \theta)$ tracking using optical wheel encoders:

### Forward Kinematics
Given left wheel displacement $\Delta s_L$ and right wheel displacement $\Delta s_R$ over sample period $\Delta t$:

$$\Delta s = \frac{\Delta s_R + \Delta s_L}{2}$$
$$\Delta \theta = \frac{\Delta s_R - \Delta s_L}{W}$$

where $W$ is the wheel track width ($\sim 70\text{ mm}$ to $80\text{ mm}$).

### Pose Update
$$x_{k+1} = x_k + \Delta s \cdot \cos\left(\theta_k + \frac{\Delta \theta}{2}\right)$$
$$y_{k+1} = y_k + \Delta s \cdot \sin\left(\theta_k + \frac{\Delta \theta}{2}\right)$$
$$\theta_{k+1} = \theta_k + \Delta \theta$$

---

## 5. Wall Centering PID Control

As the mouse travels along a corridor, distance errors to left and right walls generate steering corrections:

$$\text{Error} = (d_{\text{Left}} - d_{\text{Right}}) - (d_{\text{target\_Left}} - d_{\text{target\_Right}})$$

$$\text{SteerCorrection} = K_p \cdot \text{Error} + K_d \cdot \frac{\Delta \text{Error}}{\Delta t} + K_i \int \text{Error} \, dt$$

$$\text{PWM}_{\text{Left}} = \text{BasePWM} - \text{SteerCorrection}$$
$$\text{PWM}_{\text{Right}} = \text{BasePWM} + \text{SteerCorrection}$$

---

## 6. Trapezoidal Velocity Profile

To prevent wheel slip on high-speed runs, acceleration and deceleration are governed by a trapezoidal motion profile:

$$v(t) = 
\begin{cases} 
a \cdot t & 0 \le t < t_{\text{accel}} \\
v_{\max} & t_{\text{accel}} \le t < t_{\text{decel}} \\
v_{\max} - a \cdot (t - t_{\text{decel}}) & t_{\text{decel}} \le t \le t_{\text{total}}
\end{cases}$$

The required stopping distance $d_{\text{stop}}$ from current velocity $v$ at constant deceleration $a$ is:
$$d_{\text{stop}} = \frac{v^2}{2a}$$
