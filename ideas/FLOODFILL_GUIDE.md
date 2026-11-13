# 🌊 Complete Guide to the Floodfill Algorithm for Micromouse

The **Floodfill Algorithm** is the definitive, industry-standard path planning algorithm for competition Micromouse robots. It guarantees discovering the optimal path to the goal while actively updating its internal mental map as walls are encountered.

---

## 🧭 Why Floodfill?

In traditional graph search (like Dijkstra or A*), an agent assumes the graph is static and known in advance. In a real Micromouse competition:
1. The maze layout is completely unknown at the start.
2. The robot discovers walls one cell at a time.
3. Every time a new wall is detected, paths that once seemed optimal may now be dead-ends.

Floodfill treats the maze like a physical basin where water is continuously pumped into the destination (Goal cells) at height 0. Water flows outward into accessible cells, giving each cell a **height** (distance potential). The robot simply behaves like a marble rolling down the gradient (from higher distance numbers to lower distance numbers).

---

## 🔢 Mathematical Formulation

### 1. The Potential Field $V(x, y)$
For any cell $u = (x, y)$:
- If $u \in \text{Goals}$, $V(u) = 0$.
- Otherwise:
  $$V(u) = 1 + \min_{v \in \mathcal{N}_{\text{open}}(u)} V(v)$$
  where $\mathcal{N}_{\text{open}}(u)$ is the set of adjacent neighbor cells not blocked by a wall.

### 2. Initial Matrix (Empty Maze Assumption)
Before moving, the robot initializes every cell with its unobstructed Manhattan distance:
$$V_0(x, y) = \min_{(gx, gy) \in \text{Goals}} (|x - gx| + |y - gy|)$$

For a $16 \times 16$ maze with center goals at $(7,7), (7,8), (8,7), (8,8)$:
- Starting cell $(0, 0)$ begins with potential:
  $$V_0(0, 0) = |0 - 7| + |0 - 7| = 14$$

---

## 🔄 The Floodfill Execution Cycle

At every single cell step:
1. **Read Sensors**: Sample Left, Front, and Right distance sensors.
2. **Update Wall Map**: If a wall is detected on North, East, South, or West, record it in both the current cell and the neighboring cell's opposite face.
3. **Check Consistency**:
   - Look at the accessible neighbors of the current cell.
   - Check if $V(\text{current}) == 1 + \min(V(\text{neighbors}))$.
4. **Flood (if Inconsistent)**:
   - If blocked by a newly discovered wall, push the current cell into a FIFO queue.
   - Pop cell $u$, recalculate $V(u) = 1 + \min(V(\text{open\_neighbors}))$.
   - If $V(u)$ changed, push all accessible neighbors into the queue.
   - Continue until the queue is empty.
5. **Decide Next Step**:
   - Examine accessible neighbors.
   - Turn and move toward the neighbor with the **strictly smallest potential value**.
   - If multiple neighbors tie, prefer straight over turning to minimize motor deceleration.

---

## 🔁 Return-to-Start & Exploration Completion

A rookie mistake is running to the center goal and immediately stopping. The competition allows multiple runs (typically 5 attempts within 7 to 10 minutes).

### Return-to-Start Procedure
1. Upon reaching the goal center $(7..8, 7..8)$, pause and flash LEDs.
2. Invert the Floodfill target:
   - Set destination to $(0, 0)$ ($V(0, 0) = 0$).
   - Run Floodfill to recompute distances back to $(0, 0)$.
3. Navigate back to start. While returning, the robot often travels through unvisited corridors, discovering shortcuts that were hidden during the forward run!
4. Once safely back at $(0, 0)$, compute the global shortest path using only confirmed walls.
5. Engage **Speed Run Mode**!

---

## 💻 C++ Implementation Reference

```cpp
#define MAZE_SIZE 16
uint8_t mazeWalls[MAZE_SIZE][MAZE_SIZE];
uint8_t cellDist[MAZE_SIZE][MAZE_SIZE];

struct Coord { int8_t x, y; };

void recomputeFloodfill(Coord goal1, Coord goal2, Coord goal3, Coord goal4) {
    // Standard BFS queue implementation
    Coord queue[MAZE_SIZE * MAZE_SIZE];
    int head = 0, tail = 0;

    // Reset distances
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            cellDist[x][y] = 255;
        }
    }

    // Enqueue goal cells with distance 0
    auto enqueue = [&](Coord c, uint8_t d) {
        cellDist[c.x][c.y] = d;
        queue[tail++] = c;
    };

    enqueue(goal1, 0);
    enqueue(goal2, 0);
    enqueue(goal3, 0);
    enqueue(goal4, 0);

    const int dx[4] = {0, 1, 0, -1}; // N, E, S, W
    const int dy[4] = {1, 0, -1, 0};
    const uint8_t wallMask[4] = {WALL_N, WALL_E, WALL_S, WALL_W};

    while (head < tail) {
        Coord cur = queue[head++];
        uint8_t curDist = cellDist[cur.x][cur.y];

        for (int dir = 0; dir < 4; dir++) {
            // If no wall in direction 'dir'
            if (!(mazeWalls[cur.x][cur.y] & wallMask[dir])) {
                int nx = cur.x + dx[dir];
                int ny = cur.y + dy[dir];

                if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
                    if (cellDist[nx][ny] > curDist + 1) {
                        cellDist[nx][ny] = curDist + 1;
                        queue[tail++] = { (int8_t)nx, (int8_t)ny };
                    }
                }
            }
        }
    }
}
```
