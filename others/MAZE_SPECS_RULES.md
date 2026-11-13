# 🏁 Official Micromouse Maze Specifications & Rules

This document outlines the standard dimensional parameters, surface characteristics, and competition rules governing the international **IEEE and All-Japan Micromouse Competitions**.

---

## 📏 1. Maze Geometry & Dimensions

```
   |<---------- 180 mm --------->|
---+-----------------------------+---
 | |                             | |
 | |       168 mm corridor       | |
 | |                             | |
50mm                             | |
 | |                             | |
---+-----------------------------+---
   |<- 12 mm ->|
```

| Parameter | Specification | Tolerance |
|---|---|---|
| **Grid Size** | $16 \times 16$ cells (Classical) or $32 \times 32$ (Half-Size) | Exact |
| **Cell Pitch (Center to Center)** | $180\text{ mm}$ | $\pm 2\text{ mm}$ |
| **Corridor Clear Width** | $168\text{ mm}$ (between wall faces) | $\pm 2\text{ mm}$ |
| **Wall Height** | $50\text{ mm}$ | $\pm 2\text{ mm}$ |
| **Wall Thickness** | $12\text{ mm}$ | $\pm 1\text{ mm}$ |
| **Grid Post (Peg) Size** | $12\text{ mm} \times 12\text{ mm} \times 50\text{ mm}$ | $\pm 1\text{ mm}$ |
| **Total Maze Footprint** | $2.88\text{ m} \times 2.88\text{ m}$ (Classical 16×16) | $\pm 10\text{ mm}$ |

---

## 🎨 2. Optical & Surface Specifications

- **Maze Floor**:
  - Material: Wood / MDF or high-density composite board.
  - Finish: Matte non-reflective black paint.
  - Static friction coefficient $\mu_s \approx 0.5 - 0.7$ against standard rubber/silicone tires.
- **Maze Walls**:
  - Sides: Matte white finish (reflective to infrared light $\sim 850\text{ nm} - 940\text{ nm}$).
  - Top Surface: Painted vivid red (indicates wall presence to human spectators and overhead cameras).
- **Start Cell**:
  - Located at the South-West corner: coordinate $(0, 0)$.
  - Enclosed by walls on three sides (South, West, and East in standard layouts, with exit to North).
- **Goal Zone**:
  - Consists of a $2 \times 2$ block of 4 cells: $\{(7, 7), (7, 8), (8, 7), (8, 8)\}$.
  - The perimeter has an opening (at least one entry point); there are no interior walls inside the $2 \times 2$ goal square.
  - A small marker pole or flag is placed at the center intersection peg $(8, 8)$ in some tournaments.

---

## ⏱️ 3. Competition Rules & Scoring

1. **Allocated Time**:
   - Each mouse is granted a total time window (typically $7\text{ to }10\text{ minutes}$) inside the maze.
2. **Number of Runs**:
   - A robot may make up to $5$ runs within its allotted time.
   - The fastest successful run to the goal determines the official score.
3. **Autonomous Requirement**:
   - Once placed in the start cell, the mouse must operate completely autonomously.
   - No remote control, wireless transmission of maps, or human intervention while running is allowed.
4. **Touch Penalties**:
   - If the robot crashes or loses localization and must be picked up by the operator, a touch penalty (e.g. $+2\text{ to }+5\text{ seconds}$) is assessed, and the mouse must restart from cell $(0, 0)$.
