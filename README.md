# Coverage Path Planning for Mobile Robots using Grid-Based Boustrophedon Cell Decomposition (BCD)

## Overview
This project implements a complete **grid-based coverage path planning pipeline** for a mobile robot, designed and tested on **Boston Dynamics Spot**. The system converts raw environment data into an occupancy grid, decomposes free space using **Boustrophedon Cell Decomposition (BCD)**, and generates executable coverage paths that respect obstacles and robot kinematics.

The work emphasizes:
- Robust map preprocessing
- Correct BCD sweep-line event handling
- Graph-based cell connectivity
- Deterministic coverage path generation
- Interactive UI-based path editing
- Reliable waypoint execution on a real robot

---

## Motivation
Standard shortest-path planners (A*, Dijkstra) are insufficient for **coverage tasks** where the objective is to systematically visit all free space. Boustrophedon decomposition provides:
- Reduced complexity compared to exact cellular decomposition
- Deterministic coverage guarantees
- Natural “lawnmower / S-pattern” traversal

This project focuses on implementing BCD **from scratch** on a grid representation and integrating it with a real robotic platform.

---

## Environment Representation

- Input maps are grayscale images  
  - White = free space  
  - Black = obstacles
- Converted into a grid of discrete cells at configurable resolution
- Each cell stores occupancy and adjacency information

---

## Map Preprocessing

To ensure reliable decomposition and connectivity:

- **Otsu thresholding** is used for binarization
- Obstacles are **flood-filled and morphologically closed**
- Careful tuning avoids obstacle inflation that would reduce free space

> Proper obstacle preprocessing proved critical — incomplete contours or over-dilation caused invalid BCD connectivity.

---

## Boustrophedon Cell Decomposition (BCD)

### Sweep-Line Method
- Vertical sweep from left → right across the grid
- Each column is segmented into contiguous free-space intervals
- Intervals are compared with the previous column to determine connectivity

### Event Classification
- **IN**: New free-space segment appears  
- **OUT**: Free-space segment disappears  
- **FLOOR / CEIL**: Segment expands or contracts vertically  

Connectivity is determined strictly by **interval overlap**, ensuring correct cell splitting and merging.

---

## Cell Connectivity Graph

- Each BCD cell is treated as a node
- Adjacency edges represent navigable transitions between cells
- The resulting graph enables deterministic traversal strategies

Traversal strategy:
- **BFS** for predictable and debuggable coverage order

---

## Coverage Path Generation

- Cells are visited in graph traversal order
- Each cell is traversed using an alternating direction strategy
  - Even cells: top → bottom
  - Odd cells: bottom → top

This naturally produces the classic **boustrophedon (S-shaped) coverage pattern**.

---

## UI & Human-in-the-Loop Control

An OpenCV-based UI enables:
- Visualizing the grid, obstacles, and paths
- Clicking cells to define waypoints
- Keyboard controls for saving/loading paths

Stored data:
- Ordered cell indices
- Reloadable paths aligned with map orientation

This UI significantly accelerated debugging and real-world testing.

---

## Coordinate Frames

- **Grid frame**: discrete cell indices (planning)
- **Map frame**: metric coordinates (execution)
- Paths can be saved and reloaded with yaw alignment to maintain consistency across runs

---

## Waypoint Execution

Design principles:
- No global replanning
- Waypoints executed **sequentially**
- Optional yaw goals per waypoint

Execution loop:
1. Query robot pose
2. Send relative motion command
3. Enforce position and yaw tolerances
4. Retry on transient failures

> The robot is not allowed to skip waypoints — each must be reached before advancing.

---

## Failure Handling

Early testing revealed that single command failures could stall execution.  
This was resolved by:
- Adding retry logic with timeouts
- Continuing execution unless repeated failures occur

This significantly improved robustness during real-world operation.

---

## Results

- Accurate obstacle-respecting coverage
- Clean S-pattern traversal of free space
- Deterministic and repeatable behavior
- Successful execution on **Boston Dynamics Spot**

---

## Lessons Learned

- BCD correctness depends heavily on **event classification**
- Obstacle preprocessing is as important as planning
- Deterministic planners are easier to validate on real robots
- Interactive visualization dramatically improves development speed

---

## Future Work

- Polygon-based (non-grid) BCD
- Online replanning for dynamic obstacles
- Path smoothing with curvature constraints
- Multi-robot coverage

---

## References
- H. Choset, *Coverage for Robotics – A Survey of Recent Results*
- J.-C. Latombe, *Robot Motion Planning*
- Boston Dynamics Spot SDK Documentation

## Author
Sabiq Khan (sabiq.khan@comcast.net)

