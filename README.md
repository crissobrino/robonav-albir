# Embedded Vision Navigation on OpenMV

**Cristina Sobrino**

---

## Summary

This project investigates visually guided navigation on an OpenMV-based mobile robot.
The work integrates onboard colour segmentation, closed-loop motor control, and local map construction to enable autonomous exploration in visually structured environments.

Rather than relying on external localisation or global planning, navigation emerges from reactive sensorimotor loops driven entirely by embedded vision.

---

## System Overview

### Visual Tracking

Colour-based blob detection is used to stabilise gaze and steering relative to dynamic visual targets.
Frequency-domain characterisation of the tracking loop informed controller tuning, exposing trade-offs between responsiveness and smooth pursuit.

---

### Reactive Navigation

Lane following and obstacle negotiation behaviours are implemented using monocular perception.
Obstacle distance is inferred from camera geometry, allowing the robot to stop and orient relative to targets using only onboard sensing.

---

### Mapping and Exploration

A grid-based internal representation is incrementally constructed from sequential observations.
The navigation stack combines:

* blob-based landmark perception
* PID-controlled gaze and steering
* uncertainty-driven exploration heuristics
* lightweight recursive path selection for revisiting informative regions

The resulting behaviour allows the robot to discover environment boundaries, prioritise unexplored regions, and reach a visual goal without prior map knowledge.

---

## Implementation

The system runs entirely on an OpenMV microcontroller, leveraging:

* real-time blob detection and colour thresholds
* servo-based differential drive control
* embedded PID controllers for perception-action alignment

Core navigation logic is implemented in `mapSolver`, which manages perception, control, and mapping within a unified behavioural loop.


