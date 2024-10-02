# Safety-enhanced UAV Path Planning with Spherical Vector-based Particle Swarm Optimization

This repository contains the MATLAB implementation of the Spherical Vector-based Particle Swarm Optimization (SPSO) algorithm for safety-enhanced path planning of unmanned aerial vehicles (UAVs) in complicated environments.

## Overview

This project presents the Spherical Vector-based Particle Swarm Optimization (SPSO), an algorithm designed to plan safe and efficient flight paths for unmanned aerial vehicles (UAVs) in complex environments. Think of it like creating a smart GPS for drones that avoids obstacles and finds the best route.

### The SPSO Advantage

SPSO stands out because it:

1. **Mimics Nature**: It uses a flocking-inspired approach where potential paths work together to find the optimal route.
2. **Ensures Safety**: It keeps UAVs clear of danger zones by adhering to safety constraints.
3. **Optimizes Routes**: It balances exploring new paths with refining existing ones for the shortest, most efficient flight plan.

### Simplifying Complexity

In simpler terms, if you're planning a road trip and want to avoid traffic while taking the quickest route, SPSO does the same for drones—but in three-dimensional space and with more obstacles. It's about making smart decisions based on a collective 'brainpower' to find the best path possible.

### Key Benefits

- **Safe**: Prioritizes UAV safety by avoiding no-fly zones and other threats.
- **Efficient**: Saves time and energy with the shortest, most direct routes.
- **Effective**: Performs well in tough, real-world scenarios compared to other methods.

SPSO brings a smart, nature-inspired approach to UAV path planning, making flights safer and more efficient.

## Getting Started

### Dependencies

- MATLAB

### Installation

1. Clone the repository:

   https://github.com/hadliner/Safety-enhanced-UAV-Path-Planning-with-Spherical-Vector-based-Particle-Swarm.git


### Usage

1. Open MATLAB and load the project.
2. Run the `main.m` script to execute the SPSO algorithm and visualize the results.

## Algorithm Description

The SPSO algorithm encodes each path as a set of spherical vectors, each representing the movement of the UAV from one waypoint to another. It searches for the optimal path by efficiently exploring the configuration space of the UAV.

## Results

The algorithm generates the best path that minimizes the cost function while satisfying the constraints. The results are visualized using MATLAB plots.

Author
Manh Duong Phung Quang Phuc Ha
