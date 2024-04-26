# Tarjan's Bridge Detection Algorithm with Storm Simulation
By Andrew Jop & Chris LaDuke
## Overview

This README provides an overview of how Tarjan's bridge detection algorithm is implemented and extended with a storm simulation to determine the number of homes affected after a storm. 

## Tarjan's Bridge Detection Algorithm

Tarjan's bridge detection algorithm is used to find bridges in a graph. A bridge is an edge in a graph whose removal disconnects the graph. The algorithm works by performing a depth-first search (DFS) and assigning to each vertex the earliest reachable ancestor in the DFS tree. An edge (u, v) is a bridge if and only if there is no back edge or cross edge from v or any of its descendants to an ancestor of u, i.e., if the earliest reachable ancestor of v is u, then (u, v) is a bridge.

## Storm Simulation

To simulate the impact of a storm on a graph representing a network of homes, we extend Tarjan's bridge detection algorithm with the following steps:

1. **Remove Roads**: First, we identify and remove roads from the graph using Tarjan's algorithm. Bridges represent critical connections that, if broken, would disconnect subsets of the graph.

2. **Storm Impact Analysis**: After removing roads, we simulate the storm's impact by considering each remaining edge in the graph. If an bridge is destroyed due to the storm (e.g., due to fallen trees or damaged infrastructure), we count the number of homes affected.

3. **Output**: Once a bridge is destroyed, the simulation is terminated, and the impact analysis can be used by emergency response teams and city planners to assess the impact of the storm on the community.

## Usage

To use the bridge detection algorithm with storm simulation:

1. Compile the Java program provided (`HighwayGraph.java`).
2. Run the program using the following command:
   
   ```bash
   java HighwayGraph METALfiles/nameOfFile
## Graphs

Graphs are copyright © [James D. Teresco](https://j.teresco.org/), generated from highway data gathered and maintained by [Travel Mapping Project](https://travelmapping.net/credits.php#contributors) contributors. Graphs may be downloaded freely for academic use. Other use is by written permission only.
