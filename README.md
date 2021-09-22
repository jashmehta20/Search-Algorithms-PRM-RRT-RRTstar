# RBE 550 - Standard Search Algorithms Implementation

## Overview

Implementation of Sampling based algorithms **Probalistic Road Maps (PRM)**, **Rapidly Explroting Random Tree (RRT)** and **RRT***. 

4 different sampling methods are coded for **PRM**, namely - **uniform sampling**, **random sampling**, **gaussian sampling** and **bridge sampling**.


Files included:

**PRM.py** includes a PRM class with four different sampling methods.

**RRT.py** includes a RRT class for RRT and RRT*.

**main.py** is the script that provides helper functions that load the map from an image and call the classes and functions from **PRM.py** and **RRT.py**.

**WPI_map.jpg** is a binary WPI map image with school buildings. You could replace it with some other maps you prefer.

## Instruction to Run the Code 
- Gitclone the repository.
- Add a new binary image file as a map and updated the start and goal coordinates for path planning.
- Run the main.py script to see the plotted results of all 4 PRM sampling methods, RRT and RRT*.

