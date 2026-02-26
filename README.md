# Risk-Aware Global Planner for ROS2 Navigation2

## Project Objective
Implement a Nav2 global planner that prioritizes safer paths instead of only shortest paths.

## Algorithm
Risk = Σ(costmap inflation cost²)

Score = Risk + λ|offset|

## Experimental Scenarios
1. Open space
2. Obstacle blocking path
3. Dense obstacle cluster
4. Left vs right decision

## Demo Videos

Navigation experiments for the risk-aware planner can be viewed here:

Google Drive Folder:
(https://drive.google.com/drive/folders/1Tlz7hgiJLgKio-HTtbukOiAE_I9ZCi3z?usp=sharing)
