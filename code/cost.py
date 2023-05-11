#!/usr/bin/env python

# trunc8 did this

import numpy as np

def minimize_step_length(prog, terrain, n_steps, decision_variables):
    # unpack only decision variables needed in this function
    position_left, position_right = decision_variables[:2]

    # goal position of the feet of the robot
    foot_offset = np.array([0, 0.2])
    center = terrain.get_stone_by_name("goal").center
    goal_position_left = center
    goal_position_right = center - foot_offset

    Qg = 100

    prog.AddQuadraticCost(
        Qg*(position_left[-1] - goal_position_left).dot(position_left[-1] - goal_position_left)
    )

    prog.AddQuadraticCost(
        Qg*(position_right[-1] - goal_position_right).dot(position_right[-1] - goal_position_right)
    )

    Qr = 1

    for t in range(n_steps):
        prog.AddQuadraticCost(
            Qr*(position_left[t + 1] - position_left[t]).dot(position_left[t + 1] - position_left[t])
        )
        prog.AddQuadraticCost(
            Qr*(position_right[t + 1] - position_right[t]).dot(position_right[t + 1] - position_right[t])
        )
        
    