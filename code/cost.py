#!/usr/bin/env python

# trunc8 did this

import numpy as np

def minimize_step_length(prog, n_steps, decision_variables):
    # unpack only decision variables needed in this function
    position_left, position_right = decision_variables[:2]

    for t in range(n_steps):
        prog.AddQuadraticCost(
            (position_left[t + 1] - position_left[t]).dot(position_left[t + 1] - position_left[t])
        )
        prog.AddQuadraticCost(
            (position_right[t + 1] - position_right[t]).dot(position_right[t + 1] - position_right[t])
        )
        
    