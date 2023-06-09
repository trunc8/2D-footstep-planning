#!/usr/bin/env python

# trunc8 did this

import numpy as np
from pydrake.all import eq, ge, le

def add_decision_variables(prog, terrain, n_steps):
    # number of stones in the terrain
    n_stones = len(terrain.stepping_stones)

    # position of each foot at each step
    position_left = prog.NewContinuousVariables(rows=n_steps + 1, cols=2)
    position_right = prog.NewContinuousVariables(rows=n_steps + 1, cols=2)

    # binaries that assign feet to stones for each step
    stone_left = prog.NewBinaryVariables(rows=n_steps + 1, cols=n_stones)
    stone_right = prog.NewBinaryVariables(rows=n_steps + 1, cols=n_stones)

    # # binaries that trim the number of footsteps
    # trim = prog.NewBinaryVariables(n_steps)

    return position_left, position_right, stone_left, stone_right#, trim

def set_initial_and_goal_position(prog, terrain, decision_variables):
    # unpack only decision variables needed in this function
    position_left, position_right = decision_variables[:2]

    # initial position of the feet of the robot
    foot_offset = np.array([0, 0.2])
    center = terrain.get_stone_by_name("initial").center
    initial_position_left = center
    initial_position_right = center - foot_offset

    # enforce initial position of the feet
    prog.AddLinearConstraint(eq(position_left[0], initial_position_left))
    prog.AddLinearConstraint(eq(position_right[0], initial_position_right))

def relative_position_limits(prog, n_steps, step_span, decision_variables):
    # unpack only decision variables needed in this function
    position_left, position_right = decision_variables[:2]

    for t in range(n_steps+1):
        prog.AddLinearConstraint(le(position_right[t], position_left[t] + np.array([step_span/2, step_span/2]) ))
        prog.AddLinearConstraint(ge(position_right[t], position_left[t] - np.array([step_span/2, step_span/2]) ))

def step_sequence(prog, n_steps, step_span, decision_variables):
    '''
    Assuming that first stepping with left foot at t=0
    '''
    # unpack only decision variables needed in this function
    position_left, position_right = decision_variables[:2]

    # note that the step_span coincides with the maximum distance
    # (both horizontal and vertical) between the position of
    # a foot at step t and at step t + 1
    step_limit = np.ones(2) * step_span

    # sequence for the robot steps
    for t in range(n_steps):
        # lengths of the steps
        step_left = position_left[t + 1] - position_left[t]
        step_right = position_right[t + 1] - position_right[t]

        limit_left = step_limit*(t % 2 == 0) # even steps
        limit_right = step_limit*(t % 2 != 0) # odd steps

        # constraints on left-foot relative position
        prog.AddLinearConstraint(le(step_left, limit_left))
        prog.AddLinearConstraint(ge(step_left, -limit_left))

        # constraints on right-foot relative position
        prog.AddLinearConstraint(le(step_right, limit_right))
        prog.AddLinearConstraint(ge(step_right, -limit_right))

def one_stone_per_foot(prog, n_steps, decision_variables):
    # unpack only decision variables needed in this function
    stone_left, stone_right = decision_variables[2:4]

    for t in range(n_steps+1):
        prog.AddLinearConstraint(eq(np.sum(stone_left[t]), np.array([1])))
        prog.AddLinearConstraint(eq(np.sum(stone_right[t]), np.array([1])))

# parameter for the big-M method
# carefully chosen for the terrain above
def get_big_M(terrain):
    # big-M parameter for the horizontal axis
    initial = terrain.get_stone_by_name("initial")
    goal = terrain.get_stone_by_name("goal")
    M = [goal.center[0] - initial.center[0]]

    # big-M parameter for the vertical axis
    lateral = terrain.get_stone_by_name("lateral")
    M.append(lateral.top_right[1] - initial.center[1])

    return np.array(M * 2)

def foot_in_stepping_stone(prog, terrain, n_steps, decision_variables):
    # unpack only decision variables needed in this function
    (
        position_left,
        position_right,
        stone_left,
        stone_right,
    ) = decision_variables[:4]

    # big-M vector
    M = get_big_M(terrain)

    for t in range(n_steps+1):
        for i in range(len(terrain.stepping_stones)):
            prog.AddLinearConstraint(
                le(
                    terrain.stepping_stones[i].A.dot(position_left[t]),
                    terrain.stepping_stones[i].b + (1-stone_left[t,i])*M
                )
            )
            prog.AddLinearConstraint(
                le(
                    terrain.stepping_stones[i].A.dot(position_right[t]),
                    terrain.stepping_stones[i].b + (1-stone_right[t,i])*M
                )
            )

# def trim_time_steps(prog, terrain, n_steps, decision_variables):
#     # unpack only decision variables needed in this function
#     position_left, position_right = decision_variables[:2]
#     trim = decision_variables[4]

#     A = np.array([[1, 0], [0, 1], [-1, 0], [0, -1]])

#     # initial position of the feet of the robot
#     foot_offset = np.array([0, 0.2])
#     center = terrain.get_stone_by_name("initial").center
#     initial_position_left = center
#     initial_position_right = center - foot_offset

#     M = get_big_M(terrain)

#     for t in range(1, n_steps+1):        
#         prog.AddLinearConstraint(
#             le(
#                 A.dot(position_left[t]),
#                 np.concatenate([initial_position_left]*2) + (1-trim[t-1])*M
#             )
#         )
#         prog.AddLinearConstraint(
#             le(
#                 A.dot(position_right[t]),
#                 np.concatenate([initial_position_right]*2) + (1-trim[t-1])*M
#             )
#         )
