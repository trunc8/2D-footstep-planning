#!/usr/bin/env python

# trunc8 did this

# python libraries
import matplotlib.pyplot as plt
import numpy as np
import time

# increase default size matplotlib figures
from matplotlib import rcParams
rcParams["figure.figsize"] = (10, 5)

# drake imports
from pydrake.all import MathematicalProgram, OsqpSolver, eq, ge, le
from pydrake.solvers import MixedIntegerBranchAndBound


# Local file imports
from Terrain import Terrain
from utilities import animate_footstep_plan

from constraints import (
    add_decision_variables, 
    set_initial_and_goal_position,
    relative_position_limits,
    step_sequence,
    one_stone_per_foot,
    foot_in_stepping_stone
)
from cost import minimize_step_length

def footstep_planner(terrain, n_steps, step_span):
    # initialize optimization problem
    prog = MathematicalProgram()

    # optimization variables
    decision_variables = add_decision_variables(prog, terrain, n_steps)

    # constraints
    set_initial_and_goal_position(prog, terrain, decision_variables)
    relative_position_limits(prog, n_steps, step_span, decision_variables)
    step_sequence(prog, n_steps, step_span, decision_variables)
    one_stone_per_foot(prog, n_steps, decision_variables)
    foot_in_stepping_stone(prog, terrain, n_steps, decision_variables)

    # objective function
    minimize_step_length(prog, terrain, n_steps, decision_variables)

    # solve
    bb = MixedIntegerBranchAndBound(prog, OsqpSolver().solver_id())
    result = bb.Solve()

    # ensure that the problem is feasible
    if result != result.kSolutionFound:
        raise ValueError("Infeasible optimization problem.")

    # retrieve result of the optimization
    decision_variables_opt = [bb.GetSolution(v) for v in decision_variables]
    objective_opt = bb.GetOptimalCost()

    return decision_variables_opt, objective_opt

def generate_and_animate_footstep_plan(
    terrain, n_steps, step_span, title=None, to_save=False
):
    start_time = time.time()

    # run footstep planner
    decision_variables, objective = footstep_planner(
        terrain, n_steps, step_span
    )

    end_time = time.time()

    print(f"\n<< Finished solving footstep planning problem in {end_time-start_time:.2f}s\n")

    # animate result
    animate_footstep_plan(terrain, n_steps, step_span, *decision_variables[:2], title, f"n_steps={n_steps} step_span={step_span}m", to_save)


if __name__ == '__main__':
    print("\nWelcome to the 2D footstep planning solver!\n")

    # complete bridge
    terrain_A = Terrain([1, 1, 1, 1])

    # one stepping stone missing in the bridge
    terrain_B = Terrain([1, 1, 1, 0])

    # maximum number of steps to reach the goal
    # n_steps = 8
    n_steps = 18

    # side of the square that limits each step
    step_span = 0.8
    # step_span = 0.42

    to_save=False

    generate_and_animate_footstep_plan(terrain_A, n_steps, step_span, "Terrain A", to_save)
    generate_and_animate_footstep_plan(terrain_B, n_steps, step_span, "Terrain B", to_save)
