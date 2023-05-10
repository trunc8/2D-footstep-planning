#!/usr/bin/env python

# trunc8 did this

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle, Circle
from matplotlib.animation import FuncAnimation

# helper function that plots a rectangle with given center, width, and height
def plot_rectangle(center, width, height, ax=None, frame=0.1, **kwargs):
    # make black the default edgecolor
    if not "edgecolor" in kwargs:
        kwargs["edgecolor"] = "black"

    # make transparent the default facecolor
    if not "facecolor" in kwargs:
        kwargs["facecolor"] = "none"
    
    # get current plot axis if one is not given
    if ax is None:
        ax = plt.gca()

    # get corners
    c2c = np.array([width, height]) / 2
    bottom_left = center - c2c
    top_right = center + c2c

    # plot rectangle
    rect = Rectangle(bottom_left, width, height, **kwargs)
    ax.add_patch(rect)

    # scatter fake corners to update plot limits (bad looking but compact)
    ax.scatter(*bottom_left, s=0)
    ax.scatter(*top_right, s=0)

    # make axis scaling equal
    ax.set_aspect("equal")

    return rect

def animate_footstep_plan(
    terrain, n_steps, step_span, position_left, position_right, 
    title=None,
    experiment_attributes=None
):
    # initialize figure for animation
    fig, ax = plt.subplots()

    # plot stepping stones
    terrain.plot(title=title, ax=ax)

    # initial position of the feet
    left_foot = ax.scatter(0, 0, color="r", zorder=3, label="Left foot")
    right_foot = ax.scatter(0, 0, color="b", zorder=3, label="Right foot")

    # initial step limits
    left_limits = plot_rectangle(
        [0, 0],  # center
        step_span,  # width
        step_span,  # eight
        ax=ax,
        edgecolor="b",
        linestyle="-.",
        label="Left-foot limits",
    )
    right_limits = plot_rectangle(
        [0, 0],  # center
        step_span,  # width
        step_span,  # eight
        ax=ax,
        edgecolor="r",
        linestyle="-.",
        label="Right-foot limits",
    )
    facecolor = [0, 1, 0, 0.1]
    left_circ = Circle((0,0), 0.1, facecolor=facecolor)
    right_circ = Circle((0,0), 0.1, facecolor=facecolor)
    ax.add_patch(left_circ)
    ax.add_patch(right_circ)

    # misc settings
    ax.legend(loc="upper left", bbox_to_anchor=(0, 1.3), ncol=2)

    def animate(n_steps):
        # scatter feet
        left_foot.set_offsets(position_left[n_steps])
        right_foot.set_offsets(position_right[n_steps])
        left_circ.center = position_left[n_steps]
        right_circ.center = position_right[n_steps]

        # limits of reachable set for each foot
        c2c = np.ones(2) * step_span / 2
        right_limits.set_xy(position_left[n_steps] - c2c)
        left_limits.set_xy(position_right[n_steps] - c2c)

    # create ad display animation
    anim = FuncAnimation(fig, animate, frames=n_steps + 1, interval=8e2)
    # anim.save(f'../results/{title}_{experiment_attributes}.gif', writer='imagemagick', fps=1)
    plt.show()