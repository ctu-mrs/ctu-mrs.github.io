---
layout: default
title: Motor thrust model
parent: The UAV System
nav_order: 1
---

# Motor thrust model

The MRS UAV System requires a model which can estimate the relation between a throttle input and thrust force produced by the rotors.
e.g. we need a model which tells us that when we use 0.6 (60%) throttle, we will get 30N of thrust.

## The model

The System uses a very simple but reliable model, which relies on the relationship between thrust and angular velocity of the propeller, where the thrust is proportional to the angular velocity squared.
We therefore model the relationship between throttle and thrust as a quadratic curve.
The thrust is modelled as:

$$ T = a\sqrt{f} + b $$

Where $$ T $$ is throttle, $$ a $$ and $$ b $$ are the parameters of the quadratic curve and $$ f $$ is the produced force.
We can substitute $$ f = m*g $$ and invert the equation, to get an estimate of the current UAV mass based on the used throttle:

$$ m=\frac{1}{g}\left ( \frac{T-b}{a} \right )^{2} $$

This mass estimate is used for landing detection, and can be used for other purposes, like confirmation of payload attachment/release based on change in the mass of the UAV.
