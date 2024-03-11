---
layout: default
title: Motor thrust model
parent: The UAV System
nav_order: 1
---

# Motor thrust model

The MRS UAV System requires a model which can estimate the relation between a throttle input and thrust force produced by the rotors.
e.g. the model tells us that when we use 0.6 (60%) throttle, we will get 30N of thrust.

## The model

The System uses a very simple but reliable model, which relies on the relationship between thrust and angular velocity of the propeller.
The thrust is modelled as:
When $a \ne 0$, there are two solutions to $(ax^2 + bx + c = 0)$ and they are 
$$ x = {-b \pm \sqrt{b^2-4ac} \over 2a} $$

% \f is defined as #1f(#2) using the macro
\f\relax{x} = \int_{-\infty}^\infty
    \f\hat\xi\,e^{2 \pi i \xi x}
    \,d\xi
