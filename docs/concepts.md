# The big ideas, in plain words

New to control systems? Start here. This page explains what Kontrol'Em does
using everyday language and one running picture — no equations, no jargon.
Once these ideas click, the rest of the documentation will make sense.

## The problem: balancing something that wants to fall

Try balancing a broom upright on the palm of your hand. The broom always
*wants* to topple. To keep it up you watch which way it's leaning and slide
your hand under it — constantly, in tiny corrections. Stop paying attention
for half a second and it falls.

The example robot in this project is exactly that problem, mechanised: a
**cart** that slides left and right on a rail, carrying **two pendulum
links** stacked on top of each other, and the goal is to keep them pointing
straight up.

```text
        ●   ← second link (top)
        |
        ●   ← first link
        |
     [=====]   ← cart, slides ⟷ on a rail
   ─────────────
```

Left alone it topples in about two seconds. The job of a **controller** is
to be the "hand": watch the links, and slide the cart just right to keep
everything balanced.

## What a controller actually is

A controller is a **feedback loop** that runs over and over, very fast
(here, 100 times a second):

1. **Sense** — where are the links right now, and how fast are they moving?
2. **Decide** — given that, which way should the cart push, and how hard?
3. **Act** — apply that push.
4. Repeat.

That's it. The clever part is step 2 — the "decide" rule. Kontrol'Em's job
is to *compute a good decide-rule for you*, automatically, from a
description of your robot.

## A few words you'll meet

```{glossary}
State
  The short list of numbers that fully describe the situation *right now*:
  the position of each joint and how fast each one is moving. If you know
  the state, you know everything you need to decide the next push.

Operating point (equilibrium)
  The pose you want to hold — here, "everything straight up." It's a
  balance point: perfectly still if undisturbed, but tippy, so the smallest
  nudge sends it falling. Controllers work by constantly pulling the robot
  back toward this point.

Linear model
  Close to the balance point, the physics is simple and predictable, so the
  math that describes it is easy to work with. Far from the balance point
  it stops being accurate — which is why a controller has a **validity
  region**, a "don't trust me past here" boundary.

Gain
  The heart of the decide-rule: a recipe that turns "how far off, and how
  fast drifting" into "how hard to push." Designing a controller mostly
  means computing a good gain.
```

## Four ways to design the decide-rule

Kontrol'Em can compute the decide-rule four different ways. You don't need
the theory to use them — here's the plain-language version of when each one
fits:

| Method | In one sentence | Reach for it when… |
|---|---|---|
| **LQR** | The clean baseline: assumes you can measure everything perfectly, then finds the smoothest way to stay balanced. | you're starting out, or you have good sensors on everything. |
| **LQG** | LQR's realistic cousin: works when you can only measure *some* things and the sensors are noisy, by intelligently estimating the rest. | you only have, say, position sensors — no direct speed measurement. |
| **H∞** | The pessimist: designed to survive the *worst-case* disturbance or a robot that isn't quite what the model says. | robustness matters more than squeezing out the last bit of performance. |
| **PID** | The classic three-knob controller most engineers already know. | you want something familiar and simple to reason about. |

If in doubt, start with **LQR**. Every tutorial here does.

## Design once, run forever

There are two very different moments in a controller's life, and Kontrol'Em
keeps them cleanly separated:

- **Design time** (slow, on your laptop, done once). Heavy math turns your
  robot's description into a decide-rule and saves it to a small file. This
  is where LQR/LQG/H∞ live.
- **Run time** (fast, on the robot, done forever). The robot just *reads
  that file* and runs the decide-rule 100 times a second. No heavy math on
  the robot — it only does quick arithmetic.

```text
   DESIGN TIME (once, laptop)              RUN TIME (forever, robot)
   ┌───────────────────────────┐          ┌──────────────────────────┐
   │ robot description (URDF)   │  small   │ read the file            │
   │        ↓ heavy math        │  file    │        ↓ fast arithmetic │
   │ a good decide-rule  ───────┼─────────▶│ push the cart, 100×/sec  │
   └───────────────────────────┘          └──────────────────────────┘
```

## The toolbox, mapped to the steps

Kontrol'Em is five tools. Here's what each one is *for*, in order:

| Tool | Plain-language job |
|---|---|
| **urdf_state_space** | Reads your robot's description file and turns it into the math a controller designer needs. |
| **state_space_control** | Takes that math and computes the decide-rule (LQR/LQG/H∞/PID). |
| **state_space_setup_assistant** | A point-and-click **web app** that does both of the above (and more) in your browser — the friendliest way in. |
| **state_space_response_viz** | Plays back a saved motion as an animation, so you can *watch* how the robot would respond. |
| **kontrolem_controllers** | Runs the finished decide-rule on a real or simulated robot (in Gazebo or Isaac Sim). |

You rarely touch all five directly. Most people live in the **web app** for
design and **kontrolem_controllers** for running it. The
{doc}`tutorials <tutorials/index>` walk you through exactly that path.

## Where to go next

- **Want to *do* something right now?** Jump to
  {doc}`Tutorial 1 <tutorials/01_setup>` — install it and watch the robot
  move in five minutes.
- **Want the fast command-line tour?** See the {doc}`quickstart`.
- **Curious how it's built?** The {doc}`architecture` page explains the
  design decisions.
