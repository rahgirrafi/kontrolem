# Tutorial 3 — Read the response

**Goal:** understand what the **Response** screen (step 6 of the web app) is
telling you, so you can judge whether a controller is *good*, not just
*stable*.

**Time:** about 10 minutes, all reading and clicking — nothing to install.

**Before you start:** have the app open on the **Response** step from
{doc}`Tutorial 2 <02_first_controller>`, with a simulated response showing.

## The three things on screen

The Response screen shows one event — the robot getting a shove and
recovering — three ways at once, all locked to the **same clock**:

1. **The 3D animation** — the robot itself, moving.
2. **The joint-angle plots** — how far each joint is from upright, over time.
3. **The control-effort plot** — how hard the cart is being pushed, over time.

A moving vertical line (the "time cursor") sweeps all the plots together as
the animation plays, so you can line up *"the cart shoved hard right here"*
with *"because the top link was falling right there."*

## Play with the transport bar

Use the controls under the animation:

- **Play / Pause** — start and stop.
- **Speed** (0.25× to 2×) — slow it down to 0.25× to really see the
  correction happen.
- **Scrub** — drag the bar (or drag directly on a plot) to jump to any
  moment.

**Try this:** set speed to 0.25×, play, and watch how the cart moves
*opposite* to the way the links are falling — sliding under them to catch
them. That's the feedback loop from {doc}`../concepts` in action.

## Reading the joint plots — is it a *good* controller?

A stable controller brings every joint angle back to zero (upright). But
*how* it gets there is what separates good from bad:

```{list-table}
:header-rows: 1
:widths: 30 70

* - What you see
  - What it means
* - Curves settle back to zero quickly and smoothly
  - A well-behaved controller. 👍
* - Big overshoot — the angle swings way past zero before coming back
  - Twitchy; it over-corrects. Often the price of reacting very fast.
* - Slow, lazy return to zero
  - Sluggish; safe but not snappy.
* - Curves grow instead of shrinking
  - Unstable — it's falling over. (You won't see this for LQR here, but you
    will if you feed in a bad design.)
```

There's a natural trade-off: react harder and you settle faster **but** use
more effort and risk overshoot. Reading these plots is how you feel out that
trade-off.

## Reading the effort plot

This is how hard the cart motor works. A controller that balances perfectly
but demands impossible force isn't useful on a real robot. Glance here to
sanity-check that the pushes are reasonable, not enormous spikes.

## Honesty chips — when *not* to trust the picture

Kontrol'Em is deliberately honest about the limits of its own math. If the
shove is big enough to push the robot far from the balance point, the simple
"linear model" stops being accurate — and rather than hide that, the app
flags it with **chips** and shaded bands on the plots:

```{list-table}
:header-rows: 1
:widths: 30 70

* - Flag
  - Meaning
* - **linear-validity**
  - A joint swung so far from upright that the linear model is no longer
    trustworthy there. Treat that span with suspicion.
* - **limit-violation**
  - The motion passed a joint's physical limit. (The simulation doesn't
    clamp it — it shows you the truth and flags it.)
* - **instability**
  - The closed loop is actually unstable; the run was time-capped before the
    numbers blew up.
* - **settling time / overshoot**
  - Handy landmarks marking when the response settled and how far it
    overshot.
```

Seeing a linear-validity chip doesn't mean your controller is broken — it
means *this particular shove* was large enough to leave the region the design
is guaranteed for. On a real robot, the {doc}`validity guard
<../runtime/getting_started>` watches for exactly this and reacts safely.

## Save the motion for a bigger replay (optional)

The Response step saves the full motion as a `trajectory.npz` file. You can
replay it full-screen in RViz:

```bash
ros2 launch state_space_response_viz view_response.launch.py \
    trajectory:=trajectory.npz \
    urdf:=install/kontrolem_example_robots/share/kontrolem_example_robots/urdf/cart_double_inverted_pendulum.urdf \
    fixed_frame:=world
```

## What you just learned

You can now look at a response and say more than "it works" — you can tell
*fast vs sluggish*, *smooth vs twitchy*, and *trustworthy vs off-the-map*.
That's the judgment you'll use when picking between controllers.

---

**Next:** {doc}`Tutorial 4 — Make it balance in simulation <04_run_in_gazebo>`,
where the controller leaves the design app and runs on a physics-simulated
robot.
