# Tutorials

Hands-on lessons that take you from a fresh machine to a robot balancing
itself — and then to running your *own* robot. Each tutorial is a short,
self-contained session you can follow top to bottom.

If you've never worked with control systems before, read
{doc}`../concepts` first (ten minutes, no math) so the words here land.

## The path

```{list-table}
:header-rows: 1
:widths: 6 30 64

* - #
  - Tutorial
  - What you'll do
* - 1
  - {doc}`Set up and see the robot <01_setup>`
  - Install everything and watch the example robot move in your browser.
* - 2
  - {doc}`Design your first controller <02_first_controller>`
  - Use the web app to turn the wobbly pendulum into a stable one — no code.
* - 3
  - {doc}`Read the response <03_reading_response>`
  - Learn what the animation and the plots are actually telling you.
* - 4
  - {doc}`Make it balance in simulation <04_run_in_gazebo>`
  - Run your controller on a physics-simulated robot in Gazebo and steer it.
* - 5
  - {doc}`Use your own robot <05_own_robot>`
  - Point the whole pipeline at a different URDF.
```

Do them in order the first time — each builds on the last. Tutorials 1–3
need only a laptop; tutorial 4 adds the Gazebo simulator.

```{toctree}
:hidden:
:maxdepth: 1

01_setup
02_first_controller
03_reading_response
04_run_in_gazebo
05_own_robot
```
