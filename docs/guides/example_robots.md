# kontrolem_example_robots — example URDFs

Example robot URDFs used throughout the framework's documentation and
tests. Currently ships one robot; more example systems can be added under
`urdf/` and referenced the same way.

## cart_double_inverted_pendulum

A cart (prismatic, actuated) carrying two pendulum links (`joint1`
revolute, actuated; `joint2` revolute, passive):

```text
package://kontrolem_example_robots/urdf/cart_double_inverted_pendulum.urdf
```

| Joint | Type | Actuated | Notes |
|---|---|---|---|
| `cart_joint` | prismatic | yes | horizontal cart position, ±2 m |
| `joint1` | revolute | yes | first pendulum link |
| `joint2` | revolute | no | second pendulum link (passive) |

At `q = 0` both links point straight up — the classic **unstable upright
equilibrium**. It was chosen as the running example because it exercises
the framework's interesting paths:

- **Underactuated** (3 DoF, 2 actuators) yet controllable and observable —
  actuation selection matters.
- The upright pose needs **zero torque on the passive joint** by
  construction, so it linearizes with no equilibrium warning and is a
  convenient zero-effort seed for the wizard's automatic equilibrium
  finder.
- The open loop has two right-half-plane poles — stabilization is a real
  demo, and the closed-loop response animation actually shows something.

Try it end to end in the {doc}`../quickstart`.
