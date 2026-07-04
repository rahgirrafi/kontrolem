# Runtime & deployment

`kontrolem_controllers` is the **runtime consumer** of the Kontrol'Em design
pipeline: **chainable `ros2_control` controller plugins** that load
linear state-space controllers (LQR, LQG, H∞) exported by
`state_space_setup_assistant` and run them on a robot. No synthesis happens at
runtime — just realtime-safe matrix–vector math on gains computed offline.

| Design-time (Kontrol'Em, Python) | Runtime (this repo, C++)              |
| -------------------------------- | ------------------------------------- |
| URDF → LTI plant → Riccati / H∞  | load `<name>_ros2_control.yaml`       |
| runs once, offline               | runs at the controller rate, on-robot |

The bundle carries the operating point and the gains; the controllers rebuild
the deviation state `x = [q − q_eq, q̇]`, apply the law, and write effort
commands. See the [export contract](export_contract.md) for the exact schema
and conventions — it is the single source of truth shared by the exporter and
these loaders.

## Packages

```{list-table}
:header-rows: 1

* - Package
  - Description
* - `kontrolem_controllers`
  - umbrella metapackage
* - `kontrolem_controllers_base`
  - shared library: artifact loader, static-gain & dynamic-compensator cores (Tustin discretization), validity guard, chainable controller base
* - `lqr_controller`
  - static state feedback `u = u_eq − K·x`
* - `lqg_controller`
  - LQG output-feedback compensator (Kalman + LQR as one LTI), runs from position-only measurements
* - `hinf_controller`
  - H∞ output-feedback compensator (`hinf` / `hinf_mixsyn` designs)
```

```{toctree}
:maxdepth: 2
:caption: Contents

getting_started
export_contract
controllers/index
gazebo
isaac
```
