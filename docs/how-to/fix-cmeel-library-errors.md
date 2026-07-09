# How to fix the cmeel / libboost load error

> **For:** anyone hitting a Pinocchio/Boost library error while building or running. **Assumes:** you installed Pinocchio via the pip cmeel wheel. **Goal:** make the loader find the cmeel libraries.

## The symptom

At launch or when running a test, you see:

```
LibraryLoadException: ... libboost_serialization.so.1.90.0: cannot open shared object file: No such file or directory
```

or, from a test binary:

```
error while loading shared libraries: libboost_serialization.so.1.90.0: cannot open shared object file
```

## The cause

Kontrol'Em links Pinocchio from the pip **cmeel** wheel, which bundles its *own* Boost under `…/cmeel.prefix/lib`. Those libraries are not on the default loader path, so anything that `dlopen`s a Kontrol'Em plugin — `ros2_control_node`, or a test binary — can't find them.

## Fix 1 — launching a demo: nothing to do

The bundled launch files auto-detect the cmeel prefix and inject it for the launched nodes. If you launch with `ros2 launch kontrolem_bringup …`, this error should not occur. If it does, confirm the prefix exists:

```bash
ls $HOME/.local/lib/python3.10/site-packages/cmeel.prefix/lib
```

If that directory is missing, reinstall Pinocchio: `pip3 install pin`.

## Fix 2 — running a node or test by hand: export the path

When you run a binary directly (a unit test, or `ros2_control_node` outside the provided launch), set the loader path yourself:

```bash
P=$HOME/.local/lib/python3.10/site-packages/cmeel.prefix
export LD_LIBRARY_PATH="$P/lib:$P/lib64:$LD_LIBRARY_PATH"
```

Then run your command in the same shell.

## Fix 3 — building: put the prefix on `CMAKE_PREFIX_PATH`

If the error appears at **build** time (CMake can't find Pinocchio), pass the prefix to colcon:

```bash
P=$HOME/.local/lib/python3.10/site-packages/cmeel.prefix
colcon build --cmake-args -DCMAKE_PREFIX_PATH="$P" -DCMAKE_BUILD_TYPE=Release
```

## Why `colcon test` shows this even when everything is fine

`colcon test` spawns the test binaries in a clean environment that does **not** carry your `LD_LIBRARY_PATH`, so every test appears to fail with this error. That is an environment artifact, not a real failure — run the test binaries directly (Fix 2), as described in [Verify an integration](verify-an-integration.md).

> The background on why the project uses the cmeel wheel at all is in [Explanation → Design decisions](../explanation/design-decisions.md) and the exact paths are in [Reference → Build & environment](../reference/build-and-environment.md).
