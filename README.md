# Sapphire

A 2D physics and thermodynamics sandbox (Qt 6, C++17).

## Build and run

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/sapphire
```

You need Qt 6 (Widgets) and CMake 3.16 or newer. To build only the physics core and its tests (no Qt), pass `-DSAPPHIRE_BUILD_APP=OFF`.

## Tests

```sh
./build/sapphire_tests                                  # physics core, no Qt needed
QT_QPA_PLATFORM=offscreen ./build/sapphire_ui_smoke     # drives the real window headlessly
ctest --test-dir build                                  # both
```

`sapphire_ui_smoke` needs the Qt Test module. If that module is missing, the target is skipped.

## Layout

| Path | What it holds |
|---|---|
| `src/core/` | The simulation. Plain C++17 with no Qt, so it can be unit-tested. |
| `vector.h`, `random.h`, `thermo.h` | Math, random numbers, and the material models: boiling point, water density and viscosity, rubber restitution. |
| `physics_object.*` | One body: its mechanical state, rotation, temperature, phase and latent heat. |
| `constraints.*` | Springs, rigid links, pulleys and gas struts. |
| `gas_container.h` | The sealed vessel with its piston. |
| `spatial_hash.*` | The broad phase (uniform grid). |
| `world.*` | The step loop, solver, heat transfer, water, ice and gas. |
| `serialization.cpp` | The scene file format, also used for undo. |
| `src/app/` | The Qt view: `physics_canvas.*` for rendering, input, history and the fixed timestep; `main_window.*` for the toolbox, sliders and shortcuts. |
| `tests/` | `core_tests.cpp` and `ui_smoke.cpp`. |

## Controls

- **Left-click:** spawn the selected object. With a link tool (spring, pulley, piston), drag from one object to another.
- **Drag** an object to move it. Flick and release to throw it. Pins, heaters and coolers can be moved too.
- **Right-click** an object to remove it.
- **Draw Shape:** click to add points. Click the first point again, or right-click, to close the shape.
- **Hover** over anything to see its temperature, phase, boiling point, speed, spin and bounce. Hovering a vessel's piston shows the gas pressure and volume.
- **Shortcuts:**
  - `Ctrl+Z` undo
  - `Ctrl+Shift+Z` or `Ctrl+Y` redo
  - `Ctrl+S` save
  - `Ctrl+O` open
  - `Space` pause

Scenes save as plain text (`*.sapphire`). The file stores the sliders' values as well.

## How the simulation works

- **Fixed timestep.** `World::step()` always advances 1/60 s. The UI runs as many steps as real time requires, up to 4 per tick. Results therefore don't depend on the display's frame rate.
- **Substepped position-based dynamics.** Based on Müller et al., 2020. Each frame runs 6 substeps, and each substep does this:
  1. Integrate forces into predicted positions.
  2. Solve links, contacts and walls.
  3. Derive velocities from the change in position.
  4. Apply restitution and Coulomb friction in a velocity pass.
- **Rotation.** Circles have angle and spin, and friction at contacts makes them roll. Motors apply torque up to a target speed, which is why the car drives.
- **Water.** SPH-like pressure and viscosity, plus a short-range core that stops particles from passing through each other. Rigid bodies push water aside and feel Archimedes' force from the water level next to them, plus drag. Freezing water bonds into rigid floes. The buoyancy of a floe comes from its waterline, so ice floats mostly submerged.
- **Gas.** A Langevin thermostat sets each particle's speed from its temperature. Motion well above thermal speed, such as from a blast, is turned into heat.

## Known limitations

- Boxes collide as circles of the same radius, and they don't rotate.
- The particle water is slightly compressible under its own weight.
- Performance on a heavy test scene of about 2,300 particles: roughly 12 ms per frame of physics and 7 ms of rendering, on one core.
