# rusko_summer

Gazebo Sim world representing the Ružomberok area (Slovakia) under summer conditions.

## Location

| Field | Value |
|---|---|
| Latitude | 48.7749° N |
| Longitude | 19.6503° E |
| Elevation | 446 m |
| Frame | ENU (East-North-Up) |
| Datum | WGS84 |

## Files

| File | Description |
|---|---|
| `model.sdf` | SDF 1.9 world definition |
| `rusko_summer_visual.glb` | Visual mesh (textured) |
| `rusko_summer_collision.glb` | Simplified collision mesh |
| `rusko_summer.mtl` | Material definitions |
| `rusko_summer.jpg` | Aerial/satellite reference image |

## Lighting

Simulates a clear summer day. Sun is placed at 500 m altitude with a warm neutral tone:

- Diffuse: `0.904 0.904 0.904`
- Specular: `0.271 0.271 0.271`
- Intensity: `1.0`
- Ambient scene: `0.4 0.4 0.4`

## Physics

- Solver: ODE
- Step size: 1 ms
- Real-time factor: 1.0 (uncapped update rate)

## Enabled Plugins

Physics, UserCommands, SceneBroadcaster, Contact, IMU, AirPressure, AirSpeed, ApplyLinkWrench, Magnetometer, NavSat, Sensors (ogre2)

## Usage

Set `GZ_SIM_RESOURCE_PATH` to include the parent `worlds/` directory, then reference the world in a launch file:

```python
world = os.path.join(worlds_dir, 'rusko_summer', 'model.sdf')
```

Or launch directly:

```bash
gz sim path/to/rusko_summer/model.sdf
```

## Notes

- The terrain mesh is offset to `z = -32.11 m` to align the ground surface with z = 0 in simulation space.
- A flat 2000 × 2000 m ground plane is included as a collision fallback beneath the mesh.
- Pair with `rusko_winter` for seasonal environment switching in the same geographic area.
