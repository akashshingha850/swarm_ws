# rusko_winter

Gazebo Sim world representing the Ružomberok area (Slovakia) under winter conditions.

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
| `rusko_winter_visual.glb` | Visual mesh (textured, snow-covered) |
| `rusko_winter_collision.glb` | Simplified collision mesh |
| `rusko_winter.mtl` | Material definitions |
| `rusko_winter.jpg` | Aerial/satellite reference image |

## Lighting

Simulates overcast winter conditions. Sun intensity is reduced and the color is shifted cool-blue:

- Diffuse: `0.8 0.85 0.9`
- Specular: `0.3 0.35 0.4`
- Intensity: `0.8`
- Ambient scene: `0.5 0.55 0.6`

## Physics

- Solver: ODE
- Step size: 1 ms
- Real-time factor: 1.0 (uncapped update rate)

## Enabled Plugins

Physics, UserCommands, SceneBroadcaster, Contact, IMU, AirPressure, AirSpeed, ApplyLinkWrench, Magnetometer, NavSat, Sensors (ogre2)

## Usage

Set `GZ_SIM_RESOURCE_PATH` to include the parent `worlds/` directory, then reference the world in a launch file:

```python
world = os.path.join(worlds_dir, 'rusko_winter', 'model.sdf')
```

Or launch directly:

```bash
gz sim path/to/rusko_winter/model.sdf
```

## Notes

- The terrain mesh is placed at `z = 0` (no vertical offset required for this variant).
- A flat 2000 × 2000 m ground plane is included as a collision fallback beneath the mesh.
- Pair with `rusko_summer` for seasonal environment switching in the same geographic area.
