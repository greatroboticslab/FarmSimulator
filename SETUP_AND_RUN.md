# FarmSimulator Setup and Run Guide

## Requirements

- Unity Hub with Unity `2022.3.8f1`.
- Git LFS installed before cloning or pulling this repo.
- Windows is the currently verified local development target.
- Optional: Python 3.10 and ML-Agents for training workflows.

## Open the Project

1. Open Unity Hub.
2. Add this project folder.
3. Use Unity `2022.3.8f1`.
4. Open `Assets/Scenes/SampleScene.unity`.

`SampleScene` is also listed in Unity build settings so the project has a clear default scene.

## Local Play Smoke Test

1. Press Play.
2. Click `Select Robot`.
3. Pick a rover.
4. In the location dropdown, choose a farm.
5. Press `Go to Farm`.
6. Wait a few seconds for the map and robot startup flow.

Expected result: the selected farm/map prefab loads, the selected robot starts, and the Console stays free of red startup errors.

Configured dropdown farms:

- `MTSU Farms`: loads the dedicated `Assets/Maps/MTSU Farm.prefab`.
- `Farm in the City`: loads the dedicated `Assets/Subscenes/farmInTheCity.prefab`.
- `Hutchinson Farms`, `Freethought Farm`, and `Williamson Family Farm`: load the generic coordinate-backed `Assets/Maps/StartMap.prefab`.

## Farm Terrain

The original RealWorldTerrain data was never committed, so the farm
prefabs used to load with no ground at all. The ground is now a
generated farmland terrain (heightmap, grass/dirt/soil layers, waving
grass details, collider) stored under `Assets/Terrain` and installed
into the MTSU Farm and StartMap prefabs. To regenerate or tweak it,
edit `Assets/Editor/FarmTerrainBuilder.cs` and run
`Tools > Build Farm Terrain`. Scene lighting, fog, and the skybox are
applied by `Tools > Polish Scene Lighting`.

## Playing a Farming Loop

1. Press Play, pick a robot, and press `Go to Farm`.
2. Press `Farm Building`, then click and drag on the field to stake a
   plot. Crops (soybeans by default), a farmhand, and path waypoints
   spawn when you release. Press `Farm Building` again to exit.
3. Type `start farming` into the chat box (needs a Groq key, see
   below) or tick the `Self Driving` toggle. The rover moves to the
   row start and works the waypoints.

Robot status: all nine select-screen robots spawn on the farm spawn
pad and respond to input. The `Humanoid Training` entry is for
ML-Agents training and ragdolls unless a trainer is attached.

## Optional Services

The simulator can integrate with lab/network services, but local play should not require them.

- MQTT is off by default. Enable `MqNode.enableMqtt` on a robot only when a broker is running and configured.
- ROS publishing is off by default. Enable `ROSRover.enableRos` only when the ROS TCP Connector setup is ready.
- Robot chat is optional. To enable Groq chat, get a free API key from https://console.groq.com/keys, then either set the `GROQ_API_KEY` environment variable or create `Assets/StreamingAssets/config.json` (gitignored) with:

```json
{
  "groq_api_key": "your_key_here"
}
```

Never commit a real key. A key committed to a public repo gets detected and revoked automatically, which is how the chat broke before.

- Online map elevation keys are optional. The old Bing elevation component on `StartMap` is disabled so local play does not fail on an expired key.

## ML-Agents

From the repo root, activate the Python environment you use for ML-Agents, then run:

```powershell
mlagents-learn --help
```

For training, use the existing humanoid training configuration and the in-game `Humanoid (Training)` selection described in `README.md`.

## Troubleshooting

- If Unity says the project is locked, close duplicate Unity processes and reopen through Unity Hub.
- If a farm does not load, confirm its dropdown name matches `Assets/Data/locations.csv` and the matching entry in `PathMaker.subscenes`.
- If chat says the API is not configured, that is expected unless a Groq key is provided via `Assets/StreamingAssets/config.json` or `GROQ_API_KEY`. If it says the key is invalid or revoked, generate a new key at https://console.groq.com/keys.
- If MQTT or ROS messages are needed for lab hardware, explicitly enable the relevant component in the robot prefab and start the matching external service first.

## Performance on Low-End Machines

The runtime side of the graphics tiers lives in `Assets/Scripts/Interface/GraphicsSettings.cs`. One table at the top of the file maps each quality level (0 = Very Low ... 5 = Ultra) to render height, post-processing, HDR, shadow distance, terrain grass distance and density, terrain mesh detail, view distance, frame cap, decorative prop fraction, animated-prop cap and picture-in-picture camera rate. `ProjectSettings/QualitySettings.asset` holds the Unity-side values per level (shadow type and resolution, cascades, MSAA, texture mip limit, LOD bias, vsync). Change a tier in those two places only.

What the tier system relies on elsewhere:

- `RobotCameraThrottle` is attached at runtime to every camera that renders into a render texture (each robot prefab has one, some have two). It disables the camera and renders it by hand only while the picture-in-picture panel is visible or the socket feed is on, at the tier's rate, without post-processing and with reduced shadows on the low tiers. Recording still works because `RobotCamera.CaptureFrame` renders explicitly.
- `ObjectPlacer` scales the farm's decorative props by the tier, caps animated people, and spawns over several frames instead of freezing the game.
- `Plant` no longer has a per-plant `Update`; only plants inside a UV lamp trigger are ticked, from `GraphicsSettings`.
- Grass tufts (`Assets/Terrain/GrassTuft.prefab`, `DryTuft.prefab`) never cast shadows; `FarmTerrainBuilder` enforces this when regenerating terrain.
- `ProjectSettings/TimeManager.asset` caps the physics catch-up at 0.1 s per frame so a slow frame cannot spiral into more physics steps.

Measuring: build with `Tools > Build Windows Player`, then run the player once per tier:

    Builds\Windows\FarmSimulator.exe -benchmark -quality 0 -benchmark-out Builds\benchmark.csv -screen-fullscreen 0

Each run drives the menu to the farm with a rover, plants a 20 m plot, measures average and 1%-low frame rate plus CPU and GPU frame time for six seconds, appends a row to the CSV and quits. `-benchmark-seconds N` changes the window. GPU frame time is what matters for an integrated GPU: a tier that costs 4 ms of GPU time on a desktop card costs roughly ten times that on an Intel UHD chip.
