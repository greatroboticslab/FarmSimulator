# FarmSimulator Project Audit

## What Is Project-Owned

- Main simulator scene: `Assets/Scenes/SampleScene.unity`.
- Farm/map prefabs:
  - `Assets/Maps/MTSU Farm.prefab`
  - `Assets/Maps/StartMap.prefab`
  - `Assets/Subscenes/farmInTheCity.prefab`
  - Utility map prefabs such as waypoint, stake, and training plane prefabs.
- Robot prefabs: 16 prefabs under `Assets/Robots`.
- Project scripts: 56 scripts under `Assets/Scripts`.

The repo also contains many vendor/example scenes from Oculus, TextMesh Pro, Online Maps, Real World Terrain, M2MQTT, Starter Assets, and the bundled ML-Agents release. Those are useful references, but they are not the main simulator entrypoint.

## Farm Dropdown Wiring

Farm names come from `Assets/Data/locations.csv`.

- `MTSU Farms` and alias `MTSU Farm` load the dedicated MTSU farm prefab.
- `Farm in the City` loads the dedicated city farm subscene prefab.
- `Hutchinson Farms`, `Freethought Farm`, and `Williamson Family Farm` load the generic coordinate-backed `StartMap` prefab.

## Optional Services

Local play should not require lab/network services.

- MQTT is disabled by default on robot prefabs and in `MqNode`.
- ROS publishing is disabled unless `ROSRover.enableRos` is set.
- Socket streaming is disabled unless `SocketInterace.enableSocket` is set.
- Groq robot chat is disabled unless `Assets/StreamingAssets/config.json` provides a key.
- Bing elevation on `StartMap` is disabled to avoid expired-key `403` errors during local play.

## Verification

- Unity `2022.3.8f1` batch compile succeeded.
- `Git LFS fsck` passed.
- No missing Mono scripts were found in project-owned scenes, maps, subscenes, or robot prefabs.
- The fresh Unity compile log did not contain C# errors, `NullReferenceException`, `MissingReferenceException`, MQTT connection spam, Bing `403`, or inactive-controller errors.

## Known Non-Blocking Warnings

Unity still reports existing compile warnings such as hidden inherited member names, unused fields, and an obsolete `Ssl3` reference inside the bundled M2MQTT code. These do not block compilation or local play.
