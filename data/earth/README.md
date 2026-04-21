# Offline Satellite Earth Data

Place your `.earth` scene file and dependent data folders in this directory.

Recommended file name:

- `highres_global.earth` (online TMS/XYZ, no large local imagery package required)

- `world.earth`

Quick start for online high-resolution globe:

1. Use `highres_global.earth` from this folder.
2. Keep network access enabled.
3. The app will cache map tiles on disk (`./osgearth_cache`) for faster repeated runs.

The app searches these paths in order:

- `highres_global.earth`
- `default.earth`
- `world.earth`
- `data/highres_global.earth`
- `data/default.earth`
- `data/world.earth`
- `data/earth/highres_global.earth`
- `data/earth/default.earth`
- `data/earth/world.earth`
- `resources/highres_global.earth`
- `resources/default.earth`
- `resources/world.earth`
- `resources/earth/highres_global.earth`
- `resources/earth/default.earth`
- `resources/earth/world.earth`

Notes:

- If offline data is not found, the app tries online satellite/elevation tiles.
- If terrain engine plugins are unavailable, the app falls back to a procedural globe texture.
- Keep osgEarth terrain plugins in the executable directory (`osgPlugins-*`) as copied by CMake post-build.
