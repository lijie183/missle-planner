# Offline Satellite Earth Data

Place your `.earth` scene file and dependent data folders in this directory.

Recommended file name:

- `world.earth`

The app searches these paths in order:

- `default.earth`
- `world.earth`
- `data/default.earth`
- `data/world.earth`
- `data/earth/default.earth`
- `data/earth/world.earth`
- `resources/default.earth`
- `resources/world.earth`
- `resources/earth/default.earth`
- `resources/earth/world.earth`

Notes:

- If offline data is not found, the app tries online satellite/elevation tiles.
- If terrain engine plugins are unavailable, the app falls back to a procedural globe texture.
- Keep osgEarth terrain plugins in the executable directory (`osgPlugins-*`) as copied by CMake post-build.
