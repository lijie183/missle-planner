@echo off
call "C:/Program Files/Microsoft Visual Studio/18/Community/VC/Auxiliary/Build/vcvars64.bat" >nul 2>&1
cmake --build "C:/Work/missle-work/MissilePlanner/out/build/x64-Debug" --clean-first
