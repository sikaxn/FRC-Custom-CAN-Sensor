@echo off
setlocal

REM Build bindings for Python 3.12 (preferred for tkinter app)
pushd "%~dp0"
if exist build-py312\\CMakeCache.txt (
  rmdir /s /q build-py312
)
cmake -S . -B build-py312 -DPYTHON_EXECUTABLE=C:\Users\Nathan\AppData\Local\Programs\Python\Python312\python.exe -DPYTHON_LIBRARY=C:\Users\Nathan\AppData\Local\Programs\Python\Python312\libs\python312.lib
if errorlevel 1 exit /b 1

cmake --build build-py312 --config Release
if errorlevel 1 exit /b 1

echo Built build-py312\Release\ledsim.cp312-win_amd64.pyd
popd
