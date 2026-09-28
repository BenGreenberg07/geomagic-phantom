# Setting up the lab PC

What's already done (per the Oct–Nov 2025 notes): the PHANToM device driver and
OpenHaptics Developer Edition 3.5.0 are installed, and Visual Studio 2022
Community with C++ is installed. The SDK examples build and run.

## One-time

1. **Get this repo** somewhere outside the OpenHaptics folder, e.g.
   `C:\Users\<you>\Documents\geomagic-phantom`:
   ```
   git clone https://github.com/bengreenberg07/geomagic-phantom.git
   ```
   (GitHub Desktop works too.)
2. **Check the SDK location.** The OpenHaptics installer normally sets the
   environment variable `OH_SDK_BASE` (e.g. `C:\OpenHaptics\Developer\3.5.0`).
   `build.bat` falls back to that path if the variable is missing.
3. **Python for analysis** (optional): install Python 3, then
   `pip install -r analysis\requirements.txt`.

## Every time

```
build.bat                      # builds all programs into bin\
bin\device_check.exe           # sanity check + calibration + weak test spring
bin\position_tracker.exe --id P01
python analysis\analyze.py data\P01_..._tracker.csv
```

You can double-click the `.exe` files in `bin\`, but running them from a terminal
(open the repo folder in File Explorer, type `cmd` in the address bar) keeps the
window open if something fails.

## If build.bat fails

* `OpenHaptics not found` → set `OH_SDK_BASE` (System Properties → Environment
  Variables), or edit the path at the top of `build.bat`.
* `hd.lib not found` → look in `%OH_SDK_BASE%\lib\` for where `hd.lib` actually is
  and add that folder to the list near the top of `build.bat`.
* `Visual Studio C++ compiler not found` → open the Visual Studio Installer and
  make sure the **Desktop development with C++** workload is ticked.
* A compile error → it's printed along with the file and line. The build output
  for each program is also saved in `build\<name>.log`.

## Using Visual Studio instead (optional)

In Visual Studio: **File → Open → Folder** and pick the repo. It detects
`CMakeLists.txt`, and every app shows up as a target. There are no project files
to maintain.

## Working from a Mac (no device)

The apps can be compiled against a **simulated device**, so code can be written
and tested on a laptop before going to the lab:
```
cmake -S . -B build-mock -DPHANTOM_MOCK=ON
cmake --build build-mock
./bin/position_tracker --id test
```
The simulator is a point mass with a "hand" slowly drifting in circles, and it
presses the button every 3 s (`HD_MOCK_BUTTON_PERIOD=1` to change). It only
checks logic and crashes. It says nothing about how forces *feel*.
