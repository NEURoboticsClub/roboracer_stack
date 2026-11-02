# RoboRacer Stack

A blank environment for testing new driving algorithms. You get the lidar scan and return a drive command; the rest of the stack is already wired up.

## Writing an algorithm

Edit one function, `computeDrive()` in [algorithm.cpp](ros_ws/src/planner_node/src/algorithm.cpp). It runs once per scan and starts out holding the car still:

```cpp
DriveCommand computeDrive(const std::vector<float> &ranges)
{
  DriveCommand cmd;
  cmd.steering = 0.0f;
  cmd.throttle = 0.0f;
  return cmd;
}
```

**Input:** `ranges[i]` is the distance in meters seen by beam `i`, at angle `ind2angle(i)` radians (0 is straight ahead, positive is left). A beam with no return can read `0`, `inf` or `nan`.

**Output:** both values are normalized, and anything outside `[-1, 1]` is clamped.

| Field | `+1.0` | `0.0` | `-1.0` |
| --- | --- | --- | --- |
| `steering` | full lock left | straight | full lock right |
| `throttle` | max throttle | coast | max brake |

Positive throttle scales up to one max throttle value and negative throttle scales up to one max brake value, so `0.5` is half of max throttle and `-0.5` is half of max brake. Each is set by one parameter in the launch file:

| | Sim (`sim_launch.py`) | Real car (`irl_launch.py`) |
| --- | --- | --- |
| `throttle` `+1.0` | `sim_max_throttle` (0.25) | `irl_max_throttle_erpm` (15000 eRPM, ~3.25 m/s) |
| `throttle` `-1.0` | `sim_max_brake` (0.25) | `irl_max_brake_amps` (4 A) |

The same code runs on both, with these differences:

| | Sim | Real car |
| --- | --- | --- |
| Lowest usable `throttle` | none | ~0.33, because the VESC won't start the motor below about 5000 eRPM |
| `throttle` below `0` | reverse throttle; holding it backs the car up | brake current; the car stops |
| E-stop | none; the car drives as soon as scans arrive | stopped until the kill switch is armed |

To run your changes in the sim, restart the onboard container. It rebuilds `ros_ws/src` on startup:

```bash
docker compose restart onboard
```

## Setup

### 1. Install Git LFS and the Simulator

#### Git LFS (Large File Storage)

**Windows** (Git LFS comes with Git for Windows):

```bash
git lfs install
```

**macOS:**

```bash
brew install git-lfs
```

```bash
git lfs install
```

#### Simulator

Go to the [AutoDRIVE RoboRacer Sim Racing release page](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/tag/2026-iros) and download the zip file that matches your operating system. It should be named like this:

```text
autodrive_simulator_practice_<YOUR_OPERATING_SYSTEM>.zip
```

> [!IMPORTANT]
> Only the **practice** version of the simulator will work.

### 2. Build the Onboard Docker Image

Make sure Docker Desktop is open (or the Docker daemon is running). Then, from the root directory of the repository, run:

```bash
docker build --tag roboracer_stack:onboard -f ./Onboard.Dockerfile .
```

> [!NOTE]
> The first build will take a while. Later builds are faster because of caching.

### 3. Run the Autonomy Stack and the Devkit

```bash
docker compose up
```

### 4. Launch the Simulator

Unzip the simulator folder you downloaded, then open the app.

- **Windows and Linux:** you're good to go.
- **macOS:** follow the [macOS setup guide](#macos-simulator-setup) below.

---

## macOS Simulator Setup

The macOS zip was built on Windows, so extracting it **loses the executable bit**, and macOS **quarantines** the downloaded app. Either one causes "The application cannot be opened" or "app is damaged" errors. The steps below fix both.

These steps work on both Apple Silicon and Intel Macs (the binary is universal).

### Step 1: Unzip

Go to wherever you downloaded the file:

```bash
cd ~/Downloads
```

```bash
unzip -q "autodrive_simulator_practice_macos.zip"
```

> [!TIP]
> - Adjust the zip name if yours is different, e.g. `autodrive_simulator_practice_macos (3).zip`.
> - Use `unzip` in Terminal, not a double-click in Finder. It makes the next steps more reliable.

### Step 2: Fix permissions, quarantine, and signature

```bash
cd ~/Downloads/autodrive_simulator
```

Make the executable runnable:

```bash
chmod -R +x "AutoDRIVE Simulator.app/Contents/MacOS"
```

Remove the quarantine flag from every file in the bundle:

```bash
xattr -dr com.apple.quarantine "AutoDRIVE Simulator.app"
```

Re-sign the app locally. This is a safe fallback in case the bundled ad-hoc signature doesn't survive extraction, since Apple Silicon Macs won't run an arm64 binary with an invalid signature:

```bash
codesign --force --deep --sign - "AutoDRIVE Simulator.app"
```

### Step 3: Run

```bash
open "AutoDRIVE Simulator.app"
```

Or double-click the app in Finder.

### Troubleshooting

- **Still blocked by Gatekeeper:** go to **System Settings → Privacy & Security**, scroll down, and click **Open Anyway** next to the AutoDRIVE message. Or right-click the app and choose **Open**.
- **Check that it's running:**

  ```bash
  pgrep -fl "AutoDRIVE Simulator"
  ```

- **Permission denied on `chmod`:** put `sudo` in front of the command.
