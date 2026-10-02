# RoboRacer Stack

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
