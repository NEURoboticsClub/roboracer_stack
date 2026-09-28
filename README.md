# First Steps:
## 1. Install Git LFS (Large File Support) and the Simulator Application

Windows:
```bash
git lfs install
```

Mac:
```bash
brew install git-lfs
```


### Go to this [link](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/tag/2026-iros) and download the zip file that matches your operating system and make sure it looks like:
```bash
autodrive_simulator_practice_[[YOUR_OPERATING_SYSTEM_HERE]].zip
```
> ** ONLY THE "PRACTICE" VERSION OF THE SIMULATOR WILL WORK **

## 2. Build the Onboard Docker Image Locally on your Computer
Make sure you have Docker Desktop open (or the docker daemon is running)

Make sure you are in the root directory of the repository, then:
>This will probably take a bit of time the first time it is run, but it'll be faster after that because of caching
```bash
docker build --tag roboracer_stack:onboard -f ./Onboard.Dockerfile .
```


## 3. Run the Autonomy Stack and the Devkit
```bash
docker compose up
```

## 4. Launch the Simulator App
Unzip the simulator folder that you downloaded, then open the app. On Windows and Linux you should be good.

On macos you should follow this guide to get it up and running:

#### AutoDRIVE Simulator – macOS Setup:

The macOS zip was built on Windows, so extracting it **loses the executable bit**, and
macOS **quarantines** the downloaded app. Either one causes "The application cannot be
opened" or "app is damaged". These commands fix both.

Works on Apple Silicon and Intel Macs (the binary is universal).

## 1. Unzip

```bash
# cd wherever you have the file downloaded
cd ~/Downloads
```

```bash
unzip -q "autodrive_simulator_practice_macos.zip"
```

> Adjust the zip name if yours is different, e.g. `autodrive_simulator_practice_macos (3).zip`.
> Use `unzip` in Terminal, not Finder's double-click. It makes the next steps more reliable.

## 2. Fix permissions, quarantine, and signature

```bash
cd ~/Downloads/autodrive_simulator
```

Make the executable runnable:

```bash
chmod -R +x "AutoDRIVE Simulator.app/Contents/MacOS"
```

Remove the quarantine flag (recursively, from every file in the bundle):

```bash
xattr -dr com.apple.quarantine "AutoDRIVE Simulator.app"
```

Re-sign the app locally. This is a safe fallback in case the bundled ad-hoc signature doesn't survive extraction. Apple Silicon Macs won't run an arm64 binary with an invalid signature:

```bash
codesign --force --deep --sign - "AutoDRIVE Simulator.app"
```

## 3. Run

```bash
open "AutoDRIVE Simulator.app"
```

Or double-click the app in Finder.

## Troubleshooting

- **Still blocked by Gatekeeper:** go to System Settings → Privacy & Security, scroll down,
  and click **Open Anyway** next to the AutoDRIVE message. Or right-click the app and choose **Open**.
- **Check that it's running:**
  ```bash
  pgrep -fl "AutoDRIVE Simulator"
  ```
- **Permission denied on `chmod`:** put `sudo` in front of the command.
