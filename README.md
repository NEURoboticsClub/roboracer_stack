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
** ONLY THE "PRACTICE" VERSION OF THE SIMULATOR WILL WORK **

## 2. Build the Onboard Docker Image Locally on your Computer
Make sure you have Docker Desktop open (or the docker daemon is running)

Make sure you are in the root directory of the repository, then:
```bash
# This will probably take a bit of time the first time it is run, but it'll be faster after that because of caching
docker build --tag roboracer_stack:onboard -f .\Onboard.Dockerfile .
```


## 3. Run the Autonomy Stack and the Devkit
```bash
docker compose up
```

## 4. Launch the Simulator App
Unzip the simulator folder that you downloaded, then open the app.