# Fix: `docker compose up` fails with `autodrive_devkit.sh: permission denied`

## Symptom

```
Error response from daemon: failed to create task for container: ... exec: "/home/autodrive_devkit.sh": permission denied: unknown
```

The `onboard` container starts. The `devkit` container stays stuck in `Created`.

## Cause

The published image `dheeshik/roboracer_neu:devkit` has its entrypoint script without the execute bit:

```
-rw-r--r-- 1 root root 384 /home/autodrive_devkit.sh
```

- Git tracks `devkit/autodrive_devkit.sh` as `100644` (not executable).
- `autodrive_devkit.Dockerfile` copied the script but never ran `chmod +x` on it. `Onboard.Dockerfile` does run `chmod +x` for `onboard_setup.sh`, which is why `onboard` works.
- The image was built from a checkout where the file wasn't executable, so the container can't run it.
- `compose.yaml` sets `pull_policy: always` for `devkit`. Every `docker compose up` pulls the broken image from Docker Hub again, so a local rebuild alone gets overwritten.

## Fix (already applied in the repo)

`devkit/autodrive_devkit.Dockerfile` now has this line:

```dockerfile
COPY autodrive_devkit.sh /home
RUN chmod +x /home/autodrive_devkit.sh
ENTRYPOINT ["/home/autodrive_devkit.sh"]
```

This makes the image work whatever the file permissions are on the build machine, including Windows.

## Commands to run

Run everything from the repo root.

### 1. Mark the script executable in git (optional, recommended)

```bash
git update-index --chmod=+x devkit/autodrive_devkit.sh onboard_setup.sh
```

### 2. Rebuild and push both architectures

Run `docker login` first.

On the Mac (arm64):

```bash
docker buildx build --platform linux/arm64 -f ./devkit/autodrive_devkit.Dockerfile -t dheeshik/roboracer_neu:devkit-arm64 --metadata-file ./devkit/build-metadata-arm64.json --push ./devkit
```

On Windows (amd64), after pulling the Dockerfile change:

```powershell
docker buildx build --platform linux/amd64 -f ./devkit/autodrive_devkit.Dockerfile -t dheeshik/roboracer_neu:devkit-amd64 --metadata-file ./devkit/build-metadata-amd64.json --push ./devkit
```

Then combine both into the `devkit` tag (from either machine):

```bash
docker buildx imagetools create -t dheeshik/roboracer_neu:devkit dheeshik/roboracer_neu:devkit-amd64 dheeshik/roboracer_neu:devkit-arm64
```

If you only rebuild one architecture, the combined `devkit` tag still points to the old, broken image for the other one.

### 3. Recreate the containers

```bash
docker compose down
```

```bash
docker compose up
```

### 4. Verify

```bash
docker run --rm --entrypoint ls dheeshik/roboracer_neu:devkit -l /home/autodrive_devkit.sh
```

The output should show `-rwxr-xr-x`.

## Quick unblock without rebuilding

If you need it working right now before pushing a new image, override the entrypoint for the `devkit` service in `compose.yaml`. Running the script through `bash` doesn't need the execute bit:

```yaml
  devkit:
    image: "dheeshik/roboracer_neu:devkit"
    pull_policy: always
    entrypoint: ["bash", "/home/autodrive_devkit.sh"]
    ...
```

Remove this once the fixed image is pushed.

## Side note: 218 "modified" files in `git status`

Almost every file shows as modified with 0 lines changed. These are only permission-mode changes (`100644 → 100755`); the files were chmod'd to `777` locally. They have nothing to do with this bug. To hide them:

```bash
git config core.fileMode false
```
