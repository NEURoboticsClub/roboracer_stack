# Build & Push: `dheeshik/roboracer_neu:devkit`

Run all commands from the **repo root** (the folder that contains `devkit/`).
Run `docker login` first on each machine.

- Dockerfile: `./devkit/autodrive_devkit.Dockerfile`
- Build context: `./devkit`, so `COPY` paths in the Dockerfile are relative to `devkit/`
- Build metadata is written to `./devkit/build-metadata-<arch>.json`

## Mac (Apple Silicon → arm64)

```bash
docker buildx build --platform linux/arm64 -f ./devkit/autodrive_devkit.Dockerfile -t dheeshik/roboracer_neu:devkit-arm64 --metadata-file ./devkit/build-metadata-arm64.json --push ./devkit
```

## Windows (amd64)

```powershell
docker buildx build --platform linux/amd64 -f ./devkit/autodrive_devkit.Dockerfile -t dheeshik/roboracer_neu:devkit-amd64 --metadata-file ./devkit/build-metadata-amd64.json --push ./devkit
```

## Combine into one `devkit` tag

After both are pushed, run this from either machine:

```bash
docker buildx imagetools create -t dheeshik/roboracer_neu:devkit dheeshik/roboracer_neu:devkit-amd64 dheeshik/roboracer_neu:devkit-arm64
```