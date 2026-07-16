# MATRiX Release 0.1.2

## Highlights

- Added Pixel Streaming support for viewing the Unreal Engine simulation stream from a browser.
- Added custom URDF import support for loading third-party MuJoCo/URDF robot models through the provided launch scripts.
- Added one-click RoamerX Open integration in `sim_launcher` to start the ROS 2 Nav2 navigation stack together with MATRiX simulation.
- Added runtime environment checks before launch. The checks now validate system dependencies, ROS 2 Humble, MuJoCo, MC, UE runtime assets, shared libraries, and required configuration files.
- Added full offline package download options for `matrix_0.1.2.zip`, including Artifactory, Google Drive, and Baidu Netdisk links, with SHA256 verification.

## New Maps

- **ApartmentWorld**: apartment-style indoor scene for navigation and obstacle traversal tests.
- **MeetRoomWorld**: meeting-room indoor scene for compact navigation and interaction tests.
- **CaliWorld**: calibration and basic validation scene for quick functional checks.

## Bug Fixes

- Fixed `sim_launcher` keyboard control mode capturing movement keys globally and preventing other applications from using those keys normally.
- Fixed release chunk installation issues around manifest refresh, split package download, merge, and checksum validation.
- Fixed incomplete assets installation detection so missing runtime files trigger a reinstall instead of failing later at launch.
- Fixed Ubuntu 22.04 dependency installation issues, including local deb install paths, optional ROS image transport packages, Zenoh, and `robot-forward` compatibility.
- Fixed runtime shared library isolation between UE, MuJoCo, and MC processes to avoid `LD_LIBRARY_PATH` contamination.
- Improved error reporting for missing runtime assets and shared libraries, including clearer `ldd` version-error diagnostics.
- Fixed RoamerX Open workspace detection and launcher path handling.

## Upgrade Notes

- Recommended environment: Ubuntu 22.04 with ROS 2 Humble.
- Existing users should rerun `scripts/install_deps.sh` and reinstall the 0.1.2 assets/base/chunk packages.
- The Artifactory package link is for internal network use. External users should use Google Drive or Baidu Netdisk.

## Required Packages

- **assets-0.1.2.tar.gz** (~675MB)
  - Launcher files: `bin/sim_launcher`, `bin/sim_launcher.bin`
  - MuJoCo runtime binary and dynamic map payloads
  - MC runtime binaries and shared libraries, including WBC libraries
  - ONNX controller models: `xg`, `xg_wheel`, `zg_wheels`
  - UE runtime dependencies, including OpenCV shared libraries

- **base-0.1.2.tar.gz** (~2.0GB)
  - Chunk 0 / EmptyWorld core content
  - Core Blueprints and system files
  - Published robot model directories: `xgb`, `xgw`, `zgws`, `go2`, `go2w`
  - Runtime template directories: `Content/model/config`, `Content/model/SceneLoder`

Internal robot model directories are not included in this release. `Content/model/dynamicmap` is created at runtime for MoonWorld from `dynamicmaps/moonworld.bin`.

## Recommended / Optional Packages

- **shared-0.1.2.tar.gz** (~3.3GB): shared resources used by multiple maps. This file exceeds GitHub's single-asset size limit and is uploaded as split parts when needed.
- **Map packages**: install only the maps you need.

## Checksums

- `assets-0.1.2.tar.gz`: `ec18164489775c9f5ac1f73da4790c8c5d18f701cddf48ec1a8e8d7b3861e8fc`
- `base-0.1.2.tar.gz`: `2cbb40861e89c40735cd64b24e8b64d88d012f335bdb405b5ed52db86f8b4e38`

Verify all downloaded packages with:

```bash
sha256sum -c checksums-0.1.2.sha256
```

## Install

```bash
bash scripts/release_manager/install_chunks.sh 0.1.2
```

For offline/local installation, place downloaded packages in `releases/` and run:

```bash
bash scripts/release_manager/install_chunks_local.sh 0.1.2
```
