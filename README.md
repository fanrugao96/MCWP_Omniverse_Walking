# MCWP Omniverse Walking

A Windows desktop demonstration of a moving MCWP platform and a rigged worker, using NVIDIA Omniverse Kit 110.4. Platform lifting is kinematic; worker motion uses procedural skeletal animation, planted-foot controls and geometry-derived walking boundaries.

## Requirements

- Windows with an NVIDIA RTX-capable graphics environment.
- Omniverse Kit 110.4.0 with its base-editor extension cache. The project application definition and extension version selections are in `apps/mcwp.walking.kit`; the SDK and extensions are installed separately.
- The Kit Python 3.12 modules listed in `requirements.txt`. The existing Kit pip archive supplies these versions. `carb`, `omni.*` and `pxr` come from Kit, not from this requirements file.

## Launch

Set `MCWP_KIT` to the SDK release directory that contains `kit/kit.exe` and `extscache`. For example, in PowerShell:

```powershell
$env:MCWP_KIT = 'D:\Omniverse\Kit-110.4.0\_build\windows-x86_64\release'
.\Launch_MCWP.cmd
```

Without `MCWP_KIT`, the launcher checks `%USERPROFILE%\NVIDIA\Omniverse\Kit-110.4.0\_build\windows-x86_64\release`. Set the variable through Windows environment settings for double-click launching from another installation location. `Start MCWP Walking.cmd` is an equivalent entry point.

The launcher uses the `.kit` file and scripts from its own folder, so the project may be moved or renamed. It creates the output directories when needed. See the [Kit application configuration documentation](https://docs.omniverse.nvidia.com/kit/docs/kit-manual/latest/guide/configuring.html) for how application files and extension folders are loaded.

Opening `MCWP_Simulation.usda` alone displays the saved scene without interactive controls. A normal launch reconstructs and saves the demonstration scene and resets platform height, worker position/pose and the view. Save a separate copy before making scene edits you want to retain.

## Controls

| Panel or command | Action |
| --- | --- |
| MCWP Lift: Raise / Lower / Stop | Raise, lower or stop the platform; travel 0–1.60 m, demonstration speed 0.08 m/s |
| Worker Remote: FORWARD / BACK | Hold to walk; release to stop root translation |
| Worker Remote: TURN LEFT / TURN RIGHT | Hold to turn; release to stop turning |
| Worker Remote: STOP | Clear worker movement input |
| Arms down / Arms forward / up | Select a standing pose; walking blends into arm swing |
| Close view / Full view | Worker close-up or full-platform view |
| Reset worker | Reset the worker |
| Simulation: Play / Pause / Reset | Run/pause simulation updates or reset the demonstration |
| Camera navigation | WASD and Q/E move; arrow keys turn; T teleports forward 2 m |
| G; I/J/K/L and U/O | Grab/release the orange diagnostic ball; move the desktop hand proxy |
| Record | Generate an orbit recording in `recordings/` |
| Verify controls | Run the selected checks and generate diagnostic files in `evidence/` |

If a control panel is closed, use the top **MCWP** menu: **MCWP Lift**, **Worker Remote**, **MCWP Simulation Controls** or **Show All Controls**. Reopening a panel preserves simulation state and does not call Reset.

Use mouse buttons for worker control. A raised foot completes its landing after release, within approximately 0.57 seconds. The walking region follows the 13 deck panels; masts, equipment and closed guardrails remain obstacles. The worker cannot leave the deck or simulate falling. **Pose limited: obstacle** means an arm pose has stopped within its clearance limit. **Shoe-to-edge** is the minimum planar clearance to the outer deck boundary, not a hazard score or distance to every obstacle.

## Files and generated outputs

- `scripts/`: runtime modules and source-package utilities.
- `assets/`: platform/worker USD assets, textures, navigation data, the original controller log, and source/license metadata.
- `apps/mcwp.walking.kit`: project-local application configuration.
- `capture_exposure_profile.json`, `runtime_control.json`, `ros_config.json`, `depth_tf_config.json`: runtime configuration.
- `evidence/`, `recordings/`: generated diagnostics and recordings, excluded from source delivery except the required worker-clearance reference.

Keep `assets/lift/lift_geometry_validation.json` and `evidence/worker_envelope_validation.json`: the lift verification routine reads both files. Worker verification updates the latter. `assets/UE_RightHand_original.csv` is input for controller replay, not a recording produced by this application.

All current model, texture and input files are included. Recreating the original Unreal project or rebuilding the worker in an authoring tool requires the separate original source packages.

## Optional ROS and depth/TF features

`ros_config.json` defaults to `url=null`; no ROS connection is made automatically. Configure a working ROS1 rosbridge endpoint before manually sending an RGB image.

For depth/TF output, set the real parent-frame name, optical-frame name and USD parent-prim mapping in `depth_tf_config.json`, then run **Validate depth / TF adapters** to generate a fresh capture. `mcwp_depth_tf.publish_latest_capture(url)` sends that capture explicitly. Depth uses optical Z, little-endian 16UC1 millimeters, with 0 for invalid values. The adapter is not a continuous ROS stream; original camera extrinsics and real deployment frame names must be supplied by the operator.

The application does not establish real actuator forces, payload capacity, full-body dynamics, falling, physical human balance, live XR integration or a real ROS deployment. Recheck walking and clearances after changing platform geometry or equipment.

## Verify and package

Use Python 3.12 or newer for these standard-library utilities; they do not start Kit:

```powershell
python scripts/verify_delivery.py
python scripts/package_delivery.py --manifest-only
python scripts/package_delivery.py
python scripts/verify_delivery.py --archive dist/MCWP_Omniverse_Walking.zip
```

Run the first command to check the supplied source inventory. After intentional changes, use `--manifest-only` to refresh both file manifests, or run the packager to refresh them and create the ZIP. The package includes runtime source, assets, configuration, documentation and the two required validation inputs. It excludes recordings, generated diagnostics, caches, local environments and backups.

`.gitattributes` preserves exact file bytes across Git checkouts for SHA-256 verification. `.gitignore` excludes future generated outputs. The source manifest is a snapshot: launching the application can update the saved scene and runtime files; refresh the manifest before a new delivery if those changes are intended.

## Asset provenance

Worker source information is in `assets/worker/Source_Provenance.json`; its Microsoft Rocketbox license is retained in `assets/worker/LICENSE_Rocketbox_MIT.txt`. `assets/worker/SHA256_Manifest.json` describes the worker files actually included in this project.
