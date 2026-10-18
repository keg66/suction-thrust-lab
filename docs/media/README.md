# README animation previews

These GIFs were generated from this project's MuJoCo simulation and primitive vehicle/course geometry. No external footage, product images, logos, manufacturer CAD or third-party audio were used. These project-generated GIFs are covered by the [MIT License](../../LICENSE). This grant does not cover external material.

| Preview | Local source recording | Playback |
|---|---|---|
| `mini-vehicle-roll180.gif` | `output/videos/suction-thrust-lab/halfpipe-roll-side.mp4` | 49.8 s, including appended 0.25× air-control replay |
| `full-size-roll180.gif` | `output/videos/suction-thrust-lab/full_size-side.mp4` | 98.4 s, including appended 0.25× air-control replay |

Both previews are 960 × 540 pixels, 10 fps and loop indefinitely. A 128-color palette with ordered dithering reduces size. Frame sampling and color quantization do not change the source playback speed. Titles, physical-time captions, side/axial views and the labelled slow replay come from the recording scripts; the GIF conversion does not recompute or alter physics.

Regenerate from the repository root, with the project installed and system FFmpeg available:

```bash
MUJOCO_GL=egl python scripts/record_halfpipe.py --demo halfpipe
MUJOCO_GL=egl python scripts/record_halfpipe.py --demo full_size
python scripts/make_readme_gifs.py
```

The source MP4 recordings and telemetry remain ignored under `output/`. The two GIFs are intended for repository/README display and are included in the source archive, while the runtime wheel excludes them. The full-size setup uses unverified hardware assumptions; these previews demonstrate simulated behavior, not measured production-car performance.
