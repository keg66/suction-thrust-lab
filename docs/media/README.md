# README animation previews

These GIFs were generated from this project's MuJoCo simulation and primitive vehicle/course geometry. No external footage, product images, logos, manufacturer CAD or third-party audio were used. These project-generated GIFs are covered by the [MIT License](../../LICENSE). This grant does not cover external material.

| Preview | Local source recording | Playback |
|---|---|---|
| `mini-vehicle-roll180.gif` | `output/videos/suction-thrust-lab/halfpipe-roll-side.mp4` | 49.8 s, including appended 0.25× air-control replay |
| `full-size-roll180.gif` | `output/videos/suction-thrust-lab/full_size-side.mp4` | 98.4 s, including appended 0.25× air-control replay |
| `mini-vehicle-tornado.gif` | `output/videos/suction-thrust-lab/tornado.mp4` | 21.0 s, including appended 0.25× flight replay and cumulative gas use (g) |
| `full-size-tornado.gif` | `output/videos/suction-thrust-lab/tornado_full_size.mp4` | 21.4 s, including appended 0.25× flight replay and cumulative gas use (kg) |

All previews are 960 × 540 pixels and loop indefinitely. Half-pipe previews use 10 fps; tornado previews use 20 fps to preserve fast roll visibility. Half-pipe GIFs use 128 colors with ordered dithering; tornado GIFs use 64 colors without dithering to keep the larger frame count compact. Frame sampling and color quantization do not change the source playback speed. Titles, physical-time captions, side/axial views and the labelled slow replay come from the recording scripts; the GIF conversion does not recompute or alter physics.

Regenerate from the repository root, with the project installed and system FFmpeg available:

```bash
MUJOCO_GL=egl python scripts/record_halfpipe.py --demo halfpipe
MUJOCO_GL=egl python scripts/record_halfpipe.py --demo full_size
MUJOCO_GL=egl python scripts/record_tornado.py
MUJOCO_GL=egl python scripts/record_tornado.py --demo tornado_full_size
python scripts/make_readme_gifs.py
```

To regenerate only the tornado GIFs, run `python scripts/make_readme_gifs.py --demo tornado tornado_full_size` after recording them.

The source MP4 recordings and telemetry remain ignored under `output/`. The four GIFs are intended for repository/README display and are included in the source archive, while the runtime wheel excludes them. The full-size setup uses unverified hardware assumptions; these previews demonstrate simulated behavior, not measured production-car performance.
