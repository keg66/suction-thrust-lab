"""Convert the demo recordings to complete, looping README previews.

Run the recording scripts first, then: python scripts/make_readme_gifs.py
Requires system ffmpeg; no additional Python packages are needed.
"""
import argparse
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory

ROOT = Path(__file__).resolve().parents[1]
RECORDINGS = ROOT / 'output' / 'videos' / 'suction-thrust-lab'
MEDIA = ROOT / 'docs' / 'media'
PREVIEWS = {
    'halfpipe': ('halfpipe-roll-side.mp4', 'mini-vehicle-roll180.gif', 10),
    'full_size': ('full_size-side.mp4', 'full-size-roll180.gif', 10),
    'tornado': ('tornado.mp4', 'mini-vehicle-tornado.gif', 20),
    'tornado_full_size': ('tornado_full_size.mp4', 'full-size-tornado.gif', 20),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo', choices=PREVIEWS, nargs='+', help='convert only selected demos (default: all)')
    args = parser.parse_args()
    previews = [PREVIEWS[name] for name in (args.demo or PREVIEWS)]
    for source, _, _ in previews:
        if not (RECORDINGS / source).is_file():
            raise SystemExit(f'Missing recording: {source}. Run the matching recording script first.')
    MEDIA.mkdir(parents=True, exist_ok=True)
    # Both frame rates map exactly to centiseconds. The tornado previews use
    # 20 fps so fast rolling is legible without changing the playback speed.
    with TemporaryDirectory(prefix='suction-thrust-gif-') as temporary:
        for source, name, fps in previews:
            base = f'fps={fps},scale=960:-1:flags=lanczos'
            colors = 64 if fps == 20 else 128
            dither = 'none' if fps == 20 else 'bayer:bayer_scale=5'
            palette = Path(temporary) / 'palette.png'
            target = MEDIA / name
            subprocess.run([
                'ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
                '-i', str(RECORDINGS / source), '-vf',
                base + f',palettegen=max_colors={colors}:stats_mode=diff',
                '-frames:v', '1', str(palette),
            ], check=True)
            subprocess.run([
                'ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
                '-i', str(RECORDINGS / source), '-i', str(palette),
                '-filter_complex', f'[0:v]{base}[scaled];[scaled][1:v]'
                f'paletteuse=dither={dither}:diff_mode=rectangle',
                '-map_metadata', '-1', '-loop', '0', '-final_delay', str(100//fps), str(target),
            ], check=True)
            print(f'{target.name}: {target.stat().st_size / 1024 / 1024:.2f} MiB', flush=True)


if __name__ == '__main__':
    main()
