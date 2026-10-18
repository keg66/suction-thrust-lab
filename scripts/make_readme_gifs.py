"""Convert the two branded recordings to complete, looping README previews.

Run the recording scripts first, then: python scripts/make_readme_gifs.py
Requires system ffmpeg; no additional Python packages are needed.
"""
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory

ROOT = Path(__file__).resolve().parents[1]
RECORDINGS = ROOT / 'output' / 'videos' / 'suction-thrust-lab'
MEDIA = ROOT / 'docs' / 'media'
PREVIEWS = [
    ('halfpipe-roll-side.mp4', 'mini-vehicle-roll180.gif'),
    ('full_size-side.mp4', 'full-size-roll180.gif'),
]


def main():
    for source, _ in PREVIEWS:
        if not (RECORDINGS / source).is_file():
            raise SystemExit(f'Missing recording: {source}. Run scripts/record_halfpipe.py for both demos first.')
    MEDIA.mkdir(parents=True, exist_ok=True)
    # 10 fps maps exactly to GIF's centisecond timing; playback speed is unchanged.
    base = 'fps=10,scale=960:-1:flags=lanczos'
    with TemporaryDirectory(prefix='suction-thrust-gif-') as temporary:
        for source, name in PREVIEWS:
            palette = Path(temporary) / 'palette.png'
            target = MEDIA / name
            subprocess.run([
                'ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
                '-i', str(RECORDINGS / source), '-vf',
                base + ',palettegen=max_colors=128:stats_mode=diff',
                '-frames:v', '1', str(palette),
            ], check=True)
            subprocess.run([
                'ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
                '-i', str(RECORDINGS / source), '-i', str(palette),
                '-filter_complex', f'[0:v]{base}[scaled];[scaled][1:v]'
                'paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle',
                '-map_metadata', '-1', '-loop', '0', '-final_delay', '10', str(target),
            ], check=True)
            print(f'{target.name}: {target.stat().st_size / 1024 / 1024:.2f} MiB', flush=True)


if __name__ == '__main__':
    main()
