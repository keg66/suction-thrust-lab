# Third-party dependencies and notices

The project source does not vendor dependency source, binary wheels, manufacturer CAD, third-party photographs or footage, logos, decals or fonts. Dependencies are installed separately and retain their own licenses; the project's MIT license does not relicense them.

This review checked installed metadata/license files in the verified environment and official upstream license sources. A different dependency version or platform wheel can contain different bundled components. Preserve the exact upstream license/NOTICE files if you later redistribute those components.

| Component | Reviewed version | License / scope |
|---|---:|---|
| [MuJoCo](https://github.com/google-deepmind/mujoco/blob/main/LICENSE) | 3.16.0 | Apache-2.0; its wheel also carries `LICENSES_THIRD_PARTY.md` |
| [NumPy](https://github.com/numpy/numpy/blob/main/LICENSE.txt) | 2.5.4 | BSD-3-Clause core plus additional notices; see binary caveat below |
| [pytest](https://github.com/pytest-dev/pytest/blob/main/LICENSE) | 9.1.1 | MIT; test-only dependency |
| [Python glfw](https://github.com/FlorianRhiem/pyGLFW) | 2.11.0 | MIT wrapper; bundled native GLFW uses the [zlib/libpng-style license](https://github.com/glfw/glfw/blob/master/LICENSE.md) |
| [PyOpenGL](https://github.com/mcfletch/pyopengl/blob/master/license.txt) | 3.1.10 | BSD-style and component notices; wheel metadata says BSD but does not expose a top-level license file, so upstream license text was checked |
| absl-py; etils | 2.5.1; 1.14.0 | Apache-2.0; installed license files checked |
| websockets; fsspec | 17.2; 2026.9.0 | BSD-3-Clause |
| typing_extensions | 4.16.0 | PSF-2.0 |
| zipp; iniconfig; pluggy | 4.1.1; 2.3.1; 1.7.0 | MIT |
| packaging; Pygments | 26.3; 2.21.0 | Apache-2.0 OR BSD-2-Clause; BSD-2-Clause |
| setuptools; build; wheel; pyproject_hooks | 84.0.0; 1.6.1; 0.48.0; 1.3.3 | MIT; build tools, not runtime code copied into this repository |

## Binary-distribution caveats

The reviewed NumPy wheel includes OpenBLAS (BSD-3-Clause), LAPACK (BSD-3-Clause-Open-MPI), GCC runtime libraries (GPL-3.0-or-later WITH GCC-exception-3.1), and libquadmath (LGPL-2.1-or-later), as documented in its installed `LICENSE.txt`. Its metadata also records 0BSD, MIT, Zlib and CC0-1.0 components. Do not describe the entire binary stack as MIT or as having only permissive licenses.

The MuJoCo wheel includes additional third-party components and notices. Importing installed libraries does not change their license. This repository's wheel/sdist should contain only this project's files; distributing a frozen application, container, dependency bundle, or executable would require another inventory and retention of the applicable notices/obligations. No such binary bundle is prepared here.

## Optional system tools

Recording calls a separately installed FFmpeg executable through `subprocess`; no FFmpeg source/binary is included. FFmpeg licensing depends on its build options; the inspected Ubuntu build enables GPL components including libx264. See the [official FFmpeg license information](https://ffmpeg.org/legal.html). System fonts and graphics drivers are also outside this source distribution.

## Geometry, data and media

Vehicle surfaces, wheels, sites and curved-track blocks are generated locally from primitives and numerical parameters. No imported mesh, texture, dataset or manufacturer asset was found among current or historical tracked files. Reference links do not grant asset permissions.

Generated CSV, logs, screenshots and full-resolution MP4 recordings remain under ignored `output/` and are not part of the publication payload. The two README GIF previews in `docs/media/` are project-generated renders, reviewed for inclusion in the repository and source distribution at the user's request. They use the current primitive scenes and new branding, with no manufacturer footage or images; [their provenance and regeneration](docs/media/README.md) are recorded alongside them. Earlier local renderings may retain retired names and remain excluded. Review additional media separately before distribution.
