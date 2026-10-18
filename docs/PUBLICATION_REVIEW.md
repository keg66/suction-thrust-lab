# Publication review

Project: **Suction & Thrust Lab**, intended repository: `suction-thrust-lab`.
Review date: 2026-10-18. This is a technical source audit, not legal clearance.

## Scope and provenance

All three reachable commits and their 32 distinct file-content blobs were inspected before the rename. Their 20 historical paths contain Python, JSON, Markdown, packaging and ignore files. There were no binary blobs, vendored libraries, third-party model/media assets, or blobs larger than 1 MB. No existing LICENSE or source license headers were present. Local implementations generate primitive vehicle/course geometry and compute the physical models and control. This supports describing the implementation as developed within the project; it does not independently establish copyright ownership.

The reviewed current source does not include manufacturer CAD, logos, decals, product photographs or presentation video. User-supplied reference images remain outside the repository. Existing local generated videos/logs are ignored under `output/`; some retain earlier titles and are not included in source/wheel distributions. Their source footage was not imported into the tracked tree. Review any future media separately before publishing it.

## Subsequently added README previews

At the user's request, two project-generated GIF previews are now included under `docs/media/` and selected for the source distribution. They are conversions of the newly branded miniature and experimental full-size recordings made by `scripts/record_halfpipe.py`, with the complete air-control replay retained. They contain only the locally generated primitive scene and telemetry; no manufacturer imagery or presentation footage was used. Source MP4s, CSV and older renderings remain ignored. The GIFs reduce frame rate/color detail without changing the recorded playback speed. See [media provenance and regeneration](media/README.md). This addition changes the earlier text-only publication payload; the earlier audit counts below describe that audit's scope, not the current file count.

Two additional project-generated tornado race previews are now included for the miniature and full-size demos, bringing the README total to four GIFs. They come from `scripts/record_tornado.py`, retain the labelled slow replay and show cumulative thruster gas use. Their vehicles, scripted pace cars and course use the same locally generated primitive geometry.

## Security and privacy

The historical scan covered private-key markers, common cloud/GitHub token patterns, credential assignments, personal absolute paths and source email addresses. No secret or personal local path was identified in the tracked current or historical source. Apparent email-like matches in Python matrix multiplication expressions were false positives. Pattern-based scans cannot prove that every possible secret is absent.

**Human decision required:** the Git author email remains in commit metadata. The user confirmed the public copyright-holder name, but publication of the retained email address still needs a privacy decision.

At the user's request, all three commits on `main` were rewritten to remove retired project/product names from historical file paths, contents and commit messages. The public refs' 32 distinct blobs / 50 historical file versions were rechecked; no retired names remain there. Author/committer identities and dates were preserved, as were all historical JSON parameters. A private recovery snapshot was saved outside the checkout; current uncommitted source files were verified byte-for-byte before and after the branch replacement. Commit IDs changed:

| Previous commit | Rewritten commit |
|---|---|
| `9dc546e` | `39baf3e` |
| `93fb657` | `7f1021f` |
| `cd53087` | `03ce7fc` |

**Local recovery cleanup completed with explicit user approval:** the user separately approved deleting the old history and reflogs. After verifying the external recovery copy, `ORIG_HEAD` was removed, all reflogs expired, and unreachable Git objects pruned. All three original commit objects are now unavailable in this repository. A full object-store scan found no retired names, and `git fsck --full --no-reflogs` passed. Current uncommitted source files were again verified unchanged. The private recovery snapshot outside the checkout is intentionally retained and must not be published. No remote change, push, release or publication was performed.

`.gitignore` excludes generated output, caches, environments, build products, editor files, local agent settings and common credential files. Ignore rules are preventive; they do not remove already committed content. Packaging explicitly selects the new Python package/configuration files and excludes local output and dependency binaries. The existing local checkout directory name is not embedded into distributable source.

## License status: MIT; holder confirmed

On 2026-10-18, the user confirmed `keg66` as the copyright-holder name and selected MIT for project-owned material. The year 2026 is supported by Git commit dates. The root `LICENSE` now contains the standard MIT text and `Copyright (c) 2026 keg66`; the temporary draft has been removed. This records the user's ownership/license instruction rather than inferring a holder from author metadata.

The grant covers the project's own code, configurations, documentation and generated README previews. English/Japanese README and media notices link to the final license. Package metadata declares the SPDX expression `MIT` and includes the final license and third-party notice file in distributions. Copyright-holder confirmation is resolved; the retained Git author email still needs the publication/privacy decision noted above.

Dependencies are separately installed and retain their own licenses; see [THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md). In particular, NumPy wheels may contain GCC runtime and libquadmath components under GPL with runtime exception / LGPL terms. Project MIT licensing does not relicense dependency binaries or third-party material.

## References and claims

Only verified official Tesla product and diecast-model pages are linked in README. The exact official URL of the supplied loop demonstration could not be verified; no video URL was invented. Full-size dimensions were inferred from the 1:18 model dimensions, not asserted as production specifications. All propulsion, fan, mass, tank and control inputs are experimental assumptions. No real vehicle feasibility, affiliation or access to proprietary engineering is asserted.

## Verification

Verified on Ubuntu, Python 3.14.4, MuJoCo 3.16.0, NumPy 2.5.4 and pytest 9.1.1:

- Built a source distribution and a pure-Python wheel with `python -m build --no-isolation`. The wheel is built from the sdist, checking that its included configuration files suffice.
- After the user confirmed MIT and the holder, rebuilt the distributions and checked `License-Expression: MIT`, the final LICENSE and third-party notice file, with no obsolete draft shipped. Runtime code and numerical inputs were unchanged.
- Installed the wheel plus separately obtained dependencies into a fresh virtual environment. Ran from outside the source checkout and checked that imports resolve to that environment's `site-packages`.
- All existing tests passed: **33 passed in 44.39 s** against the isolated wheel.
- Ran all five CLI demos to their full default duration. Ceiling retention error before shutdown was below 0.001 mm, followed by release/fall. The lift reached 0.20486 m and consumed 1.4889 g of gas. Downforce contact load rose from 1.1772 N to 4.70754 N with approximately 3.53034 N suction, returning to the initial load after fan shutdown.
- Miniature half-pipe: `landed`, no failure, touchdown 0.050212 m/s, gas used 29.5327 g. Full-size: `landed`, no failure, touchdown 0.281554 m/s, gas used 451.9012 kg. Full-size minimum roll/recovery height was 7.23796 m.
- The new console command and module entry point worked. A 0.2 s X11 MuJoCo Viewer run completed. Both vehicle scales rendered through EGL, and FFmpeg accepted the new title. Existing camera/slow-replay edits were preserved. In a subsequent user-requested recording pass, the ceiling/lift clips and both half-pipe sequences were regenerated with the new branding under ignored `output/videos/suction-thrust-lab/`. Final durations are 30.0 s for the basic combined movie, 49.8 s for the miniature roll, and 98.4 s for the full-size roll, including slow replays. Full decoding, subtitle/output-name/metadata scans, sampled-frame inspection, and five-pose side-view visibility checks passed for the new recordings. The preview-only segmentation check disables antialiasing to avoid blending discrete object IDs; recording and physics are unchanged.
- Compared every configuration value against the preceding commit: all physical numbers are unchanged; only the full-size profile's generic vehicle label changed. Python executable ASTs match after identifier/text normalization, and remaining text diffs were reviewed as docstrings/model display names. No physics/control changes were made.
- Inspected wheel/sdist member paths and contents and the 25 current source candidates. No retired project names, personal absolute paths, credential-pattern matches, binary/media assets, output directories or dependency binaries were present. All local Markdown links resolved. `git diff --check` passed. This initial audit preceded the subsequently authorized history rewrite described above; after cleanup, only the three sanitized commits remained, with local recovery reflogs/old objects removed after explicit approval. Subsequent publication-preparation changes are committed separately by topic. A post-rewrite regression run also passed all 33 tests in 43.21 s.

These are regression and packaging checks, not empirical validation or a legal opinion. The distributions are local verification artifacts, not published releases. The project license is now MIT with the confirmed holder, as described above.

## Change inventory

- Renamed the Python package to `suction_thrust_lab`, distribution/entry point to `suction-thrust-lab`, and the experimental full-size module/config/demo to `full_size`. Updated imports and test names; old aliases are intentionally absent from the public source.
- Updated both recording scripts' imports, identifiers and displayed branding, preserving the existing camera and replay behavior.
- Replaced README/ROADMAP with English scope, status, setup, models, configuration, inspiration, disclaimer and license information. Added `docs/PHYSICS.md`, this review and `THIRD_PARTY_NOTICES.md`.
- Updated `pyproject.toml`, added `MANIFEST.in`, and expanded `.gitignore`. The initial MIT draft was replaced by the final root LICENSE after the user confirmed the holder and license. Runtime dependency requirements and numerical inputs were preserved.

## Before public release

- MIT and the copyright-holder display are finalized; keep the LICENSE and dependency notices when distributing the project.
- Approve publication of the retained Git author email. Historical-name sanitization and local old-object/reflog cleanup are complete; do not include the external private recovery snapshot.
- Review this provenance assessment and any newly added assets/contributions.
- Choose/create the intended repository and review the final public file list. No remote repository URL is assumed here.
- Resolve any verification failure recorded below before release.
