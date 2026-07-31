# Deployment / Operations Handoff

## Implemented

- Added `epuzzle-gui.service`, attached it to `epuzzle.target`, made it wait for and require preflight, and configured tty2 plus unprivileged `epuzzle` access to `video input render dialout gpio`.
- Reworked `start_pick_place_page.sh` to use the packaged venv, require a systemd/local VT context, and avoid all `sudo`/`openvt` privilege escalation.
- Changed Supervisor service to call the future canonical entrypoint `/opt/epuzzle/workspace/src/integration/run_supervisor.py` directly. Removed the `Condition*` rule that caused a missing entrypoint to be silently skipped.
- Added explicit build gates: missing canonical Supervisor source, frozen calibration artifacts, or a unique SHA256-covered `pygame-ce==2.5.5` aarch64 wheel now stop image construction visibly. No image was built.
- Added pygame-ce requirement, pygame import to offline reinstall, and pygame version/import checks to selfcheck. The docs explicitly require aarch64 wheelhouse resolution and target import validation rather than claiming wheel availability.
- Aligned selfcheck calibration validation to the frozen 20260731 manifest/artifact names and SHA256 values. The previously absent accepted kinematic and contact-sheet artifacts are included in this isolated copy to make the manifest complete.
- Preserved `parameters_frozen` as an error-level `UNKNOWN` gate. Added registry/environment checks that ACT, generic vision, texture, and magnet feature gates are all disabled. `EPUZZLE_MAGNET_ENABLED=0`; no magnet code was activated.
- Updated deployment README and `docs/18_树莓派5部署.md` with systemd order, no-sudo GUI behavior, reproducible pygame arm64 policy, target import/fb0/touch acceptance, and local-VT/known-good-image fallback.
- Added nine development-machine tests for units, shell behavior, requirements/build gates, frozen calibration validation, feature-gate fail-closed behavior, and no-sudo GUI launch behavior.

## Changed files

- `configs/parameter_registry.yaml`
- `deployment/raspberry_pi/build-image.sh`
- `deployment/raspberry_pi/requirements-arm64.in`
- `deployment/raspberry_pi/README.md`
- `deployment/raspberry_pi/overlay/etc/epuzzle/epuzzle.env`
- `deployment/raspberry_pi/overlay/etc/systemd/system/epuzzle.target`
- `deployment/raspberry_pi/overlay/etc/systemd/system/epuzzle-preflight.service`
- `deployment/raspberry_pi/overlay/etc/systemd/system/epuzzle-supervisor.service`
- `deployment/raspberry_pi/overlay/etc/systemd/system/epuzzle-gui.service` (new)
- `deployment/raspberry_pi/overlay/usr/local/sbin/epuzzle-selfcheck`
- `deployment/raspberry_pi/overlay/usr/local/sbin/epuzzle-offline-reinstall`
- `docs/18_树莓派5部署.md`
- `workspace/tools/start_pick_place_page.sh`
- `workspace/tests/test_deployment_contracts.py` (new)
- `workspace/calibration/arm_kinematics_single_arm_accepted_20260731.json` (frozen source artifact copied into isolated checkout)
- `workspace/calibration/contact_sheet_to_robot_base_20260731.json` (frozen source artifact copied into isolated checkout)

## Validation

Passed:

```bash
bash -n deployment/raspberry_pi/build-image.sh \
  deployment/raspberry_pi/flash-image.sh \
  deployment/raspberry_pi/overlay/usr/local/sbin/epuzzle-offline-reinstall \
  workspace/tools/start_pick_place_page.sh
python -m compileall -q workspace/src workspace/tools workspace/apps workspace/tests \
  deployment/raspberry_pi/overlay/usr/local/sbin \
  deployment/raspberry_pi/overlay/usr/local/lib
ruff check workspace/src workspace/tools workspace/apps workspace/tests \
  deployment/raspberry_pi/overlay/usr/local/sbin \
  deployment/raspberry_pi/overlay/usr/local/lib
python -m unittest workspace.tests.test_deployment_contracts -v
```

Result: syntax validation passed; compileall passed; ruff returned `[]`; deployment contracts ran 9 tests, all `OK`.

Additional passed static checks:

- JSON parsing for local calibration JSON.
- Environment feature gates asserted to `0`.
- Systemd unit structural assertions and no-`sudo` assertions.
- `shellcheck` is unavailable on this development machine; `bash -n` was used as the safe static substitute.

Attempted but not a meaningful target validation:

```bash
systemd-analyze verify ...
```

It returned nonzero only because host-side systemd cannot resolve image paths such as `/opt/epuzzle/venv/bin/python` and `/usr/local/sbin/epuzzle-selfcheck`; unit structure is covered by local static tests. It did not start any service.

The existing full suite was also run. Deployment tests passed, but overall discovery remains non-green due to isolated-checkout pre-existing artifacts/imports outside this task: absent `workspace/calibration/evidence/2026-07-31_single_arm_contacts_v2/...`, absent `arm_kinematics_legacy_rejected_20260731.json`, and unavailable `lerobot_robot_so101_5d` import. No production/control/plugin source was modified to conceal these failures.

## Residual risks / required Pi gates

1. `workspace/src/integration/run_supervisor.py` is still absent in this isolated copy. The new build gate intentionally rejects an image until the Supervisor worker supplies it; this is fail-visible rather than a silent systemd skip.
2. The wheelhouse was deliberately not rebuilt and no image was built/flashed. Rebuild pygame-ce and all arm64 dependencies in aarch64 CPython 3.13, update wheel SHA256, and verify import on rootfs/Pi.
3. On Pi 5, run `epuzzle-selfcheck --json`, verify `import pygame`, systemd dependency/failure behavior, `/dev/fb0` non-black/color output, QDTECH touch hit-testing, tty2 ownership, and GUI restart behavior.
4. Full preflight must currently remain failed until actual camera udev attributes are frozen and registry `UNKNOWN` values are measured. This is intentional; do not delete gates or substitute estimates.
5. Validate that changing the safe default to `EPUZZLE_MAGNET_ENABLED=0` is propagated to the final Supervisor configuration; do not turn it on before PICK/PLACE and other grasp gates are accepted.

## Repository state

This assigned isolated directory is not a Git worktree, so it has no Git index or staged files. `git rev-parse --is-inside-work-tree` failed and no staging action was performed. Deployment source was copied from `/home/gxxl/nudec-re101` as read-only baseline only; cache/build/output/images/keys were not copied or modified.