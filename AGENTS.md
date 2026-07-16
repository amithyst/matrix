# Matrix Development Contract

## Source Of Truth

- The canonical repository is `https://github.com/amithyst/matrix`.
- `main` is the stable integration branch. Use short-lived feature branches on
  both TRNA and Heyuan; do not create long-lived machine-specific branches.
- Runtime binaries, models, release archives, generated robot files, logs, and
  recordings belong below ignored runtime/cache directories. Do not commit them.

## Reproducible Runtime

- `config/runtime/matrix-sonic.lock.json` is the version and SHA256 authority.
- `config/hosts/trna.env` and `config/hosts/heyuan.env` contain non-secret host
  defaults. Put local overrides in `.matrix/local.env`.
- Bootstrap with `scripts/bootstrap_matrix_sonic.sh`; launch through
  `scripts/run_matrix_sonic*.sh --profile <host>`.
- Build or refresh private runtime artifacts with
  `scripts/package_matrix_sonic_artifacts.sh`; never publish them to this public repo.
- Do not bypass the runtime verifier for an acceptance run.

## Verification

Run the focused checks before pushing:

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
bash -n scripts/*.sh scripts/release_manager/*.sh
python3 scripts/verify_matrix_sonic_runtime.py --schema-only
```

GPU/runtime acceptance must also prove no fall, no numerical reset, the locked
TensorRT ABI, physics frequency, real-time factor, and cleanup of child processes.
