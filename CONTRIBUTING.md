# Contributing

## Commit messages

[Conventional Commits](https://www.conventionalcommits.org/):

```
<type>(<scope>): <short summary in imperative mood>
```

| type | use for |
|---|---|
| `feat` | new behaviour (node, topic, parameter) |
| `fix` | bug fix |
| `refactor` | code change without behaviour change |
| `docs` | README / comments only |
| `test` | tests only |
| `ci` | GitHub Actions |
| `chore` | repo housekeeping (.gitignore, versions) |

Scopes: `driver`, `safety`, `motion`, `workshop`, `bringup`.
Breaking changes: `feat!:` / `refactor!:` plus a `BREAKING CHANGE:` footer.

## Before pushing

```bash
flake8 --max-line-length 100 src
PYTHONPATH=src/lite3_driver pytest -q src/lite3_driver/test
PYTHONPATH=src/lite3_safety pytest -q src/lite3_safety/test
PYTHONPATH=src/lite3_motion pytest -q src/lite3_motion/test
colcon build --symlink-install
```

Keep protocol and math in pure modules (`protocol.py`, `guard_logic.py`, `plan.py`)
so they stay testable without ROS. Never call `time.sleep()` inside a ROS callback.
