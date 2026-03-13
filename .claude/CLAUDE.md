# CLAUDE.md - Testing Guide for the WATonomous Monorepo

## Project Overview

ROS 2 monorepo for WATonomous, fully Docker-based. Six module groups: **infrastructure**, **interfacing**, **perception**, **world_modeling**, **action**, **simulation**. All orchestrated by `watod`, a CLI wrapper around `docker compose`.

## Quick Reference

| Task | Command |
|------|---------|
| Run all tests | `watod test` |
| Test one module | `watod test perception` |
| Test specific packages | `watod test world_modeling prediction` |
| Lint/format everything | `pre-commit run --all-files` |
| Build active modules | `watod build` |
| Build and start | `watod up` |
| Open shell in container | `watod -t perception_bringup_dev` |

## Running Unit Tests with `watod test`

```bash
watod test                                # all active modules
watod test <module>                       # single module (e.g., perception)
watod test <module> <pkg> [pkg...]        # specific packages within a module
```

**How it works:** Builds `{module}_test` profile images (Docker target: `build` stage), then runs `colcon test` inside the container with `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` and isolated `ROS_DOMAIN_ID=99`.

**Perception caveat:** In test/CI, GPU-dependent packages (`deep_ort_gpu_backend_plugin`, `deep_sample`) are automatically skipped via `COLCON_TEST_PACKAGES_SKIP_ARGS`.

**Infrastructure:** Build-only in CI (no tests). See `.github/workflows/build_and_unitest.yml`.

## Running Tests Inside a DevContainer

1. Set module to dev mode in `watod-config.local.sh`:

   ```bash
   ACTIVE_MODULES="perception:dev"
   ```

2. Start containers: `watod up`
3. Attach: `watod -t perception_bringup_dev` or use VSCode DevContainers extension
4. Inside the container:

   ```bash
   cd /ws
source /opt/watonomous/setup.bash
   colcon build --packages-select <pkg>
   colcon test --packages-select <pkg> --event-handlers console_direct+
   colcon test-result --verbose

   ```

## Writing Tests

### C++ (Catch2) - Using `wato_test`

The `wato_test` package provides the `add_wato_test()` CMake macro, which automatically links Catch2, rclcpp, and rclcpp_lifecycle.

**CMakeLists.txt usage:**

```cmake
find_package(wato_test REQUIRED)
add_wato_test(test_my_node
  test/test_my_node.cpp
  LIBRARIES
    ${my_msgs_TARGETS}
)
```

**Key resources in `src/wato_test/`:**
- `include/test_fixtures/` - Test executor fixture (manages rclcpp init/shutdown and spinning)
- `include/test_nodes/` - Helper nodes: `publisher_test_node`, `subscriber_test_node`, `service_test_node`, `client_test_node`
- `include/wato_test/wato_test.hpp` - Main include header

**Best practices:**
- Use `wait_for_subscribers()` / `wait_for_publishers()` instead of sleeps for DDS discovery
- For lifecycle nodes, set parameters **before** activation to avoid race conditions with executor callbacks

### Python (pytest)

Configure in `setup.cfg` with `testpaths = tests`, pattern `test_*.py`. Use `unittest.mock` for external dependencies.

## Linting and Formatting (pre-commit)

**Setup:**

```bash
sudo apt-get install -y libxml2-utils
pip install pre-commit
pre-commit install
```

**Run all checks:** `pre-commit run --all-files`

| Language | Formatter | Linter | Config |
|----------|-----------|--------|--------|
| Python | ruff-format | ruff | (default) |
| C++ | clang-format | cpplint | `.config/clang-format`, `.cpplint.cfg` |
| CMake | - | cmakelint | 140 char lines |
| Shell | - | shellcheck | ignores SC1091 |
| Docker | - | hadolint | ignores SC1091, DL3006, DL3008 |
| Markdown | - | pymarkdown | ignores MD013 |
| XML | - | ament_xmllint | - |
| YAML | - | yamllint | `.config/.yamllint.yaml` |

License headers are auto-inserted: `#` style for Python/CMake/Shell, `//` style for C/C++. Template: `.config/copyright.txt`.

## CI/CD Pipeline

What runs on PRs to `main`:

- **`build_and_unitest.yml`** - Detects changed modules, matrix-builds each, runs `watod test`. Infrastructure is build-only (no tests).
- **`pre-commit.yml`** - Runs all pre-commit hooks.
- **`block_watod_config_change.yml`** - Blocks direct edits to `watod-config.sh`. Use `watod-config.local.sh` instead.

## Module Configuration

- Edit `watod-config.local.sh` (git-ignored) to set `ACTIVE_MODULES`
- **Never** edit `watod-config.sh` directly (CI will block the PR)
- Module variants: `module`, `module:dev`, `module:bag`
- Shorthand: `all`, `all:dev`

Available modules: `action`, `interfacing`, `perception`, `world_modeling`, `simulation` (plus `infrastructure` which is always added automatically).

## Building

```bash
watod up      # build and start active modules
watod build   # build without starting
```

Builds use multi-stage Dockerfiles: source -> deps -> rosdep_install -> build -> deploy/develop.
