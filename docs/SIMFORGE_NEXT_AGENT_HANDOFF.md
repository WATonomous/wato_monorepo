# SimForge prototype: execution handoff

Prepared 2026-11-11. Branch: `stephen/simforge-prototype`.
Base: `origin/main`, commit `f6590d7c677548939c5c142620233ddb204662a9`.

## Scope and honest status

The user requested implementation on a Mac, a pushed branch, and this handoff for a stronger machine. No Docker container, simulation, rendering, or WATO stack was run on the Mac. The source is ready for the installation and integration gates below. **No live-runtime or WATO behavior verdict has been established.** Do not equate passing unit tests with working driving integration.

Read [the original specification](simforge_original_handoff.md), sections 1 through 12. Its appendix is the original starter, not the current source of truth. Current tracked files supersede the appendix. The original document's reference to seven tests describes its preparation environment.

Implemented:

- Isolated CPU ROS 2 Jazzy adapter image, CycloneDDS, Compose runner and optional NVIDIA rendering override.
- Pinned `simforge-gym==0.2.0`, official task materialization, runtime probe and same-seed smoke comparison.
- Reviewed coordinate/control contract gates, odometry, pose, TF, clock and verified actor-box mapping.
- Timestamp-matched diagnostic/WATO command loop with paused stepping and fail-fast command envelope checks.
- Raw observations, mapped actors, commands, events and termination recorded per tick.
- Collision/duration/travel scorer with JSON and JUnit output. Diagnostic, unavailable-event, truncated, nonfinite-threshold and stationary cases cannot masquerade as a passing driving test.
- Launch generation from the actual repository launch files, with command/detection remaps, one simulation-time setting per node, ground-truth localization and combined launch copy.
- Fresh-process suite orchestration, stage liveness monitoring, timeouts, process-group cleanup, optional explicit container cleanup, and incremental suite results.
- Provenance collector for source status, dependencies, scenario file digests, image identity and installed CLI inspection output. The adapter image includes Git so source provenance can be captured from its mounted checkout.

No fabricated map, runtime field indices, coordinate semantics, actor truth, command authority, obstacle cloud or pipeline barrier was added. Those require runtime inspection and experiments. All contract review flags remain false.

## First commands on Ubuntu 24.04 x86_64

```bash
git fetch origin
git switch stephen/simforge-prototype
git pull --ff-only origin stephen/simforge-prototype
bash watod_scripts/simforge/sf build
bash watod_scripts/simforge/sf run python -m pytest /repo/src/simulation/sim_forge/test -q
bash watod_scripts/simforge/sf run bash
```

Inside the container, follow original sections 2 through 4 to authenticate, select an actually available runtime version, install CPU assets and the pinned Richmond v9 map, and materialize `lead-hard-brake`. Account sign-in must be completed by the account owner. Do not store credentials in Git. `.simforge/` is ignored and excluded from the image build context.

CLI installation commands from the original spec:

```bash
curl -fsSL https://simforge.ai/install.sh -o /tmp/simforge_install.sh
sh /tmp/simforge_install.sh
simforge --version
simforge login --device
simforge runtime status --available
```

Select a version from that actual output, then install using the original commands. Do not assume a CLI/runtime version from a web page. The source SDK test covers the published Python wheel, not compatibility with an authenticated server.

Use unique socket and output paths. The server must be running before probe/smoke. Both tools use authored scenario actions at this stage and explicitly do not evaluate WATO driving.

```bash
# Terminal A, checkout root on host:
bash watod_scripts/simforge/sf run simforge env serve \
  /repo/.simforge/workspaces/lead_brake_authored \
  --socket /run/simforge/env.sock --no-sensors --max-sessions 1

# Terminal B, checkout root on host:
bash watod_scripts/simforge/sf run ros2 run sim_forge simforge_probe \
  --socket /run/simforge/env.sock --steps 5 --out /repo/.simforge/results/probe_001
bash watod_scripts/simforge/sf run ros2 run sim_forge simforge_smoke \
  --socket /run/simforge/env.sock --seed 7 --steps 100 --out /repo/.simforge/results/smoke_001
```

If the server limits completed client sessions, restart it between probe and smoke after verifying the old process stopped. Do not remove a socket belonging to a live process.

## Runtime-dependent work, in order

1. Capture handshake/reset/steps. Determine actual JSON pointers, units, axes, yaw convention, origin, steering sign and scale. Use known straight motion and left/right turns. Copy `src/simulation/sim_forge/config/contract.template.json` to `.simforge/contract.json`; set review flags only after evidence supports them.
2. Prove diagnostic stop, straight, left and right controls. Run `simforge_diagnostic_driver` with ROS parameters `speed_mps` and `steering_rad`, then `simforge_ros_bridge --controller-kind diagnostic`. See original section 6 for complete bridge arguments.
3. Inspect the installed SDK/runtime live truth interface. Configure `actors.list`, `actors.fields` and `actors.class_map` from observed records. Verify geometry and signal semantics. Duplicate actor IDs fail closed. Actor boxes are expected to describe their centers, separately from the ego reference offset.
4. Obtain/convert the exact matching Lanelet2 map, including topology, projection, stop lines and signals. Validate landmarks and box/lane overlays. Investigate existing repo map utilities first. Existing CARLA maps do not establish alignment with Richmond.
5. Supply the costmap obstacle pointcloud or explicitly implement and validate an object occupancy path. The generated launch retains `/perception/non_ground_cloud`; this dependency is intentionally visible. Empty data does not prove obstacle avoidance.
6. Build/source action and world-modeling development containers. Verify CycloneDDS is installed in both and match ROS domain 183 (or a deliberately selected free domain). Adapt host networking for WatCloud rootless shared namespaces. Do not launch CARLA simultaneously.
7. Generate launches with a measured map config. Generator paths are resolved in the environment where it runs: run inside `/repo` or supply paths valid in WATO containers. It now also writes `combined_simforge.launch.py`. The minimal adapter image cannot launch WATO packages. Mount each generated launch/map into its corresponding development container; use the combined launch only with a common built ROS environment.
8. Set a real route goal, publish EVE description if needed, audit unique TF/clock publishers, remove ego authored assistance in a copied scenario, and prove command authority before setting `scenario_control_reviewed`.
9. Implement and verify full pipeline input provenance before claiming whole-stack deterministic CI. The MPC currently stamps its command with `now()` in `src/action/mpc_controller/src/mpc_controller_node.cpp`; that alone does not establish that perception/truth, world model, behavior, planning and control processed the same tick. The adapter only rejects mismatched command stamps.
10. Verify actual collision event names and actor identities before scoring. The scorer checks all matching collision events, not an ego-specific collision count. Add scenario-specific gap, stopping, conflict-zone and lane assertions when claiming that coverage.

Do not set review flags merely to get past a startup exception. If an interface is absent, record the actual command/output and missing capability, and retain the last demonstrated gate.

## Evidence and lifecycle

After assets and a scenario are installed, capture provenance inside the adapter:

```bash
python /repo/watod_scripts/simforge/collect_evidence.py \
  --repo /repo --workspace /repo/.simforge/workspaces/lead_brake_authored \
  --out /repo/.simforge/results/environment_001
```

After contract discovery also pass `--contract /repo/.simforge/contract.json`. Obtain the image ID with host-side `docker image inspect`, then pass it using `--image-id`. Additional release-specific inspection commands may be supplied with `--command-json '["simforge", "...actual inspection subcommand..."]'`. Record exact runtime/map digests and machine-readable help using the actual installed CLI; the collector cannot infer them. CLI failures are retained in `provenance.json`, never treated as passing checks.

Start automation only after manual gates pass. `suite.template.json` remains deliberately unreviewed and contains an unverified collision-kind placeholder. Its stack command requires both built WATO workspaces. For an externally supervised Compose stack, supply a per-case `cleanup_argv` string array that explicitly tears down the containers. It runs even after failure and must be idempotent. Killing a Docker CLI process does not kill detached containers. Validate crash cleanup, fresh state for the next case, timeout behavior and repeatability on the real host.

Suite and bridge output directories must be fresh. The runner creates each case directory; the bridge accepts it only when no prior episode artifacts are present. Each case uses a fresh socket and stack process group. Preserve failed case logs. Suite exit 1 means one or more cases failed or could not evaluate; inspect per-case status to distinguish driving `failed` from infrastructure `error`. Scorer exits: 0 pass, 1 driving assertion failure, 2 evaluation error.

## Mac validation already performed

Python 3.12.15, published SDK 0.2.0, pytest 8.3.5, PyYAML 6.0.3. Nineteen tests passed, including the actual SDK against a transport fixture. Python compile and Bash syntax checks passed. Source distribution and wheel built successfully; the wheel installed and its probe entry point ran. The official SDK template materialized `lead-hard-brake@1e29b9bd` with the pinned Richmond v9 digest, without starting simulation. No simulation compute was performed.

Reproduce lightweight checks without ROS, using an isolated Python 3.12 environment:

```bash
python -m pip install -r src/simulation/sim_forge/requirements.txt pytest==8.3.5 PyYAML
PYTHONPATH=src/simulation/sim_forge python -m pytest src/simulation/sim_forge/test -q
```

Tests cover SDK action slots and events, coordinates/reflection/reference offsets, missing state, control limits, actor mapping/signals/IDs, scoring false-pass cases, repeatability, generated launches, failed-server cleanup, premature stack exit and real subprocess score-exit classification. They do not cover Docker build, DDS delivery, runtime schema, map alignment, actual collision semantics or WATO command provenance.

Finish with an original-spec verdict: runtime smoke, ROS control prototype, WATO behavior prototype, blocked with exact evidence, or unsuitable with a demonstrated limitation. Include versions, commands, artifact paths, failed gates and one reproducible command. Do not claim perception, SLAM, or calibrated EVE dynamics from this ground-truth prototype.
