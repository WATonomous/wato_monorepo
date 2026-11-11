# SimForge prototype adapter

Start with [the execution handoff](../../../docs/SIMFORGE_NEXT_AGENT_HANDOFF.md).
This package is an additional isolated simulator backend. It requires a locally
installed authenticated SimForge runtime and measured state/control contracts.
No runtime or WATO behavior integration has yet been demonstrated.

The `simforge_probe` and `simforge_smoke` tools work without ROS. The ROS bridge
publishes ground-truth observations and waits for a command bearing the current
observation stamp before advancing one step. The diagnostic driver is only a
plumbing check. WATO mode requires reviewed actor truth and scenario authority.
Full asynchronous pipeline provenance remains a separate integration gate.

Build: `bash watod_scripts/simforge/sf build` from the checkout root.
Run tests: `bash watod_scripts/simforge/sf run python -m pytest /repo/src/simulation/sim_forge/test -q`.
Local runtime data and results live in ignored `.simforge/`.
