# WATO lifecycle manager

The C++ lifecycle manager owns configure, activate, pause and shutdown operations
for a declared group of lifecycle nodes. Its existing interface is unchanged.

Simulation launch-process ownership and presets live in the separate
[`simulation_runtime`](../../simulation/simulation_runtime/README.md) package.
