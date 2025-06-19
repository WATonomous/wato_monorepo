# Docker Context

Tiny helper script that **scans every `docker-compose.*.yaml` file under
`modules/`**, builds a **deduplicated GitHub Actions matrix** of Dockerfiles to
test, and emits registry information for downstream jobs.

---

## Why it exists 💡
* **Faster CI** – each unique Dockerfile is built exactly once, even if multiple
  services share it.
* **Automatic scope** – only modules touched in the PR (or infrastructure-wide
  changes) are included.
* **Debuggable locally** – run the script with `--debug` for a pretty report
  without needing GitHub.

---

## Requirements
| Tool | Version | Notes |
|------|---------|-------|
| Bash | 4.x+ | Uses associative arrays and `set -euo pipefail` |
| Docker Compose | v2+ | Needed for `docker compose config --format json` |
| jq   | 1.6+ | JSON wrangling, installed by default on Microsoft runners. |

---

## Usage

### In CI (GitHub Actions)

```yaml
name: Generate Docker Environment

inputs:
  modified_modules:
    description: "Space deliminated list of modified modules"
    required: true
    default: ''

outputs:
  docker_matrix:
    description: "list of docker compose services"
    value: ${{ steps.environment-generator.outputs.docker_matrix }}
  registry:
    description: "name of the docker registry we are using"
    value: ${{ steps.environment-generator.outputs.registry }}
  repository:
    description: "name of the docker repository we are using"
    value: ${{ steps.environment-generator.outputs.repository }}

runs:
  using: "composite"
  steps:
    - id: environment-generator
      env: 
        MODIFIED_MODULES: ${{ inputs.modified_modules }}
      run: ${{ github.action_path }}/docker_context.sh
      shell: bash
```

## Testing Procedure

You will need to install `jq` locally to test the script.
```bash
sudo apt-get update
sudo apt-get install -y jq
```

After that, you can run the test script like so:
```bash
cd <monorepo_directory_root> # only run the script if you are at the root of the monorepo
# Pretend “perception” and “action” were the modules changed in your branch
MODIFIED_MODULES="perception action infrastructure" ./.github/templates/docker_context/docker_context.sh --debug
```