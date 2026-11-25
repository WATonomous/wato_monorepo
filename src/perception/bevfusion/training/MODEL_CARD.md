# {{VERSION}}

- Source checkpoint: `{{CHECKPOINT}}`
- Init, layers trained, epochs: _fill in from the training run_
- PTQ: `{{PTQ}}`
- Monorepo commit: `{{MONOREPO_COMMIT}}`
- wato-cuda-bevfusion commit: `{{FORK_COMMIT}}`
- Exported (UTC): `{{EXPORTED_UTC}}`
- nuScenes val mAP/NDS, PyTorch: _fill in after evaluation_
- INT8 (PTQ fake-quant, PyTorch) mAP/NDS: _fill in after evaluation_
- Deploy check (bag, output Hz, notes): _fill in after deployment testing_

## Known limits

- The training and perception images both use wato-cuda-bevfusion `08ec4db`.
- TensorRT engines are tied to the GPU and TensorRT version. Never copy `build/` between machines or versions.
- The export bakes six cameras into `camera.backbone.onnx`.
