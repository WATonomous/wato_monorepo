# {{VERSION}}

- Source checkpoint: `{{CHECKPOINT}}`
- Init, layers trained, epochs: TODO (from the training hand-off)
- PTQ: `{{PTQ}}`
- Monorepo commit: `{{MONOREPO_COMMIT}}`
- wato-cuda-bevfusion commit: `{{FORK_COMMIT}}`
- Exported (UTC): `{{EXPORTED_UTC}}`
- nuScenes val mAP/NDS, PyTorch: TODO
- INT8 (PTQ fake-quant, PyTorch) mAP/NDS: TODO
- Deploy check (bag, output Hz, notes): TODO

## Known limits

- The training image clones wato-cuda-bevfusion without pinning a commit. The perception image pins `08ec4db`.
- TensorRT engines are tied to the GPU and TensorRT version. Never copy `build/` between machines or versions.
- The export bakes six cameras into `camera.backbone.onnx`.
