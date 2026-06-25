# MTR Model I/O Verification

Date: 2026-06-25
Verifier: Codex, using official MTR source inspection

## Status

Task 0 is partially completed:

- Official repository obtained: `https://github.com/sshaoshuai/MTR.git`
- Source revision inspected: `a5ba7bdafa09a1a355cc34f8a895499a2b14ddb3` (`bugfixed: multi-gpu evaluation bug (#63)`)
- Static source-level tensor/key verification completed.
- Runtime forward-pass dump not completed in this checkout because neither the host shell nor cached ROS container has `torch`, and the required processed Waymo sample/checkpoint is not present.
- TensorRT binding dump not completed because the official repo has no ONNX/TensorRT export path and no engine artifact is present.

This document is therefore evidence for source-level contract amendments, not final A/B/C sign-off that a deployed engine binding contract is frozen.

## Evidence Commands

```bash
git clone --depth 1 https://github.com/sshaoshuai/MTR.git /private/tmp/mtr-task0
git -C /private/tmp/mtr-task0 rev-parse HEAD
rg -n "obj_trajs|track_index_to_predict|center_objects_type|map_polylines|pred_scores|pred_trajs|input_dict|pred_dicts" /private/tmp/mtr-task0 -S
python3 - <<'PY'
try:
    import torch
    print('torch', torch.__version__, 'cuda', torch.cuda.is_available())
except Exception as e:
    print('NO_TORCH', type(e).__name__, str(e))
PY
```

The same torch probe was run inside the cached `ghcr.io/watonomous/wato_monorepo/world_modeling/world_modeling:dep_main` container and also returned `NO_TORCH ModuleNotFoundError`.

## Source-Level Tensor Contract

Names below are from the official MTR Waymo dataset/model path.

| Tensor/key | Official source shape | Official source dtype/semantic | Current skeleton field | Delta |
|---|---:|---|---|---|
| `obj_trajs` | `[N, O, H, 29]` | `float32` | `std::vector<float> obj_trajs`, `obj_trajs_shape` | No field-type delta. `H=11` for Waymo history; feature dim verified from source and config. |
| `obj_trajs_mask` | `[N, O, H]` | `bool` | `std::vector<uint8_t> obj_trajs_mask` | Amended from `float`; store 0/1 bool bytes for contiguous C++ buffer. |
| `obj_trajs_last_pos` | `[N, O, 3]` | `float32` | `std::vector<float> obj_trajs_last_pos` | No delta. |
| `track_index_to_predict` | `[N]` | integer index tensor | `std::vector<int32_t> track_index_to_predict` | No source-level blocker; engine export should confirm exact integer width. |
| `center_objects_type` | `[N]` | object-type token (`TYPE_VEHICLE`, `TYPE_PEDESTRIAN`, `TYPE_CYCLIST`) used by decoder | `std::vector<std::string> center_objects_type` | Amended from `int32`; official PyTorch path uses object-type tokens, not a numeric tensor. TensorRT export must define how these are encoded. |
| `map_polylines` | `[N, P, 20, 9]` | `float32` | `std::vector<float> map_polylines`, `map_polylines_shape` | No field-type delta. `P` defaults to 768 source polylines. |
| `map_polylines_mask` | `[N, P, 20]` | `bool` | `std::vector<uint8_t> map_polylines_mask` | Amended from `float`; store 0/1 bool bytes for contiguous C++ buffer. |
| `map_polylines_center` | `[N, P, 3]` | `float32` | `std::vector<float> map_polylines_center` | No delta. |
| `pred_scores` | `[N, 6]` | `float32`, softmaxed mode scores | `std::vector<float> pred_scores`, `scores_shape` | No delta. |
| `pred_trajs` | `[N, 6, 80, 7]` before dataset export; dataset keeps x/y for Waymo eval output | `float32` | `std::vector<float> pred_trajs`, `trajs_shape` | No delta for raw model output. Output converter may consume only x/y plus optional velocity/GMM fields depending on export. |

Legend:

- `N`: number of center/target objects.
- `O`: number of valid context objects after filtering.
- `H`: past history steps. Official Waymo preprocessing uses `current_time_index + 1`, with `current_time_index = 10`.
- `P`: selected map polylines. Official config uses `NUM_OF_SRC_POLYLINES: 768`.

## Source Anchors

- `/private/tmp/mtr-task0/mtr/datasets/dataset.py`: collates `obj_trajs`, masks, map tensors, `track_index_to_predict`, and `center_objects_type` into `batch_dict['input_dict']`.
- `/private/tmp/mtr-task0/mtr/datasets/waymo/waymo_dataset.py`: creates `obj_trajs`, boolean masks, map polylines, object type tokens, and documents `pred_scores`/`pred_trajs`.
- `/private/tmp/mtr-task0/mtr/models/context_encoder/mtr_encoder.py`: asserts `obj_trajs_mask.dtype == torch.bool` and `map_polylines_mask.dtype == torch.bool`.
- `/private/tmp/mtr-task0/mtr/models/motion_decoder/mtr_decoder.py`: returns final `pred_scores` and `pred_trajs`, with `NUM_MOTION_MODES = 6` and `NUM_FUTURE_FRAMES = 80` from config.

## Amendments Applied

`src/world_modeling/prediction_ml/include/prediction_ml/mtr_types.hpp` was amended:

- `obj_trajs_mask`: `std::vector<float>` -> `std::vector<uint8_t>`
- `map_polylines_mask`: `std::vector<float>` -> `std::vector<uint8_t>`
- `center_objects_type`: `std::vector<int32_t>` -> `std::vector<std::string>`

## Remaining Gate Before Final Freeze

Run this in the real MTR environment with processed Waymo data and a checkpoint:

```python
batch = next(iter(dataloader))['input_dict']
for k in ['obj_trajs','obj_trajs_mask','obj_trajs_last_pos','track_index_to_predict',
          'center_objects_type','map_polylines','map_polylines_mask','map_polylines_center']:
    v = batch[k]
    print(k, tuple(v.shape) if hasattr(v, 'shape') else len(v), getattr(v, 'dtype', type(v)))

pred = model({'input_dict': batch, 'batch_size': 1, 'batch_sample_count': [len(batch['track_index_to_predict'])]})
for k in ['pred_scores','pred_trajs']:
    print(k, tuple(pred[k].shape), pred[k].dtype)
```

If TensorRT is the deploy backend, also dump the exported engine bindings and update `MtrModelContract` with binding names, dtypes, and dynamic dimensions.

## Sign-Off

- Person A (SceneBuilder): pending real forward-pass dump review
- Person B (Backend/TensorRT): pending engine export/binding review
- Person C (Runtime/Converter): pending output tensor semantic review
