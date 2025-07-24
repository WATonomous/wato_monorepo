# yolo_utils.py
# Helper module to handle YOLOv8 TensorRT inference in a reusable class

import os
import cv2
import numpy as np
import tensorrt as trt
import torch
from cuda import cudart
from dataclasses import dataclass
from typing import Tuple, List, Dict

@dataclass
class Tensor:
    name: str
    dtype: np.dtype
    shape: Tuple[int, ...]
    cpu: np.ndarray
    gpu: int

class YOLOTensorRT:
    def __init__(
        self,
        engine_path: str,
        batch_size: int = 3,
        input_size: Tuple[int, int] = (640, 640),
        conf_thres: float = 0.5,
        iou_thres: float = 0.45,
    ):
        """
        Initialize the TensorRT engine and CUDA buffers.
        :param engine_path: Path to the serialized .engine file
        :param batch_size: Number of images per inference batch
        :param input_size: (height, width) of model input
        :param conf_thres: Confidence threshold for NMS
        :param iou_thres: IoU threshold for NMS
        """
        self.batch_size = batch_size
        self.input_h, self.input_w = input_size
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres

        # TensorRT runtime and engine
        self.logger = trt.Logger(trt.Logger.WARNING)
        trt.init_libnvinfer_plugins(self.logger, namespace="")
        with trt.Runtime(self.logger) as runtime:
            engine_bytes = Path(engine_path).read_bytes()
            self.engine = runtime.deserialize_cuda_engine(engine_bytes)
        self.context = self.engine.create_execution_context()
        if not self.context:
            raise RuntimeError("Failed to create TensorRT execution context")

        # Allocate I/O bindings and CUDA buffers
        self._collect_io_names()
        self._alloc_cuda_buffers()

    def _collect_io_names(self):
        """Collect TensorRT engine I/O tensor names and modes."""
        all_names = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)]
        modes = [self.engine.get_tensor_mode(n) for n in all_names]
        self.input_names = [n for n, m in zip(all_names, modes) if m == trt.TensorIOMode.INPUT]
        self.output_names = [n for n, m in zip(all_names, modes) if m == trt.TensorIOMode.OUTPUT]

    def _alloc_cuda_buffers(self):
        """Allocate host/device buffers for all inputs and outputs."""
        status, self.stream = cudart.cudaStreamCreate()
        assert status.value == 0, "Failed to create CUDA stream"

        # Inputs
        self.input_tensors: List[Tensor] = []
        for name in self.input_names:
            shape = tuple(self.context.get_tensor_shape(name))
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            host_mem = np.empty(shape, dtype=dtype)
            status, dev_mem = cudart.cudaMallocAsync(host_mem.nbytes, self.stream)
            assert status.value == 0, f"cudaMallocAsync failed for input {name}"
            self.input_tensors.append(Tensor(name, dtype, shape, host_mem, dev_mem))

        # Outputs
        self.output_tensors: List[Tensor] = []
        for name in self.output_names:
            shape = tuple(self.context.get_tensor_shape(name))
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            host_mem = np.empty(shape, dtype=dtype)
            status, dev_mem = cudart.cudaMallocAsync(host_mem.nbytes, self.stream)
            assert status.value == 0, f"cudaMallocAsync failed for output {name}"
            self.output_tensors.append(Tensor(name, dtype, shape, host_mem, dev_mem))

    def preprocess(self, img: np.ndarray) -> np.ndarray:
        """
        Resize, normalize, and transpose an image for model input.
        :param img: BGR image as NumPy array
        :return: Array of shape (3, H, W) with values in [0,1]
        """
        img_resized = cv2.resize(img, (self.input_w, self.input_h), interpolation=cv2.INTER_LINEAR)
        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
        img_norm = img_rgb.astype(np.float32) / 255.0
        # HWC to CHW
        return np.transpose(img_norm, (2, 0, 1))

    def infer(self, batch_imgs: List[np.ndarray]) -> List[List[Dict]]:
        """
        Run inference on a batch of images and parse detections.
        :param batch_imgs: List of BGR images
        :return: A list (per image) of detections dicts {bbox, confidence, class}
        """
        # Prepare batch tensor
        batch_array = np.stack([self.preprocess(img) for img in batch_imgs], axis=0)
        inp = self.input_tensors[0]
        assert batch_array.nbytes == inp.cpu.nbytes, "Batch size mismatch"
        np.copyto(inp.cpu, batch_array)

        # Copy to GPU
        cudart.cudaMemcpyAsync(
            inp.gpu, inp.cpu.ctypes.data, inp.cpu.nbytes,
            cudart.cudaMemcpyKind.cudaMemcpyHostToDevice, self.stream
        )

        # Bind and run
        self.context.set_tensor_address(inp.name, inp.gpu)
        for out in self.output_tensors:
            self.context.set_tensor_address(out.name, out.gpu)
        assert self.context.execute_async_v3(self.stream), "Inference execution failed"

        # Copy outputs back
        for out in self.output_tensors:
            cudart.cudaMemcpyAsync(
                out.cpu.ctypes.data, out.gpu, out.cpu.nbytes,
                cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost, self.stream
            )
        cudart.cudaStreamSynchronize(self.stream)

        # Parse
        outputs = tuple(t.cpu for t in self.output_tensors)
        return self.parse_detections(outputs)

    def parse_detections(self, outputs: Tuple[np.ndarray, ...]) -> List[List[Dict]]:
        """
        Convert raw model outputs to human-readable detections with NMS.
        """
        # Assuming outputs[0] is [batch, features, anchors]
        tensor = torch.tensor(outputs[0], dtype=torch.float32)
        batch, _, anchors = tensor.shape
        dets = tensor.permute(0, 2, 1)
        bboxes = dets[..., :4]
        probs = dets[..., 4:]
        conf, cls = probs.max(dim=2)
        x, y, w, h = bboxes.split(1, dim=2)
        boxes = torch.cat((x - w/2, y - h/2, x + w/2, y + h/2), dim=2)
        valid = torch.cat((boxes, conf.unsqueeze(2), cls.unsqueeze(2)), dim=2)
        nms = non_max_suppression(valid, conf_thres=self.conf_thres, iou_thres=self.iou_thres)

        results = []
        for img_dets in nms:
            det_list = []
            for det in img_dets:
                det_list.append({
                    'bbox': det[:4].tolist(),
                    'confidence': det[4].item(),
                    'class': int(det[5].item())
                })
            results.append(det_list)
        return results

    def destroy(self):
        """Free CUDA resources."""
        for t in self.input_tensors + self.output_tensors:
            cudart.cudaFreeAsync(t.gpu, self.stream)
        cudart.cudaStreamDestroy(self.stream)
