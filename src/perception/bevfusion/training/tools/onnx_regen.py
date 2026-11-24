# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# From the team Drive "Bevfusion Models" folder. TensorRT 11 is strongly typed, so the node's FP16 buffers need FP16 ONNX I/O.

import os
import sys

import onnx
from onnx import TensorProto, helper


def patch_onnx_to_pure_fp16(input_onnx, output_onnx):
    if not os.path.exists(input_onnx):
        print(f"Skipping: {input_onnx} does not exist.")
        return

    model = onnx.load(input_onnx)
    graph = model.graph

    fp32_inputs = [
        inp.name
        for inp in graph.input
        if inp.type.tensor_type.elem_type == TensorProto.FLOAT
    ]
    fp32_outputs = [
        out.name
        for out in graph.output
        if out.type.tensor_type.elem_type == TensorProto.FLOAT
    ]

    if not fp32_inputs and not fp32_outputs:
        print(f"{input_onnx} is already fully patched to FP16 IO ports.")
        if input_onnx != output_onnx:
            onnx.save(model, output_onnx)
        return

    # Redirect consumers of inputs and producers of outputs to internal FP32 streams
    for node in graph.node:
        for i, node_input in enumerate(node.input):
            if node_input in fp32_inputs:
                node.input[i] = node_input + "_fp32_internal"
            if node_input in fp32_outputs:
                node.input[i] = node_input + "_fp32_internal"
        for i, node_output in enumerate(node.output):
            if node_output in fp32_outputs:
                node.output[i] = node_output + "_fp32_internal"

    # Create input cast nodes (External FP16 -> Internal FP32)
    input_cast_nodes = []
    for inp_name in fp32_inputs:
        cast_node = helper.make_node(
            "Cast",
            inputs=[inp_name],
            outputs=[inp_name + "_fp32_internal"],
            to=TensorProto.FLOAT,
            name=f"cast_in_{inp_name}",
        )
        input_cast_nodes.append(cast_node)

    # Create output cast nodes (Internal FP32 -> External FP16)
    output_cast_nodes = []
    for out_name in fp32_outputs:
        cast_node = helper.make_node(
            "Cast",
            inputs=[out_name + "_fp32_internal"],
            outputs=[out_name],
            to=TensorProto.FLOAT16,
            name=f"cast_out_{out_name}",
        )
        output_cast_nodes.append(cast_node)

    # Update external graph boundary metadata definitions to FLOAT16
    for inp in graph.input:
        if inp.name in fp32_inputs:
            inp.type.tensor_type.elem_type = TensorProto.FLOAT16
    for out in graph.output:
        if out.name in fp32_outputs:
            out.type.tensor_type.elem_type = TensorProto.FLOAT16

    # Reconstruct graph node execution order sequence safely
    original_nodes = list(graph.node)
    graph.ClearField("node")
    graph.node.extend(input_cast_nodes + original_nodes + output_cast_nodes)

    onnx.save(model, output_onnx)
    print(
        f"Successfully patched {input_onnx} -> {output_onnx} "
        "(Inputs & Outputs unified to FP16)"
    )


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: onnx_regen.py <onnx_dir>")
        sys.exit(1)

    target_dir = sys.argv[1]
    models = [
        "camera.backbone.onnx",
        "fuser.onnx",
        "camera.vtransform.onnx",
        "head.bbox.onnx",
    ]

    for m in models:
        full_path = os.path.join(target_dir, m)
        backup_path = full_path + ".bak"

        # Always read from the clean original backup file if it exists
        if os.path.exists(backup_path):
            patch_onnx_to_pure_fp16(backup_path, full_path)
        else:
            if os.path.exists(full_path):
                os.rename(full_path, backup_path)
                patch_onnx_to_pure_fp16(backup_path, full_path)
