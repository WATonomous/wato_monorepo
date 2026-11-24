#!/usr/bin/env python3
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
"""Check deployed model signatures: check_onnx_io.py DIR [--ignore-dtype].

Use --ref PATH to select a reference, or --write-ref to generate it from DIR.
"""

import argparse
import json
from pathlib import Path

import onnx

FILES = (
    "camera.backbone.onnx",
    "camera.vtransform.onnx",
    "fuser.onnx",
    "head.bbox.onnx",
    "lidar.backbone.xyz.onnx",
)


def signature(path):
    """Read graph I/O without loading external tensor data."""
    graph = onnx.load(str(path), load_external_data=False).graph
    result = {}
    for direction, values in (("inputs", graph.input), ("outputs", graph.output)):
        result[direction] = []
        for value in values:
            tensor = value.type.tensor_type
            dims = [
                dim.dim_value if dim.HasField("dim_value") else dim.dim_param
                for dim in tensor.shape.dim
            ]
            result[direction].append(
                [value.name, dims, onnx.TensorProto.DataType.Name(tensor.elem_type)]
            )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument(
        "--ref", type=Path, default=Path(__file__).with_name("onnx_io_reference.json")
    )
    parser.add_argument("--ignore-dtype", action="store_true")
    parser.add_argument("--write-ref", action="store_true")
    args = parser.parse_args()
    actual = {}
    mismatches = []
    for filename in FILES:
        path = args.directory / filename
        if not path.is_file():
            mismatches.append("{}: missing file".format(filename))
        else:
            actual[filename] = signature(path)
    if args.write_ref:
        if not mismatches:
            args.ref.write_text(
                json.dumps(actual, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
            return 0
    else:
        reference = json.loads(args.ref.read_text(encoding="utf-8"))
        for filename, observed in actual.items():
            for direction in ("inputs", "outputs"):
                expected = reference[filename][direction]
                values = observed[direction]
                if len(values) != len(expected):
                    mismatches.append(
                        "{}: {} count {} != {}".format(
                            filename, direction, len(values), len(expected)
                        )
                    )
                for value, target in zip(values, expected):
                    label = "{}: {} {}".format(filename, direction[:-1], value[0])
                    for index, field in enumerate(("name", "shape", "dtype")):
                        if field == "dtype" and args.ignore_dtype:
                            continue
                        if value[index] != target[index]:
                            mismatches.append(
                                "{} {} {} != {}".format(
                                    label, field, value[index], target[index]
                                )
                            )
    for mismatch in mismatches:
        print(mismatch)
    if mismatches:
        return 1
    print("OK: {} files match".format(len(FILES)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
