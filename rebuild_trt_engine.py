#!/usr/bin/env python3
"""
Script to rebuild TensorRT engine for the current GPU architecture.
Searches for ONNX files and rebuilds the engine automatically.
"""

import sys
from pathlib import Path

# Try to find the trt_engine_builder module
try:
    from camera_object_detection.trt_engine_builder import build_trt_engine
except ImportError:
    try:
        import sys
        sys.path.insert(0, str(Path(__file__).parent / "src" / "perception" / "camera_object_detection"))
        from trt_engine_builder import build_trt_engine
    except ImportError:
        print("ERROR: Cannot import build_trt_engine. Make sure camera_object_detection is installed.")
        sys.exit(1)

def find_onnx_file():
    """Search for ONNX files in common locations."""
    search_dirs = [
        Path("/mnt/wato-drive/perception_models"),
        Path("/perception_models"),
        Path("/home/bolty/ament_ws/src/perception/camera_object_detection"),
    ]
    
    for search_dir in search_dirs:
        if not search_dir.exists():
            continue
        
        # Look for .onnx files
        onnx_files = list(search_dir.glob("*.onnx"))
        if onnx_files:
            # Prefer files with "tensorRT" or "tensor" in the name
            preferred = [f for f in onnx_files if "tensor" in f.name.lower()]
            if preferred:
                return preferred[0]
            return onnx_files[0]
    
    return None

def main():
    # Find ONNX file
    print("Searching for ONNX model file...")
    onnx_path = find_onnx_file()
    
    if onnx_path is None:
        print("ERROR: Could not find any .onnx files in:")
        for search_dir in [
            Path("/mnt/wato-drive/perception_models"),
            Path("/perception_models"),
        ]:
            print(f"  - {search_dir}")
        sys.exit(1)
    
    print(f"Found ONNX file: {onnx_path}")
    
    # Determine engine output path
    engine_path = onnx_path.parent / "tensorRT.engine"
    print(f"Output engine path: {engine_path}")
    
    # Rebuild engine
    print("\nRebuilding TensorRT engine for current GPU...")
    print("This may take a few minutes...")
    
    try:
        build_trt_engine(
            onnx_model_path=str(onnx_path),
            engine_output_path=str(engine_path),
            min_shape=[1, 3, 640, 640],
            opt_shape=[3, 3, 640, 640],
            max_shape=[6, 3, 640, 640],
            fp16=True,
            int8=False,
        )
        print(f"\n✓ Successfully rebuilt TensorRT engine: {engine_path}")
        
        # Also try to create a symlink or copy to /perception_models if it doesn't exist
        target_engine = Path("/perception_models/tensorRT.engine")
        if not target_engine.exists() and target_engine.parent.exists():
            try:
                import shutil
                shutil.copy2(engine_path, target_engine)
                print(f"✓ Copied engine to: {target_engine}")
            except Exception as e:
                print(f"Note: Could not copy to {target_engine}: {e}")
        
    except Exception as e:
        print(f"\n✗ ERROR: Failed to rebuild engine: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()

