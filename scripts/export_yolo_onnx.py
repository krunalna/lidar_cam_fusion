"""
Export YOLOv8 checkpoint to ONNX for C++ inference.

Usage:
  pixi run export-onnx                          # exports yolov8n.pt → models/yolov8n.onnx
  pixi run export-onnx -- --model yolov8s.pt    # different model variant
  pixi run export-onnx -- --output /custom/path/model.onnx
"""

import argparse
import shutil
import sys
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description="Export YOLOv8 model to ONNX")
    parser.add_argument("--model",  default="yolov8n.pt",
                        help="YOLOv8 checkpoint name or path (default: yolov8n.pt)")
    parser.add_argument("--output", default="models/yolov8n.onnx",
                        help="Destination .onnx path (default: models/yolov8n.onnx)")
    parser.add_argument("--imgsz",  type=int, default=640,
                        help="Input image size (default: 640)")
    parser.add_argument("--opset",  type=int, default=12,
                        help="ONNX opset version (default: 12)")
    args = parser.parse_args()

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    try:
        from ultralytics import YOLO
    except ImportError:
        print("ERROR: ultralytics not found. Run: pixi install", file=sys.stderr)
        sys.exit(1)

    print(f"Loading {args.model} ...")
    model = YOLO(args.model)

    print(f"Exporting to ONNX  (imgsz={args.imgsz}, opset={args.opset}) ...")
    exported_path = model.export(
        format="onnx",
        imgsz=args.imgsz,
        opset=args.opset,
        simplify=True,
        dynamic=False,
    )

    # ultralytics writes the file next to the .pt; move it to the requested location
    exported = Path(exported_path)
    if exported.resolve() != output.resolve():
        shutil.move(str(exported), str(output))

    print(f"\nSaved: {output.resolve()}")
    print("\nOutput tensor shape: (1, 84, 8400)")
    print("  rows 0-3  : cx, cy, w, h  (letterboxed 640-space)")
    print("  rows 4-83 : 80 COCO class scores")
    print("\nNext steps:")
    print(f"  KITTI_SEQ=<path> YOLO_ONNX=$(pwd)/{output} pixi run launch-cpp")


if __name__ == "__main__":
    main()
