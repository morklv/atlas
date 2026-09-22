"""Capture one Gazebo Harmonic float32 optical-depth image to NPZ.

Requires gz.transport13 and gz.msgs10 Python bindings. The output is a sensor
probe, NOT a mapped observation: synchronized camera extrinsics are still needed.
"""
import argparse
import json
import threading
from pathlib import Path
import numpy as np


def decode_float_depth(message):
    field = message.DESCRIPTOR.fields_by_name["pixel_format_type"]
    fmt = field.enum_type.values_by_number[message.pixel_format_type].name
    if fmt != "R_FLOAT32":
        raise ValueError(f"Expected R_FLOAT32 depth, received {fmt}")
    stride = message.step or message.width * 4
    if stride < message.width * 4 or len(message.data) < stride * message.height:
        raise ValueError("Depth image buffer is truncated or has an invalid row stride")
    return np.ndarray((message.height, message.width), dtype="<f4",
                      buffer=message.data, strides=(stride, 4)).copy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", required=True, help="Exact depth topic from gz topic -l")
    parser.add_argument("--out", type=Path, default=Path("output/gazebo-depth.npz"))
    parser.add_argument("--timeout", type=float, default=20.)
    args = parser.parse_args()
    try:
        from gz.transport13 import Node
        from gz.msgs10.image_pb2 import Image
    except ImportError:
        parser.exit(1, "Gazebo Harmonic Python bindings are unavailable. See docs/GAZEBO.md.\n")
    completed = threading.Event()
    result = {}
    def receive(message):
        if completed.is_set():
            return
        try:
            result["depth"] = decode_float_depth(message)
        except Exception as exc:
            result["error"] = str(exc)
        completed.set()
    node = Node()
    if not node.subscribe(Image, args.topic, receive):
        parser.exit(1, "Could not subscribe to the specified topic.\n")
    if not completed.wait(args.timeout):
        parser.exit(1, "No depth image received before timeout.\n")
    if "error" in result:
        parser.exit(1, result["error"] + "\n")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.out, optical_depth_m=result["depth"])
    print(json.dumps({"shape": list(result["depth"].shape), "saved": str(args.out),
                      "depth_convention": "optical-axis depth; convert before range mapping"}, indent=2))


if __name__ == "__main__":
    main()
