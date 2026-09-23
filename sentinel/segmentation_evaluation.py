"""Reproducible metrics for ATLAS aerial semantic segmentation.

Ground-truth masks use the browser's six class identifiers: 0 open ground,
1 road, 2 building, 3 forest, 4 low vegetation, and 5 water.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from PIL import Image


CLASS_NAMES = ("open", "road", "building", "forest", "low_vegetation", "water")


def confusion_matrix(predicted: np.ndarray, truth: np.ndarray, classes: int = len(CLASS_NAMES)) -> np.ndarray:
    """Return rows=ground truth and columns=predictions after strict validation."""
    predicted = np.asarray(predicted, dtype=int)
    truth = np.asarray(truth, dtype=int)
    if predicted.shape != truth.shape or predicted.ndim != 2:
        raise ValueError("predicted and truth must be equally sized two-dimensional arrays")
    if predicted.size == 0 or predicted.min() < 0 or truth.min() < 0 or predicted.max() >= classes or truth.max() >= classes:
        raise ValueError("labels must be in the configured class range")
    return np.bincount(truth.ravel() * classes + predicted.ravel(), minlength=classes * classes).reshape(classes, classes)


def metrics(confusion: np.ndarray) -> dict:
    """Calculate class-level and macro metrics without hiding absent classes."""
    confusion = np.asarray(confusion, dtype=float)
    if confusion.shape != (len(CLASS_NAMES), len(CLASS_NAMES)):
        raise ValueError("confusion has an unexpected shape")
    entries = {}
    ious = []
    for index, name in enumerate(CLASS_NAMES):
        true_positive = confusion[index, index]
        false_positive = confusion[:, index].sum() - true_positive
        false_negative = confusion[index, :].sum() - true_positive
        support = confusion[index, :].sum()
        predicted_count = confusion[:, index].sum()
        precision = true_positive / predicted_count if predicted_count else None
        recall = true_positive / support if support else None
        union = true_positive + false_positive + false_negative
        iou = true_positive / union if union else None
        if iou is not None:
            ious.append(iou)
        entries[name] = {
            "support_pixels": int(support),
            "predicted_pixels": int(predicted_count),
            "precision": precision,
            "recall": recall,
            "iou": iou,
        }
    accuracy = float(np.trace(confusion) / confusion.sum()) if confusion.sum() else None
    return {"pixel_accuracy": accuracy, "mean_iou": float(np.mean(ious)) if ious else None, "classes": entries}


def _mask(path: Path) -> np.ndarray:
    mask = np.asarray(Image.open(path).convert("L"), dtype=np.uint8)
    if mask.size == 0 or mask.max() >= len(CLASS_NAMES):
        raise ValueError(f"{path} must be a grayscale mask using values 0 through 5")
    return mask


def evaluate_manifest(manifest_path: str | Path, output_dir: str | Path) -> dict:
    """Run ATLAS local inference against a JSON list of image/mask pairs.

    Example manifest: `[{'image': 'farm.png', 'mask': 'farm_mask.png'}]`.
    Paths are relative to the manifest file.  The model remains local.
    """
    manifest_path = Path(manifest_path)
    items = json.loads(manifest_path.read_text())
    if not isinstance(items, list) or not items:
        raise ValueError("manifest must be a non-empty JSON list")
    from .field_server import segment_image

    aggregate = np.zeros((len(CLASS_NAMES), len(CLASS_NAMES)), dtype=np.int64)
    samples = []
    for item in items:
        if not isinstance(item, dict) or set(item) != {"image", "mask"}:
            raise ValueError("every manifest entry must contain exactly image and mask")
        image_path = manifest_path.parent / item["image"]
        mask_path = manifest_path.parent / item["mask"]
        truth = _mask(mask_path)
        image = Image.open(image_path).convert("RGB")
        prediction = segment_image(image)
        labels = np.asarray(prediction["labels"], dtype=np.uint8).reshape(prediction["n"], prediction["n"])
        resized_truth = np.asarray(Image.fromarray(truth).resize((prediction["n"], prediction["n"]), Image.Resampling.NEAREST))
        sample_confusion = confusion_matrix(labels, resized_truth)
        aggregate += sample_confusion
        samples.append({"image": item["image"], "mask": item["mask"], "metrics": metrics(sample_confusion)})
    result = {
        "schema": "atlas-segmentation-evaluation-v1",
        "class_names": list(CLASS_NAMES),
        "samples": samples,
        "aggregate_confusion_matrix": aggregate.tolist(),
        "aggregate_metrics": metrics(aggregate),
        "note": "Metrics describe only the labeled images listed in this manifest.",
    }
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "segmentation_metrics.json").write_text(json.dumps(result, indent=2) + "\n")
    return result
