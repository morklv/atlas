"""Local-only aerial image segmentation for the field workspace."""

from __future__ import annotations

from io import BytesIO
from pathlib import Path

from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.responses import FileResponse
from PIL import Image, UnidentifiedImageError

ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / ".models" / "openearth-mask2former"
FIELD_HTML = ROOT / "output" / "swarm" / "field.html"
MODEL_SOURCE = "mfaytin/mask2former-satellite"
app = FastAPI(title="ATLAS local field workspace")
_model = None
_processor = None


def _load_model():
    global _model, _processor
    if _model is None:
        if not (MODEL_DIR / "model.safetensors").is_file():
            raise RuntimeError("Local aerial model is missing. Run the model setup before starting the field server.")
        import torch
        from transformers import Mask2FormerForUniversalSegmentation, Mask2FormerImageProcessor

        _processor = Mask2FormerImageProcessor.from_pretrained(MODEL_DIR, local_files_only=True, use_fast=False)
        _model = Mask2FormerForUniversalSegmentation.from_pretrained(
            MODEL_DIR, local_files_only=True, use_safetensors=True
        ).eval()
        # CPU inference is predictable on the supported Mac and needs no GPU.
        _model.to("cpu")
    return _model, _processor


def segment_image(image: Image.Image) -> dict:
    import torch
    model, processor = _load_model()
    image = image.convert("RGB")
    inputs = processor(images=image, return_tensors="pt")
    with torch.inference_mode():
        outputs = model(**inputs)
        # 128 cells preserve narrow rural tracks much better than the earlier
        # 64-cell display grid while remaining responsive in Safari.
        classes = processor.post_process_semantic_segmentation(outputs, target_sizes=[(128, 128)])[0].numpy()
    labels = []
    counts = {"road": 0, "building": 0, "open": 0, "low_vegetation": 0, "forest": 0, "water": 0}
    # Browser grid is Cartesian with y north; image rows begin at the top.
    for y in range(127, -1, -1):
        for x in range(128):
            category = int(classes[y, x])
            # This checkpoint's eight logits follow OpenEarthMap's classes:
            # bareland, rangeland, developed space, road, tree, water,
            # agriculture, building.  Browser labels retain the existing road
            # and building ids and add terrain classes for visualisation and
            # ground-robot costs.
            label = {
                2: 1,  # developed space / likely paved or compacted track
                3: 1,  # road
                7: 2,  # building
                4: 3,  # tree / dense forest
                1: 4,  # rangeland
                6: 4,  # agriculture / low vegetation
                5: 5,  # water
            }.get(category, 0)  # bare ground or developed open space
            labels.append(label)
            name = {0: "open", 1: "road", 2: "building", 3: "forest", 4: "low_vegetation", 5: "water"}[label]
            counts[name] += 1
    return {"schema": "sentinel-image-semantics-v1", "n": 128, "labels": labels,
            "counts": counts, "model": MODEL_SOURCE, "image_type": "aerial imagery",
            "note": "Predicted 2D classes; no measured height or route clearance."}


@app.get("/")
def field_page():
    if not FIELD_HTML.is_file():
        raise HTTPException(404, "Generate the field app with: bash run.sh swarm")
    return FileResponse(FIELD_HTML, media_type="text/html")


@app.post("/api/segment")
async def segment(file: UploadFile = File(...)):
    if file.content_type not in {"image/png", "image/jpeg", "image/webp"}:
        raise HTTPException(415, "Use a PNG, JPEG or WebP aerial image.")
    data = await file.read(10 * 1024 * 1024 + 1)
    if len(data) > 10 * 1024 * 1024:
        raise HTTPException(413, "Image must be under 10 MB.")
    try:
        image = Image.open(BytesIO(data))
        image.load()
    except (UnidentifiedImageError, OSError):
        raise HTTPException(400, "The image could not be decoded.") from None
    try:
        return segment_image(image)
    except RuntimeError as exc:
        raise HTTPException(503, str(exc)) from exc
