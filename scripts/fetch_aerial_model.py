"""Download the optional public aerial-segmentation checkpoint for ATLAS."""

from pathlib import Path

from huggingface_hub import snapshot_download


REPOSITORY = "mfaytin/mask2former-satellite"
DESTINATION = Path(__file__).resolve().parents[1] / ".models" / "openearth-mask2former"


def main() -> None:
    path = snapshot_download(
        repo_id=REPOSITORY,
        local_dir=DESTINATION,
        local_dir_use_symlinks=False,
    )
    print(f"Downloaded {REPOSITORY} to {path}")


if __name__ == "__main__":
    main()
