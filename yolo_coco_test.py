import os
import requests
from pathlib import Path
from PIL import Image
from ultralytics import YOLO


def download_image(url, save_path):
    """Download image from GitHub raw URL safely."""
    try:
        response = requests.get(url, timeout=10)
        response.raise_for_status()
        save_path.write_bytes(response.content)

        # validate
        Image.open(save_path).verify()
        print(f"Downloaded: {save_path.name}")
        return True
    except Exception as e:
        print(f"❌ Failed: {save_path.name} ({e})")
        return False


def test_model():

    # Paths
    current_dir = Path.cwd()
    best_model_path = (
        current_dir / "runs" / "pose" / "yolo11n_tuned" / "weights" / "best.pt"
    )
    output_dir = current_dir / "visualizations" / "coco_val_test"
    os.makedirs(output_dir, exist_ok=True)

    print(f"Loading model: {best_model_path}")
    model = YOLO(str(best_model_path))

    # GitHub-hosted COCO sample images (always available)
    coco_images = {
        "coco_person.jpg": "https://raw.githubusercontent.com/ultralytics/yolov5/master/data/images/zidane.jpg",
        "coco_bus.jpg": "https://raw.githubusercontent.com/ultralytics/yolov5/master/data/images/bus.jpg",
        "coco_zebra.jpg": "https://raw.githubusercontent.com/ultralytics/yolov5/master/data/images/zebra.jpg",
    }

    coco_dir = current_dir / "sample_coco"
    coco_dir.mkdir(exist_ok=True)

    downloaded = []
    for name, url in coco_images.items():
        save_path = coco_dir / name
        if download_image(url, save_path):
            downloaded.append(save_path)

    if not downloaded:
        print("❌ No images downloaded.")
        return

    print("\nRunning inference...\n")

    for img_path in downloaded:
        print(f"→ Predicting on: {img_path.name}")

        model.predict(
            source=str(img_path),
            save=True,
            project=str(output_dir),
            name="results",
            exist_ok=True,
        )

    print("\n✅ Inference complete!")
    print(f"Visualizations saved in: {output_dir}")


if __name__ == "__main__":
    test_model()
