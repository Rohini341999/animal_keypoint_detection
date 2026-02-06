import os
import shutil
import random
import yaml
import torch
from ultralytics import YOLO


def create_subset_dataset(source_dir, subset_dir, count=100):
    """
    Creates a subset of the dataset for testing purposes or rapid iteration.
    Structure created:
        subset_dir/
            dataset_subset.yaml
            images/
                train/
            labels/
                train/
    """
    # 1. Setup Directories
    # YOLO typically looks for images relative to the path defined in yaml (or in 'train' subdir)
    images_train_dir = os.path.join(subset_dir, "images", "train")
    labels_train_dir = os.path.join(subset_dir, "labels", "train")

    # Clear previous subset if exists to ensure fresh sample (optional, but good for "create script" flow)
    if os.path.exists(subset_dir):
        shutil.rmtree(subset_dir)

    os.makedirs(images_train_dir, exist_ok=True)
    os.makedirs(labels_train_dir, exist_ok=True)

    # 2. Get Source Files
    # Assuming the script is run from the root where 'images' and 'labels' folders exist
    source_images_dir = os.path.join(source_dir, "images")
    source_labels_dir = os.path.join(source_dir, "labels")

    if not os.path.exists(source_images_dir):
        raise FileNotFoundError(
            f"Source images directory not found at {source_images_dir}"
        )

    all_images = [
        f
        for f in os.listdir(source_images_dir)
        if f.lower().endswith((".jpg", ".jpeg", ".png"))
    ]
    all_images.sort()  # Ensure deterministic order before random sampling if we seeded, but here random is fine

    if not all_images:
        raise FileNotFoundError(f"No images found in {source_images_dir}")

    # 3. Select Random Subset
    count = min(count, len(all_images))
    selected_images = random.sample(all_images, count)
    print(f"Selecting {len(selected_images)} images for the subset dataset...")

    # 4. Copy Files
    for img_name in selected_images:
        # Copy Image
        src_img_path = os.path.join(source_images_dir, img_name)
        dst_img_path = os.path.join(images_train_dir, img_name)
        shutil.copy2(src_img_path, dst_img_path)

        # Copy Label
        # Assuming label has same basename but .txt extension
        lbl_name = os.path.splitext(img_name)[0] + ".txt"
        src_lbl_path = os.path.join(source_labels_dir, lbl_name)
        dst_lbl_path = os.path.join(labels_train_dir, lbl_name)

        if os.path.exists(src_lbl_path):
            shutil.copy2(src_lbl_path, dst_lbl_path)
        else:
            print(f"Warning: Label not found for {img_name}")

    # 5. Create dataset.yaml
    # Note: 'path' should be an absolute path to the dataset root
    dataset_yaml = {
        "path": os.path.abspath(subset_dir),
        "train": "images/train",
        "val": "images/train",  # Using same set for validation in this test
        "names": {0: "horse"},
        "kpt_shape": [26, 3],  # 26 keypoints, 3 coords (x, y, visibility)
    }

    yaml_path = os.path.join(subset_dir, "dataset_subset.yaml")
    with open(yaml_path, "w") as f:
        yaml.dump(dataset_yaml, f, sort_keys=False)

    print(f"Subset dataset prepared at: {subset_dir}")
    print(f"YAML config created at: {yaml_path}")
    return yaml_path


def main():
    # Configuration
    CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
    SUBSET_DIR = os.path.join(CURRENT_DIR, "subset_100_dataset")

    # Create the subset data
    yaml_path = create_subset_dataset(CURRENT_DIR, SUBSET_DIR, count=100)

    # Initialize YOLO Model
    print("Loading YOLOv8n-pose model...")
    # This will download the model weights to current dir if not present
    model = YOLO("yolov8n-pose.pt")

    # Train
    # Using a small number of epochs to verify the pipeline works
    print("Starting training...")
    try:
        model.train(
            data=yaml_path,
            epochs=50,
            imgsz=640,
            batch=16,
            project="runs/pose",
            name="train_subset_100",
            exist_ok=True,  # Overwrite existing run directory if it exists
            plots=True,  # Generate plots (confusion matrix, etc.)
        )
        print("Training finished successfully.")
    except Exception as e:
        print(f"An error occurred during training: {e}")


if __name__ == "__main__":
    # Uncomment the line below to run the main function
    main()
