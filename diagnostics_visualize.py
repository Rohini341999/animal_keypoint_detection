# diagnostic_visualize.py
"""
Visualize keypoints with indices ONLY (no skeleton)
to understand the actual keypoint layout in Animal-3D
"""

import cv2
import matplotlib.pyplot as plt
from pathlib import Path
import random

DATASET_ROOT = Path(".")


def visualize_keypoints_only(img_path, label_path):
    print("visualize_keypoints_only called")
    print(img_path)
    img = cv2.imread(str(img_path))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    h, w = img.shape[:2]

    with open(label_path) as f:
        parts = list(map(float, f.readline().strip().split()))

    kp_data = parts[5:]
    print(kp_data)

    # Color gradient from head (red) to tail (blue)
    for i in range(0, len(kp_data), 3):
        idx = i // 3
        x = int(kp_data[i] * w)
        y = int(kp_data[i + 1] * h)
        v = int(kp_data[i + 2])

        if v == 0:
            continue

        # Color based on index (helps identify anatomical grouping)
        color = plt.cm.rainbow(idx / 26)[:3]
        color = tuple(int(c * 255) for c in color)

        cv2.circle(img, (x, y), 6, color, -1)
        cv2.circle(img, (x, y), 6, (0, 0, 0), 1)  # black border
        cv2.putText(
            img,
            str(idx),
            (x + 8, y + 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            img, str(idx), (x + 8, y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1
        )

    plt.figure(figsize=(12, 8))
    plt.imshow(img)
    plt.title(f"Keypoint indices: {img_path.name}")
    plt.axis("off")
    plt.tight_layout()
    print(DATASET_ROOT / f"diagnostic_{img_path.stem}.png")
    plt.savefig(DATASET_ROOT / f"diagnostic_{img_path.stem}.png", dpi=150)
    plt.close()
    print(f"Saved: diagnostic_{img_path.stem}.png")


# Run on a few samples
img_dir = DATASET_ROOT / "images" / "train"
lbl_dir = DATASET_ROOT / "labels"  # / "train"

samples = [
    "/home/roha/Documents/horse_26kp/images/n02389026_000000027646_horse.jpg",
    "/home/roha/Documents/horse_26kp/images/n02389026_000000032607_horse.jpg",
    "/home/roha/Documents/horse_26kp/images/n02389026_000000038828_horse.jpg",
    "/home/roha/Documents/horse_26kp/images/n02389026_000000081995_horse.jpg",
    "/home/roha/Documents/horse_26kp/images/n02389026_000000150098_horse.jpg",
    "/home/roha/Documents/horse_26kp/images/n02389026_000000206247_horse.jpg",
    "/home/roha/Documents/horse_26kp/images/n02389026_000000329262_horse.jpg",
    "/home/roha/Documents/horse_26kp/images/n02389026_000000356968_horse.jpg",
    "/home/roha/Documents/horse_26kp/images/n02389026_000000364586_horse.jpg",
    "/home/roha/Documents/horse_26kp/images/n02389026_000000374266_horse.jpg",
    "/home/roha/Documents/horse_26kp/images/n02389026_000000401720_horse.jpg",
    "/home/roha/Documents/horse_26kp/images/n02389026_000000464633_horse.jpg",
    "/home/roha/Documents/horse_26kp/images/n02389026_000000489266_horse.jpg",
]
# list(img_dir.glob("*.jpg"))[:10]
for img_path in samples:
    img_path = Path(img_path)
    print(img_path)
    lbl_path = lbl_dir / f"{img_path.stem}.txt"
    print(lbl_path)
    if lbl_path.exists():
        print("lbl_path exists")
        visualize_keypoints_only(img_path, lbl_path)
