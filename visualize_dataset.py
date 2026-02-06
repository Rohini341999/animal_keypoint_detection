import cv2
import os
import glob

# FIXED SKELETON PAIRS
SKELETON = [
    # Head
    (0, 1),
    (0, 20),
    (1, 21),
    (0, 22),
    (1, 23),
    (22, 24),
    (23, 24),
    (24, 2),
    # Body/Topline
    (18, 13),
    (13, 7),
    (7, 19),
    (13, 12),
    (7, 25),
    # Front Legs
    (12, 8),
    (12, 9),
    (8, 14),
    (9, 15),
    (14, 3),
    (15, 4),
    # Rear Legs
    (25, 10),
    (25, 11),
    (10, 16),
    (11, 17),
    (16, 5),
    (17, 6),
]


def visualize_horse_skeleton(img_folder, label_folder, output_folder):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    for img_path in glob.glob(os.path.join(img_folder, "*.jpg")):
        base_name = os.path.basename(img_path)
        label_path = os.path.join(label_folder, base_name.replace(".jpg", ".txt"))

        if not os.path.exists(label_path):
            continue

        img = cv2.imread(img_path)
        h, w, _ = img.shape
        # Load all lines from label file to handle multiple objects
        with open(label_path, "r") as f:
            lines = f.readlines()

        for line in lines:
            parts = line.strip().split()
            # Basic validation for YOLO format (class + bbox + at least 1 kp)
            if len(parts) < 6:
                continue

            # Keypoints start after class, x, y, w, h
            keypoint_data = parts[5:]

            kps = {}
            for i in range(0, len(keypoint_data), 3):
                if i + 2 >= len(keypoint_data):
                    break

                kp_idx = i // 3
                try:
                    kx_norm = float(keypoint_data[i])
                    ky_norm = float(keypoint_data[i + 1])
                    v = float(keypoint_data[i + 2])
                except ValueError:
                    continue

                # Filter by visibility or coordinates
                # Assuming v=0 is not visible/not labeled
                if v > 0:
                    kps[kp_idx] = (int(kx_norm * w), int(ky_norm * h))

            # Draw Skeleton Lines
            for p1, p2 in SKELETON:
                if p1 in kps and p2 in kps:
                    cv2.line(img, kps[p1], kps[p2], (0, 255, 0), 2)

            # Draw Keypoint Dots and Labels
            for idx, pt in kps.items():
                cv2.circle(img, pt, 4, (0, 0, 255), -1)
                cv2.putText(
                    img,
                    str(idx),
                    (pt[0] + 5, pt[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (255, 255, 255),
                    1,
                )

        cv2.imwrite(os.path.join(output_folder, base_name), img)
        print(f"Generated visualization for {base_name}")


if __name__ == "__main__":
    visualize_horse_skeleton("images", "labels", "visualizations")
