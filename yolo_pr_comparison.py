import os
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
from pathlib import Path

def generate_adaptive_pr_grid():
    runs_dir = Path("/home/roha/Downloads/horse_26kp-20260206T005238Z-1-001/horse_26kp/runs/pose")
    output_dir = Path("/home/roha/Downloads/horse_26kp-20260206T005238Z-1-001/horse_26kp/visualizations/pr_comparisons")
    os.makedirs(output_dir, exist_ok=True)

    # 1. Collect only models that actually have the files
    valid_exps = []
    # We scan all directories in runs/pose to be truly dynamic
    for exp_path in sorted(runs_dir.iterdir()):
        if not exp_path.is_dir(): continue
        
        box_pr = exp_path / "BoxPR_curve.png"
        pose_pr = exp_path / "PosePR_curve.png"
        
        if box_pr.exists() and pose_pr.exists():
            valid_exps.append((exp_path.name, box_pr, pose_pr))

    num_models = len(valid_exps)
    if num_models == 0:
        print("❌ No PR curves found. Check your runs directory!")
        return

    print(f"📊 Found {num_models} models with PR curves. Adjusting grid...")

    # 2. Dynamic Grid Calculation
    # We want it to be as "square" as possible
    cols = math.ceil(math.sqrt(num_models))
    rows = math.ceil(num_models / cols)
    
    fig, axes = plt.subplots(rows, cols, figsize=(8 * cols, 6 * rows))
    
    # Flatten axes for easy iteration (handle case where subplots might be a single object)
    if num_models == 1:
        axes_list = [axes]
    else:
        axes_list = axes.flatten()

    # 3. Stitch and Plot
    for i, (name, box_path, pose_path) in enumerate(valid_exps):
        img_box = Image.open(box_path)
        img_pose = Image.open(pose_path)

        # Stitch Side-by-Side
        total_width = img_box.width + img_pose.width
        max_height = max(img_box.height, img_pose.height)
        combined = Image.new('RGB', (total_width, max_height), (255, 255, 255))
        combined.paste(img_box, (0, 0))
        combined.paste(img_pose, (img_box.width, 0))

        axes_list[i].imshow(combined)
        axes_list[i].set_title(f"{name}", fontsize=14, fontweight='bold')
        axes_list[i].axis('off')

    # 4. Hide empty slots
    for j in range(i + 1, len(axes_list)):
        axes_list[j].axis('off')

    plt.tight_layout()
    save_path = output_dir / f"PR_Comparison_{num_models}_models.png"
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"✅ Success! Dynamic grid saved to: {save_path}")

if __name__ == "__main__":
    generate_adaptive_pr_grid()
