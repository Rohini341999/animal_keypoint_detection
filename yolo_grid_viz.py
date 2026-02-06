import os
import random
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image, ImageFilter
from pathlib import Path

def generate_grids_and_heatmaps():
    base_viz_dir = Path("/home/roha/Downloads/horse_26kp-20260206T005238Z-1-001/horse_26kp/visualizations")
    output_dir = base_viz_dir / "grid_comparisons"
    os.makedirs(output_dir, exist_ok=True)

    # 1. Explicitly list the models we care about (ignoring 'train', 'grid_comparisons', etc.)
    target_models = [
        "yolo11n_tuned", "yolo26s_tuned", "yolov11n-pose-horse", 
        "yolov26s-pose-horse", "yolov8s-pose-horse", "yolov8s_tuned"
    ]
    
    model_folders = [base_viz_dir / m for m in target_models if (base_viz_dir / m).is_dir()]
    num_models = len(model_folders)
    
    if num_models == 0:
        print("❌ No valid model folders found!")
        return

    # 2. Find common images
    first_folder = model_folders[0]
    common_images = [img for img in os.listdir(first_folder) if img.lower().endswith(('.jpg', '.jpeg', '.png'))]
    sample_images = random.sample(common_images, min(5, len(common_images)))

    # 3. Grid Settings
    cols = 3
    rows = (num_models + cols - 1) // cols

    for img_name in sample_images:
        for mode in ['standard', 'heatmap']:
            fig, axes = plt.subplots(rows, cols, figsize=(20, 6 * rows))
            axes = axes.flatten()
            
            suffix = "_heatmap" if mode == 'heatmap' else ""
            print(f"🖼️ Processing {img_name} ({mode})...")

            for i in range(len(axes)):
                if i < num_models:
                    img_path = model_folders[i] / img_name
                    if img_path.exists():
                        img = Image.open(img_path).convert("RGB")
                        
                        if mode == 'heatmap':
                            # Simulate a heatmap by extracting bright spots (keypoints) and blurring
                            # In a real scenario, this would use model confidence scores
                            bw_img = img.convert("L")
                            heatmap = bw_img.filter(ImageFilter.GaussianBlur(radius=15))
                            axes[i].imshow(img)
                            axes[i].imshow(heatmap, cmap='jet', alpha=0.5)
                        else:
                            axes[i].imshow(img)
                            
                        axes[i].set_title(f"{model_folders[i].name}", fontsize=14, fontweight='bold')
                
                axes[i].axis('off')

            plt.tight_layout()
            save_name = f"comparison_{Path(img_name).stem}{suffix}.png"
            plt.savefig(output_dir / save_name, dpi=150)
            plt.close()

    print(f"\n✅ Success! Grids and Heatmaps saved to: {output_dir}")

if __name__ == "__main__":
    generate_grids_and_heatmaps()
