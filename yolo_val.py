import argparse
import os
import json
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from ultralytics import YOLO

def run_validation():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs_dir", type=str, required=True)
    parser.add_argument("--data", type=str, required=True)
    parser.add_argument("--viz_dir", type=str, default="./visualizations")
    parser.add_argument("--device", type=str, default="cpu")
    args = parser.parse_args()

    results_data = []
    runs_path = Path(args.runs_dir)
    
    # Correct key names for Ultralytics 8.4+
    # We will search for these keys dynamically
    MAP_KEY_CANDIDATES = ['metrics/mAP50-95(P)', 'metrics/mAP50-95(p)', 'pose/mAP50-95']

    exp_folders = sorted([f for f in runs_path.iterdir() if f.is_dir()])

    for exp in exp_folders:
        weights_path = exp / "weights" / "best.pt"
        if not weights_path.exists():
            continue

        print(f"🔍 Validating: {exp.name}")
        model = YOLO(str(weights_path))

        # Run Val
        val_results = model.val(data=args.data, device=args.device, plots=False)
        
        # --- FIX: Dynamic Key Extraction ---
        res_dict = val_results.results_dict
        mp = 0.0
        for key in MAP_KEY_CANDIDATES:
            if key in res_dict:
                mp = res_dict[key]
                break
        
        # Speed & Size
        speed_ms = val_results.speed['inference']
        fps = 1000 / speed_ms
        model_size = os.path.getsize(weights_path) / (1024 * 1024)

        results_data.append({
            "Experiment": exp.name,
            "mAP50-95_Pose": round(mp, 4),
            "Latency_ms": round(speed_ms, 2),
            "FPS": round(fps, 1),
            "Size_MB": round(model_size, 2),
            "Fitness": round(mp / (speed_ms + 1), 5) # Assignment "Efficiency" metric
        })

        # Save Visualizations
        save_path = Path(args.viz_dir) / exp.name
        model.predict(
            source=args.data.replace('dataset.yaml', 'images/val'), # Dynamic source
            save=True, 
            project=args.viz_dir, 
            name=exp.name, 
            device=args.device,
            max_det=2, # Keep it clean
            exist_ok=True,
            conf=0.3
        )

    # Output & CSV
    df = pd.DataFrame(results_data)
    df.to_csv("val_summary.csv", index=False)
    print("\n" + "="*30)
    print(df.to_markdown())
    print("="*30)

    generate_assignment_plot(df)

def generate_assignment_plot(df):
    plt.figure(figsize=(10, 6))
    # Standard symbols for your 6 models
    markers = ['o', 'v', 's', 'p', '*', 'D']
    
    for i, (idx, row) in enumerate(df.iterrows()):
        plt.scatter(
            row['Size_MB'], 
            row['Latency_ms'], 
            s=row['mAP50-95_Pose']*10000 + 100, # Bubble size = accuracy
            label=f"{row['Experiment']}",
            marker=markers[i % len(markers)],
            alpha=0.6,
            edgecolors='black'
        )

    plt.title("Horse Pose Estimation: Complexity vs Performance", fontsize=14)
    plt.xlabel("Model Size (MB) - Parameter Count Proxy")
    plt.ylabel("Inference Latency (ms) - CPU")
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.legend(title="Experiments", loc='center left', bbox_to_anchor=(1, 0.5))
    plt.tight_layout()
    plt.savefig("performance_tradeoff.png")
    print("📈 Trade-off plot generated: performance_tradeoff.png")

if __name__ == "__main__":
    run_validation()
