import argparse
import os
import json
from ultralytics import YOLO

def train_horse_pose():
    parser = argparse.ArgumentParser(description="Flexible YOLO Pose Training Script")

    # Required Arguments
    parser.add_argument("--project", type=str, required=True, help="Directory to save results")
    parser.add_argument("--exp", type=str, help="Experiment name (e.g., 'tuned_model_v1')")
    
    # Model and Data
    parser.add_argument("--model", type=str, default="yolo11n-pose.pt", help="Model file (v8, v11, etc.)")
    parser.add_argument("--data", type=str, default="./dataset.yaml", help="Path to data.yaml")
    
    # Standard Overrides
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=-1)
    parser.add_argument("--patience", type=int, default=30)

    # Capture all other YOLO hyperparameters (lr0, momentum, weight_decay, etc.)
    args, unknown = parser.parse_known_args()

    # Logic for required 'exp' parameter with custom error
    if not args.exp:
        print("\n" + "!"*60)
        print("ERROR: '--exp' argument is missing!")
        print("Why you need it: This string identifies your specific run/tuning attempt.")
        print("Example: python train.py --project ./runs/pose --exp 'lr_0.01_batch_16'")
        print("!"*60 + "\n")
        return

    # 1. Initialize Model
    model = YOLO(args.model)

    # 2. Prepare Training Config
    # Start with the explicit arguments
    train_kwargs = {
        "data": args.data,
        "epochs": args.epochs,
        "imgsz": args.imgsz,
        "batch": args.batch,
        "patience": args.patience,
        "project": args.project,
        "name": args.exp
    }

    # 3. Parse 'unknown' args into dictionary for hyperparameters
    # Converts --lr0 0.01 into {'lr0': 0.01}
    hyp_overrides = {}
    for i in range(0, len(unknown), 2):
        key = unknown[i].lstrip('-')
        val = unknown[i+1]
        # Try to convert numeric values
        try:
            val = float(val) if '.' in val else int(val)
        except ValueError:
            pass 
        hyp_overrides[key] = val

    # Merge overrides into main config
    train_kwargs.update(hyp_overrides)

    # 4. Save metadata for tracking
    save_dir = os.path.join(args.project, args.exp)
    os.makedirs(save_dir, exist_ok=True)
    
    with open(os.path.join(save_dir, "metadata_track.txt"), "w") as f:
        f.write(f"Model: {args.model}\n")
        f.write("Full Config:\n")
        f.write(json.dumps(train_kwargs, indent=4))

    # 5. Start Training
    print(f"🚀 Starting training: {args.exp} using {args.model}...")
    model.train(**train_kwargs)

if __name__ == "__main__":
    train_horse_pose()
