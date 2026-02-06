import os
import pandas as pd
import matplotlib
matplotlib.use('Agg') # Prevents TclError
import matplotlib.pyplot as plt
from pathlib import Path
from ultralytics import YOLO

def run_safe_benchmark():
    runs_dir = "/home/roha/Downloads/horse_26kp-20260206T005238Z-1-001/horse_26kp/runs/pose"
    runs_path = Path(runs_dir)
    benchmark_results = []

    exp_folders = sorted([f for f in runs_path.iterdir() if f.is_dir()])

    for exp in exp_folders:
        weights_path = exp / "weights" / "best.pt"
        if not weights_path.exists():
            continue

        print(f"🚀 Benchmarking: {exp.name}")
        try:
            model = YOLO(str(weights_path))
            
            # Use the official benchmark tool (restricted to ONNX for speed)
            # This handles warming up the CPU and averaging iterations automatically
            df_bench = model.benchmarks(format='onnx', imgsz=640, device='cpu')
            
            # The benchmark method returns a dataframe or list of results
            # We extract the inference time (ms)
            onnx_time = df_bench.loc[df_bench['Format'] == 'ONNX', 'Inference (ms)'].values[0]
            
            benchmark_results.append({
                "Model": exp.name,
                "Inference_ms": round(onnx_time, 2),
                "FPS": round(1000 / onnx_time, 1),
                "Size_MB": round(os.path.getsize(weights_path) / (1024*1024), 2)
            })
        except Exception as e:
            print(f"❌ Failed to benchmark {exp.name}: {e}")

    # Save and Plot
    df = pd.DataFrame(benchmark_results)
    df.to_csv("hardware_benchmark.csv", index=False)
    print("\n" + df.to_markdown())

    # Save a quick bar chart of FPS
    plt.figure(figsize=(10, 5))
    plt.bar(df['Model'], df['FPS'], color='teal')
    plt.title("Hardware Throughput (CPU FPS)")
    plt.ylabel("Frames Per Second")
    plt.xticks(rotation=45)
    plt.tight_layout()
    plt.savefig("benchmark_fps_comparison.png")

if __name__ == "__main__":
    run_safe_benchmark()
