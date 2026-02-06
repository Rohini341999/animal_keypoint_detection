import csv
import json
import os

import bpy
import mathutils

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_PATH = os.path.join(SCRIPT_DIR, "glb_files")
OUT_JSON = os.path.join(SCRIPT_DIR, "horse_model_map.json")
OUT_CSV = os.path.join(SCRIPT_DIR, "horse_model_map.csv")
TARGET_HEIGHT_M = 1.70


def clear_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()


def analyze_model(filename):
    file_path = os.path.join(BASE_PATH, filename)
    if not os.path.exists(file_path):
        return None

    clear_scene()

    try:
        bpy.ops.import_scene.gltf(filepath=file_path)
    except AttributeError:
        print("Error: Blender glTF import operator not available")
        return None

    meshes = [obj for obj in bpy.data.objects if obj.type == "MESH"]
    if not meshes:
        print(f"Warning: No meshes found in {filename}")
        return None

    min_x = min_y = min_z = float("inf")
    max_x = max_y = max_z = float("-inf")

    for obj in meshes:
        corners = [obj.matrix_world @ mathutils.Vector(corner) for corner in obj.bound_box]
        for corner in corners:
            min_x = min(min_x, corner.x)
            min_y = min(min_y, corner.y)
            min_z = min(min_z, corner.z)
            max_x = max(max_x, corner.x)
            max_y = max(max_y, corner.y)
            max_z = max(max_z, corner.z)

    dim_x = max_x - min_x
    dim_y = max_y - min_y
    dim_z = max_z - min_z
    scale_for_target_height = TARGET_HEIGHT_M / dim_z if dim_z > 0 else 1.0
    center_z = (min_z + max_z) * 0.5

    return {
        "file": filename,
        "dimensions_m": {"x": round(dim_x, 4), "y": round(dim_y, 4), "z": round(dim_z, 4)},
        "bounds_m": {
            "min_x": round(min_x, 4),
            "max_x": round(max_x, 4),
            "min_y": round(min_y, 4),
            "max_y": round(max_y, 4),
            "min_z": round(min_z, 4),
            "max_z": round(max_z, 4),
        },
        "mesh_count": len(meshes),
        "scale_to_1p70m_height": round(scale_for_target_height, 6),
        "spawn_z_offset_m": round(-min_z, 4),
        "center_z_m": round(center_z, 4),
    }


def write_outputs(records):
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump(records, f, indent=2)

    headers = [
        "file",
        "dim_x",
        "dim_y",
        "dim_z",
        "scale_to_1p70m_height",
        "spawn_z_offset_m",
        "mesh_count",
    ]
    with open(OUT_CSV, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=headers)
        writer.writeheader()
        for item in records:
            writer.writerow(
                {
                    "file": item["file"],
                    "dim_x": item["dimensions_m"]["x"],
                    "dim_y": item["dimensions_m"]["y"],
                    "dim_z": item["dimensions_m"]["z"],
                    "scale_to_1p70m_height": item["scale_to_1p70m_height"],
                    "spawn_z_offset_m": item["spawn_z_offset_m"],
                    "mesh_count": item["mesh_count"],
                }
            )


def main():
    files = sorted([f for f in os.listdir(BASE_PATH) if f.lower().endswith(".glb")])
    results = []

    print("Horse model inspection")
    print(f"Input path: {BASE_PATH}")
    print(f"Found {len(files)} GLB files\n")

    for filename in files:
        result = analyze_model(filename)
        if result is None:
            continue
        results.append(result)
        dims = result["dimensions_m"]
        print(
            f"{filename}: dims=({dims['x']}, {dims['y']}, {dims['z']}) "
            f"scale_to_1.70m={result['scale_to_1p70m_height']} "
            f"z_offset={result['spawn_z_offset_m']}"
        )

    write_outputs(results)
    print(f"\nWrote mapping files:\n- {OUT_JSON}\n- {OUT_CSV}")


if __name__ == "__main__":
    main()
