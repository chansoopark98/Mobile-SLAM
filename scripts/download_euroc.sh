#!/bin/bash
# Download EuRoC MAV Dataset (ASL format)
# Usage: ./scripts/download_euroc.sh [sequence_name]
# Example: ./scripts/download_euroc.sh MH_01_easy

set -e

DATASET_DIR="$(cd "$(dirname "$0")/.." && pwd)/data/EuRoC"
BASE_URL="http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset"

# Available sequences
declare -A SEQUENCES=(
    ["MH_01_easy"]="machine_hall/MH_01_easy/MH_01_easy.zip"
    ["MH_02_easy"]="machine_hall/MH_02_easy/MH_02_easy.zip"
    ["MH_03_medium"]="machine_hall/MH_03_medium/MH_03_medium.zip"
    ["MH_04_difficult"]="machine_hall/MH_04_difficult/MH_04_difficult.zip"
    ["MH_05_difficult"]="machine_hall/MH_05_difficult/MH_05_difficult.zip"
    ["V1_01_easy"]="vicon_room1/V1_01_easy/V1_01_easy.zip"
    ["V1_02_medium"]="vicon_room1/V1_02_medium/V1_02_medium.zip"
    ["V1_03_difficult"]="vicon_room1/V1_03_difficult/V1_03_difficult.zip"
    ["V2_01_easy"]="vicon_room2/V2_01_easy/V2_01_easy.zip"
    ["V2_02_medium"]="vicon_room2/V2_02_medium/V2_02_medium.zip"
    ["V2_03_difficult"]="vicon_room2/V2_03_difficult/V2_03_difficult.zip"
)

download_sequence() {
    local seq_name=$1
    local seq_path=${SEQUENCES[$seq_name]}

    if [ -z "$seq_path" ]; then
        echo "Error: Unknown sequence '$seq_name'"
        echo "Available sequences: ${!SEQUENCES[*]}"
        exit 1
    fi

    local output_dir="$DATASET_DIR/$seq_name"
    local zip_file="$DATASET_DIR/${seq_name}.zip"
    local url="$BASE_URL/$seq_path"

    if [ -d "$output_dir/mav0" ]; then
        echo "Sequence '$seq_name' already exists at $output_dir"
        return 0
    fi

    echo "Downloading $seq_name from $url ..."
    mkdir -p "$DATASET_DIR"
    wget -c -O "$zip_file" "$url"

    echo "Extracting $seq_name ..."
    unzip -q -o "$zip_file" -d "$output_dir"
    rm -f "$zip_file"

    echo "Done: $output_dir"
}

if [ $# -eq 0 ]; then
    echo "Downloading default sequence: MH_01_easy"
    download_sequence "MH_01_easy"
else
    for seq in "$@"; do
        download_sequence "$seq"
    done
fi

echo "All downloads complete."
