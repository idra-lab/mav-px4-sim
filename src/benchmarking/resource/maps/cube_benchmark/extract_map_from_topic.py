import argparse
import sys
import yaml

#!/usr/bin/env python3
"""
extract_map_from_topic.py

Read a YAML file, find all lists named "points", extract x,y,z fields and
write them to a space-separated CSV file (one point per line: "x y z").
"""


def main(args): 

    parser = argparse.ArgumentParser(
        description="Extract points from a YAML file and write to CSV."
    )
    parser.add_argument(
        "input_yaml",
        type=str,
        help="Path to the input YAML file containing points.",
    )
    parser.add_argument(
        "output_csv",
        type=str,
        help="Path to the output CSV file.",
    )
    args = parser.parse_args()

    try:
        with open(args.input_yaml, 'r') as infile:
            data = yaml.safe_load(infile)
    except Exception as e:
        print(f"Error reading YAML file: {e}", file=sys.stderr)
        sys.exit(1)

    points = []

    try:
        with open(args.output_csv, "w") as outfile:

            for key in data: 
                for pp_id in data[key]["points"]:
                    
                    point = data[key]["points"][pp_id]
                    pos = point["position"]
                    outfile.write(f"{pos[2]} {-pos[0]} {-pos[1]}\n")
        
    except Exception as e:
        print(f"Error writing output file: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main(sys.argv)