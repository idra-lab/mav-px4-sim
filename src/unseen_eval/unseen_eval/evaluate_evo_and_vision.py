from unseen_eval.unseen_eval_lib import *


from unseen_eval.evo_evaluate_trajectory import create_table_and_plots
from unseen_eval.visual_evaluation import analyze_sequence

import yaml

def save_table(table: dict, filepath: str):
    """Saves the evaluation table to a text file."""
    with open(filepath, "wb") as f:
        pickle.dump(table, f)



def evaluate_evo_and_vision(config_file: str):

    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    dataset_folder = config.get("dataset_folder", "bags/dataset/")
    output_folder = config.get("output_folder", "bags/evaluation_results/")

    # Ensure the output folder exists
    os.makedirs(output_folder, exist_ok=True)
    
    omit_names = config.get("omit_names", [])

    summary_metrics_names = config.get("summary_metrics", [
        "APE RMSE",
        "RPE RMSE",
        "mean_wobble",
        "mean_blur_varlap",
        "mean_blur_freq_ratio",
        "median__tracking_residual",
        "p95_tracking_residual"
    ])

    summary_dict = {}

    for run in os.listdir(dataset_folder):
        run_path = os.path.join(dataset_folder, run + "/")
        if not os.path.isdir(run_path):
            continue

        

        # Skip omitted names
        if any(omit_name in run.lower() for omit_name in omit_names):
            print(f"Skipping run {run} due to omit_names filter.")
            continue

        print(run_path)

        try: 
                
            table = create_table_and_plots(run_path, show_plot=False)

            # print(table)

            rgb_folder = os.path.join(run_path, "rgb")

            res_dict = analyze_sequence(rgb_folder)

            # print("res dict:", res_dict)

            # Combine results
            combined_results = table | res_dict

            # print("combined: ", combined_results)

            # Save combined results
            save_table(combined_results, run_path + "combined_results.txt")

            short_results = {k: v for k, v in combined_results.items() if k in summary_metrics_names}

            # print(short_results)

            summary_dict[run] = short_results
            # print(summary_dict)
        except Exception as e:
            print(f"Error processing run {run}: {e}")
            continue

    # Save summary
    with open(os.path.join(output_folder, "summary_results.pkl"), "wb") as f: 
        pickle.dump(summary_dict, f)

    

    print("EVO and visual evaluation completed.")

def main():
    argparser = argparse.ArgumentParser(description="Evaluate EVO metrics and visual shakiness.")
    argparser.add_argument("config_file", help="Path to the config file containing dataset folder.")
    args = argparser.parse_args()

    evaluate_evo_and_vision(args.config_file)

if __name__ == "__main__":
    main()


# python3 src/unseen_eval/unseen_eval/evaluate_evo_and_vision.py src/unseen_eval/config/evaluation_config.yaml