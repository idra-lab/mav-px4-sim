from unseen_eval.unseen_eval_lib import *

from unseen_eval.generate_latex_table import import_result_table, store_latex_table, dict_to_latex_table, store_csv_table


def create_subtable(table: dict, metrics: list) -> dict:
    """Creates a subtable containing only the specified metrics."""
    subtable = {}
    for run in table.keys():
        subtable[run] = {}
        for metric in metrics:
            if metric in table[run]:
                subtable[run][metric] = table[run][metric]
            else:
                print(f"Warning: Metric '{metric}' not found for run '{run}'.")

                
    return subtable


if __name__ == "__main__": 
    parser = argparse.ArgumentParser(description="Generate LaTeX and CSV tables from evaluation results.")
    parser.add_argument("input_file", type=str, help="Path to the input evaluation table file.")
    parser.add_argument("config_file", type=str, help="Path to the config file.")
    args = parser.parse_args()

    config = yaml.safe_load(open(args.config_file))

    table = import_result_table(args.input_file)

    output_folder = os.path.dirname(args.input_file)

    subtables = config.get("subtables", [])

    for subtable in subtables:
        subtable_name = list(subtable.keys())[0]
        subtable_metrics = subtable[subtable_name]
        print(f"Generating subtable '{subtable_name}' with metrics: {subtable_metrics}")

        new_output_path = os.path.join(output_folder, f"{subtable_name}_results")

        # Create subtable
        table_subset = create_subtable(table, subtable_metrics)
        store_latex_table(table_subset, new_output_path)
        store_csv_table(table_subset, new_output_path)



# python3 src/unseen_eval/unseen_eval/generate_subtables.py bags/evaluation_results/summary_results.pkl  src/unseen_eval/config/evaluation_config.yaml