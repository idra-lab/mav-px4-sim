from unseen_eval.unseen_eval_lib import *



def import_csv_table(filepath: str) -> dict:
    """Imports the evaluation table from a CSV file."""

    pd_table = pd.read_csv(filepath)

    return pd_table

def pd_to_latex_table(pd_table: pd.DataFrame, remove_names: bool) -> str:
    """Converts a pandas DataFrame to a LaTeX table string."""
    if remove_names:
        pd_table.drop(pd_table.columns[0], axis=1, inplace=True)


    latex_str = pd_table.to_latex(index=True)

   

    return latex_str

def store_latex_table(pd_table: pd.DataFrame, filepath: str, remove_names:bool = True):
    """Saves the evaluation table as a LaTeX table."""

    latex_filename = filepath 
    with open(latex_filename, "w") as f:
        latex_str = pd_to_latex_table(pd_table, remove_names=remove_names)
        f.write(latex_str)



if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Generate LaTeX table from CSV evaluation results.")
    parser.add_argument("csv_filepath", type=str, help="Path to the CSV file containing evaluation results.")
    parser.add_argument("output_filepath", type=str, help="Path to save the LaTeX table (without extension).")

    args = parser.parse_args()

    pd_table = import_csv_table(args.csv_filepath)
    store_latex_table(pd_table, args.output_filepath, True)


# python3 src/unseen_eval/unseen_eval/tex_from_csv.py bags/evaluation_results/dynamic_performance.csv bags/evaluation_results/dynamic_performance.tex

# python3 src/unseen_eval/unseen_eval/tex_from_csv.py bags/evaluation_results/geometric_performance.csv bags/evaluation_results/geometric_performance.tex
# python3 src/unseen_eval/unseen_eval/tex_from_csv.py bags/evaluation_results/vision_performance.csv bags/evaluation_results/vision_performance.tex
