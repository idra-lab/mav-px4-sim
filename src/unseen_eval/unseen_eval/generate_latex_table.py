from unseen_eval.unseen_eval_lib import *


def import_result_table(filepath: str) -> dict:
    """Imports the evaluation table from a text file."""

    with open(filepath, "rb") as f:
        table = pickle.load(f)

    return table

def dict_to_latex_table(table: dict) -> str:
    """Converts a dictionary to a LaTeX table string."""

    # print(table)
    entries_names = table[list(table.keys())[0]].keys()

    print(entries_names)

    shortened_names = []

    # Get initials of each entry name
    for name in entries_names:
        words = name.split(" ")

        # initials = [w[0] + "." for w in words if len(w) > 3] + [w + " " for w in words if len(w) <= 3]  # Avoid empty strings
        initials = []
        for word in words:
            if len(word) <= 3:
                initials.append(word + " ")
            else:
                initials.append(word[0] + ".")
        shortened_name = [letter  for letter in initials]
        shortened_names.append("".join(shortened_name))


    latex_str = "\\begin{tabular}{|l|" + "c|" * len(entries_names) + "}\n"
    latex_str += "\\hline\n"
    
    # Header
    headers = ["Metric"] + list(entries_names)
    latex_str += " & ".join(headers) + " \\\\\n"
    latex_str += "\\hline\n"
    
    # Rows
    for key, values in table.items():
        row = [key] + [f"{v:.4f}" if isinstance(v, float) else str(v) for v in values.values()]
        latex_str += " & ".join(row) + " \\\\\n"
    
    latex_str += "\\hline\n"
    latex_str += "\\end{tabular}\n"
    
    return latex_str


def store_latex_table(table: dict, filepath: str):
    """Saves the evaluation table as a LaTeX table."""

    latex_filename = filepath + ".tex"
    with open(latex_filename, "w") as f:
        f.write(dict_to_latex_table(table))

def store_csv_table(table: dict, filepath: str):
    """Saves the evaluation table as a CSV file."""
    import csv

    csv_filename = filepath + ".csv"

    with open(csv_filename, "w", newline='') as f:
        writer = csv.writer(f)

        entries_names = table[list(table.keys())[0]].keys()

        # Header
        headers = ["Metric"] + list(entries_names)
        writer.writerow(headers)

        # Rows
        for key, values in table.items():
            row = [key] + [f"{v:.4f}" if isinstance(v, float) else str(v) for v in values.values()]
            writer.writerow(row)




if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate LaTeX table from evaluation results.")
    parser.add_argument("input_file", type=str, help="Path to the input evaluation table file.")
    parser.add_argument("output_file", type=str, help="Path to save the output table file.")
    args = parser.parse_args()

    table = import_result_table(args.input_file)
    store_latex_table(table, args.output_file)
    store_csv_table(table, args.output_file)


# python3 src/unseen_eval/unseen_eval/generate_latex_table.py bags/evaluation_results/summary_results.pkl bags/evaluation_results/summary_latex