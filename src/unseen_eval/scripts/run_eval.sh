#!/bin/bash


python3 src/unseen_eval/unseen_eval/evaluate_evo_and_vision.py src/unseen_eval/config/evaluation_config.yaml

python3 src/unseen_eval/unseen_eval/generate_latex_table.py bags/evaluation_results/summary_results.pkl bags/evaluation_results/summary_latex

python3 src/unseen_eval/unseen_eval/generate_subtables.py bags/evaluation_results/summary_results.pkl  src/unseen_eval/config/evaluation_config.yaml

