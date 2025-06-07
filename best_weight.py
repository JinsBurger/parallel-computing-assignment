import os
import re
import csv
from tqdm import tqdm

MRTA_LOG_PATH = "/tmp/MRTA.log"
MRTA_CMD_TEMPLATE = "WEIGHT_TICK={tick} WEIGHT_DIST_OTHER={other} WEIGHT_DIST_SELF={self} WEIGHT_DIST_ROBOT={robot} ./MRTA parse"

def parse_latest_metrics(log_path, map_size):
    with open(log_path, "r") as f:
        lines = f.read().splitlines()

    latest_observed_ratio = 0.0
    found_tasks = 0
    total_tasks = 0
    completed_tasks = 0

    in_task_block = False
    task_lines = []

    for l in lines:
        if l.startswith("End Object map"):
            observed_cnt = int(l.split(":")[1].strip())
            total_cells = map_size * map_size
            latest_observed_ratio = (observed_cnt / total_cells) * 100

        if l.startswith("Start Task Info"):
            in_task_block = True
            task_lines = []
        elif l.startswith("End Task Info"):
            in_task_block = False
            task_lines = [line for line in task_lines if line.strip() != '']
            total_tasks = int(task_lines[1].split(":")[1].split(",")[0].strip())
            completed_tasks = int(task_lines[1].split(",")[-1].strip().split()[-1])
            found_tasks = 0
            for line in task_lines[3:]:
                if "True" in line:
                    found_match = re.search(r'\s*(\d+)\s*\(\s*\d+,\s*\d+\)\s*(True|False)', line)
                    if found_match and found_match.group(2) == "True":
                        found_tasks += 1
        elif in_task_block:
            task_lines.append(l)

    return latest_observed_ratio, found_tasks, total_tasks, completed_tasks


def run_for_weight(weight_set, n, map_size):
    tick, other, self_w, robot = weight_set
    total_observed = 0
    total_found = 0
    total_total = 0
    total_completed = 0

    # tqdm 쓰고 싶지 않다면 아래 주석 사용.
    # for i in range(n):
    #     cmd = MRTA_CMD_TEMPLATE.format(tick=tick, other=other, self=self_w, robot=robot)
    #     os.system(f"{cmd} > {MRTA_LOG_PATH}")
    #     observed, found, total, completed = parse_latest_metrics(MRTA_LOG_PATH, map_size)

    #     total_observed += observed
    #     total_found += found
    #     total_total += total
    #     total_completed += completed
    
    for i in tqdm(range(n), desc=f"WEIGHT {weight_set}", leave=False):
        cmd = MRTA_CMD_TEMPLATE.format(tick=tick, other=other, self=self_w, robot=robot)
        os.system(f"{cmd} > {MRTA_LOG_PATH}")
        observed, found, total, completed = parse_latest_metrics(MRTA_LOG_PATH, map_size)

        total_observed += observed
        total_found += found
        total_total += total
        total_completed += completed

    avg_observed = total_observed / n
    avg_found = total_found / n
    avg_completed = total_completed / n

    return avg_observed, avg_found, avg_completed



def main(n, map_size):
    # weight_combinations = [
    #     (1, 6, 8, 2),
    #     (1, 8, 6, 2),
    #     (2, 6, 8, 2),
    #     (1, 10, 10, 3),
    #     (2, 8, 10, 4),
    #     (3, 5, 6, 2)
    # ]
    weight_combinations = [
        (1, 6, 8, 2),
        (1, 8, 6, 2),
    ]    
    output_path = "best_weight_summary.csv"

    with open(output_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["WEIGHT_TICK", "WEIGHT_DIST_OTHER", "WEIGHT_DIST_SELF", "WEIGHT_DIST_ROBOT",
                         "AvgObserved(%)", "AvgFoundTasks", "AvgCompletedTasks"])

        for weights in weight_combinations:
            # Run the MRTA simulation for each weight set
            print(f"Running for weight set: {weights}")
            avg_obs, avg_found, avg_comp = run_for_weight(weights, n=n, map_size=map_size)
            writer.writerow([*weights, f"{avg_obs:.2f}", f"{avg_found:.2f}", f"{avg_comp:.2f}"])
            print(f"→ Result: Obs={avg_obs:.2f}%, Found={avg_found:.2f}, Completed={avg_comp:.2f}\n")

    print(f"\n[✓] 결과가 '{output_path}'에 저장되었습니다.")


if __name__ == "__main__":
    main(n=100, map_size=20)
