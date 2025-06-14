import os
import re
import csv
import time
from tqdm import tqdm
from concurrent.futures import ThreadPoolExecutor, as_completed
import subprocess

MRTA_CMD_TEMPLATE = "../MRTA parse {seed} > {log_path}"


def parse_latest_metrics(log_path, map_size):
    with open(log_path, "r") as f:
        lines = f.read().splitlines()

    latest_observed_ratio = 0.0
    found_tasks = 0
    total_tasks = 0
    completed_tasks = 0

    in_task_block = False
    task_lines = []

    drone_energy = []
    caterpillar_energy = []
    wheel_energy = []

    robot_line_pattern = re.compile(r'\d+\s+(DRONE|CATERPILLAR|WHEEL)\s+\(\d+,\s+\d+\)\s+(\d+)')

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
            active_tasks = int(task_lines[1].split(",")[2].strip().split()[-1])
            found_tasks = active_tasks + completed_tasks
        elif in_task_block:
            task_lines.append(l)

        match = robot_line_pattern.match(l.strip())
        if match:
            rtype = match.group(1)
            energy = int(match.group(2))
            if rtype == 'DRONE':
                drone_energy.append(energy)
            elif rtype == 'CATERPILLAR':
                caterpillar_energy.append(energy)
            elif rtype == 'WHEEL':
                wheel_energy.append(energy)

    return latest_observed_ratio, found_tasks, total_tasks, completed_tasks, drone_energy, caterpillar_energy, wheel_energy


def run_single_experiment(seed, map_size):
    log_path = f"/tmp/MRTA_{seed}.log"
    if os.path.exists(log_path):
        os.remove(log_path)
    subprocess.run(MRTA_CMD_TEMPLATE.format(seed=seed, log_path=log_path), shell=True)

    for _ in range(60):
        if os.path.exists(log_path) and "Algorithm time :" in open(log_path, "r").read():
            break
        time.sleep(1)

    result = parse_latest_metrics(log_path, map_size)
    os.remove(log_path)
    return result


def run_group(group_id, map_size, n=100, max_workers=16):
    base_seed = group_id * n
    results = []
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = {
            executor.submit(run_single_experiment, base_seed + i, map_size): i
            for i in range(n)
        }
        for future in tqdm(as_completed(futures), total=n, desc=f"Group {group_id}"):
            try:
                results.append(future.result())
            except Exception as e:
                print(f"[ERROR] Seed {base_seed + futures[future]}: {e}")

    return results


def summarize_results(results):
    total_observed_ratio = sum(r[0] for r in results)
    total_found_tasks = sum(r[1] for r in results)
    total_total_tasks = sum(r[2] for r in results)
    total_completed_tasks = sum(r[3] for r in results)

    avg_observed_ratio = total_observed_ratio / len(results)
    avg_found_tasks = total_found_tasks / len(results)
    avg_completed_tasks = total_completed_tasks / len(results)
    avg_total_tasks = total_total_tasks / len(results)

    avg_found_ratio = (avg_found_tasks / avg_total_tasks) * 100 if avg_total_tasks else 0
    avg_completed_ratio = (avg_completed_tasks / avg_total_tasks) * 100 if avg_total_tasks else 0

    return avg_observed_ratio, avg_found_tasks, avg_completed_tasks, avg_total_tasks, avg_found_ratio, avg_completed_ratio


def run_all_experiments(map_size=30, n_group=100, n_per_group=100, csv_path="summary_10000.csv"):
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "GroupID", "AvgObserved(%)", "AvgFoundTasks", "AvgCompletedTasks",
            "AvgTotalTasks", "FoundRatio(%)", "CompletedRatio(%)"
        ])

        for group_id in range(n_group):
            results = run_group(group_id, map_size, n=n_per_group)
            summary = summarize_results(results)
            writer.writerow([group_id, *["%.2f" % val for val in summary]])
            print(f"✅ Group {group_id} done! -> seed : {group_id * n_per_group} ~ {group_id * n_per_group + n_per_group - 1}")
            print(f"  - Avg Observed Ratio: {summary[0]:.2f}%")
            print(f"  - Avg Found Tasks: {summary[1]:.2f} / {summary[3]:.2f} ({summary[4]:.2f}%)")
            print(f"  - Avg Completed Tasks: {summary[2]:.2f} / {summary[3]:.2f} ({summary[5]:.2f}%)")


run_all_experiments(map_size=30, n_group=100, n_per_group=100, csv_path="summary_10000.csv")
