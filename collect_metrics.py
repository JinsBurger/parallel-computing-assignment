from pathlib import Path
import os
import re
import csv

MRTA_LOG_PATH = "/tmp/MRTA.log"
MRTA_CMD = "./MRTA parse"

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
            found_tasks = 0
            for line in task_lines[3:]:
                if "True" in line:
                    found_match = re.search(r'\s*(\d+)\s*\(\s*\d+,\s*\d+\)\s*(True|False)', line)
                    if found_match and found_match.group(2) == "True":
                        found_tasks += 1
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

    print(f"Energys - Caterpillar: {caterpillar_energy}, Wheel: {wheel_energy}")
    return latest_observed_ratio, found_tasks, total_tasks, completed_tasks, drone_energy, caterpillar_energy, wheel_energy

def run_experiments(n=1000, map_size=30, csv_path="mrta_summary.csv"):
    total_observed_ratio = 0.0
    total_found_tasks = 0
    total_total_tasks = 0
    total_completed_tasks = 0

    max_observed_ratio = float('-inf')
    min_observed_ratio = float('inf')

    sum_drone_energy = 0
    sum_cat_energy = 0
    sum_wheel_energy = 0
    count_drone = 0
    count_cat = 0
    count_wheel = 0

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Run", "ObservedRatio(%)", "FoundTasks", "CompletedTasks", "TotalTasks"])

        for i in range(n):
            print(f"Running MRTA... ({i+1}/{n})")
            os.system(f"{MRTA_CMD} > {MRTA_LOG_PATH}")
            observed_ratio, found, total, completed, d_e, c_e, w_e = parse_latest_metrics(MRTA_LOG_PATH, map_size)
            writer.writerow([i+1, f"{observed_ratio:.2f}", found, completed, total])

            total_observed_ratio += observed_ratio
            total_found_tasks += found
            total_total_tasks += total
            total_completed_tasks += completed

            max_observed_ratio = max(max_observed_ratio, observed_ratio)
            min_observed_ratio = min(min_observed_ratio, observed_ratio)

            sum_drone_energy += sum(d_e)
            count_drone += len(d_e)
            sum_cat_energy += sum(c_e)
            count_cat += len(c_e)
            sum_wheel_energy += sum(w_e)
            count_wheel += len(w_e)
            
            print(f"Run {i+1} - Energy: Caterpillar={sum(c_e)}, Wheel={sum(w_e)}")

    avg_observed_ratio = total_observed_ratio / n
    avg_found_tasks = total_found_tasks / n
    avg_completed_tasks = total_completed_tasks / n
    avg_total_tasks = total_total_tasks / n
    avg_found_ratio = (avg_found_tasks / avg_total_tasks) * 100 if avg_total_tasks > 0 else 0.0
    avg_completed_ratio = (avg_completed_tasks / avg_total_tasks) * 100 if avg_total_tasks > 0 else 0.0

    avg_drone = sum_drone_energy / count_drone if count_drone else 0
    avg_cat = sum_cat_energy / count_cat if count_cat else 0
    avg_wheel = sum_wheel_energy / count_wheel if count_wheel else 0

    print(f"\n[✓] {n}회 실행 결과가 '{csv_path}'에 저장되었습니다.")
    print(f"📊 평균 탐색된 맵 비율: {avg_observed_ratio:.2f}%")
    print(f"📊 최대 탐색 비율: {max_observed_ratio:.2f}%")
    print(f"📊 최소 탐색 비율: {min_observed_ratio:.2f}%")
    print(f"📊 평균 발견 Task 수: {avg_found_tasks:.2f} / {avg_total_tasks:.2f} ({avg_found_ratio:.2f}%)")
    print(f"📊 평균 완료 Task 수: {avg_completed_tasks:.2f} / {avg_total_tasks:.2f} ({avg_completed_ratio:.2f}%)")
    print(f"📊 평균 DRONE 에너지: {avg_drone:.2f}")
    print(f"📊 평균 CATERPILLAR 에너지: {avg_cat:.2f}")
    print(f"📊 평균 WHEEL 에너지: {avg_wheel:.2f}")

run_experiments(n=100, map_size=30)
