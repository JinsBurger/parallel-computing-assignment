from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
import os
import re
import csv
import threading
import time

#MRTA_LOG_PATH = "/tmp/MRTA.log"
MRTA_CMD_TEMPLATE = "./MRTA parse {seed}"

test_seed = list(range(5000, 10000))
lock = threading.Lock()

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

    return latest_observed_ratio, found_tasks, total_tasks, completed_tasks, drone_energy, caterpillar_energy, wheel_energy

def run_single_experiment(seed, map_size):
    log_path = f"/tmp/MRTA_{seed}.log"  # 고유한 로그 경로 생성
    if os.path.exists(log_path):
        os.remove(log_path)
    os.system(f"{MRTA_CMD_TEMPLATE.format(seed=seed)} > {log_path} &")
    while True:
        if os.path.exists(log_path):
            if "Algorithm time : " in open(log_path, "r").read():
                break
        time.sleep(3)
    result = parse_latest_metrics(log_path, map_size)
    #os.remove(log_path)  # 로그 파일 삭제 (원하지 않으면 생략 가능)
    return result


def run_experiments_parallel(n=100, map_size=30, csv_path="mrta_summary.csv", max_workers=8):
    results = []
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = {executor.submit(run_single_experiment, test_seed[i], map_size): i+1 for i in range(n)}

        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Run", "ObservedRatio(%)", "FoundTasks", "CompletedTasks", "TotalTasks"])

            for future in as_completed(futures):
                i = futures[future]
                try:
                    observed_ratio, found, total, completed, d_e, c_e, w_e = future.result()
                    with lock:
                        writer.writerow([i, f"{observed_ratio:.2f}", found, completed, total])
                        results.append((observed_ratio, found, completed, total, d_e, c_e, w_e))
                        print(f"Run {i} - Found: {found}, Completed: {completed}")
                except Exception as e:
                    print(f"Error in Run {i}: {e}")

    total_observed_ratio = sum(r[0] for r in results)
    total_found_tasks = sum(r[1] for r in results)
    total_completed_tasks = sum(r[2] for r in results)
    total_total_tasks = sum(r[3] for r in results)

    all_d_e = sum((r[4] for r in results), [])
    all_c_e = sum((r[5] for r in results), [])
    all_w_e = sum((r[6] for r in results), [])

    n_runs = len(results)
    avg_observed_ratio = total_observed_ratio / n_runs
    avg_found_tasks = total_found_tasks / n_runs
    avg_completed_tasks = total_completed_tasks / n_runs
    avg_total_tasks = total_total_tasks / n_runs
    avg_found_ratio = (avg_found_tasks / avg_total_tasks) * 100 if avg_total_tasks else 0
    avg_completed_ratio = (avg_completed_tasks / avg_total_tasks) * 100 if avg_total_tasks else 0

    avg_drone = sum(all_d_e) / len(all_d_e) if all_d_e else 0
    avg_cat = sum(all_c_e) / len(all_c_e) if all_c_e else 0
    avg_wheel = sum(all_w_e) / len(all_w_e) if all_w_e else 0

    print(f"\n[✓] {n_runs}회 실행 결과가 '{csv_path}'에 저장되었습니다.")
    print(f"📊 평균 탐색된 맵 비율: {avg_observed_ratio:.2f}%")
    print(f"📊 평균 발견 Task 수: {avg_found_tasks:.2f} / {avg_total_tasks:.2f} ({avg_found_ratio:.2f}%)")
    print(f"📊 평균 완료 Task 수: {avg_completed_tasks:.2f} / {avg_total_tasks:.2f} ({avg_completed_ratio:.2f}%)")
    print(f"📊 평균 DRONE 에너지: {avg_drone:.2f}")
    print(f"📊 평균 CATERPILLAR 에너지: {avg_cat:.2f}")
    print(f"📊 평균 WHEEL 에너지: {avg_wheel:.2f}")

if __name__ == "__main__":
    run_experiments_parallel(n=100, map_size=30, max_workers=16)
