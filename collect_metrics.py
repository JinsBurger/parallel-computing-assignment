import os
import re
import csv

MRTA_LOG_PATH = "/tmp/MRTA.log"
MRTA_CMD = "./MRTA parse"

def parse_latest_metrics(log_path):
    with open(log_path, "r") as f:
        lines = f.read().splitlines()

    latest_observed_ratio = 0.0
    found_tasks = 0
    total_tasks = 0

    in_task_block = False
    task_lines = []

    for l in lines:
        if l.startswith("End Object map"):
            observed_cnt = int(l.split(":")[1].strip())
            map_size = 30  # 고정
            total_cells = map_size * map_size
            latest_observed_ratio = (observed_cnt / total_cells) * 100

        if l.startswith("Start Task Info"):
            in_task_block = True
            task_lines = []
        elif l.startswith("End Task Info"):
            in_task_block = False
            task_lines = [line for line in task_lines if line.strip() != '']
            total_tasks = int(task_lines[1].split(":")[1].split(",")[0].strip())
            found_tasks = 0  # 💡 여기서 초기화가 반드시 필요
            for line in task_lines[3:]:
                if "True" in line:
                    found_match = re.search(r'\s*(\d+)\s*\(\s*\d+,\s*\d+\)\s*(True|False)', line)
                    if found_match and found_match.group(2) == "True":
                        found_tasks += 1
        elif in_task_block:
            task_lines.append(l)

    return latest_observed_ratio, found_tasks, total_tasks



def run_experiments(n=1000, csv_path="mrta_summary.csv"):
    total_observed_ratio = 0.0
    total_found_tasks = 0
    total_total_tasks = 0

    max_observed_ratio = float('-inf')
    min_observed_ratio = float('inf')

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Run", "ObservedRatio(%)", "FoundTasks", "TotalTasks"])

        for i in range(n):
            print(f"Running MRTA... ({i+1}/{n})")
            os.system(f"{MRTA_CMD} > {MRTA_LOG_PATH}")
            observed_ratio, found, total = parse_latest_metrics(MRTA_LOG_PATH)
            writer.writerow([i+1, f"{observed_ratio:.2f}", found, total])

            total_observed_ratio += observed_ratio
            total_found_tasks += found
            total_total_tasks += total

            max_observed_ratio = max(max_observed_ratio, observed_ratio)
            min_observed_ratio = min(min_observed_ratio, observed_ratio)

    avg_observed_ratio = total_observed_ratio / n
    avg_found_tasks = total_found_tasks / n
    avg_total_tasks = total_total_tasks / n
    avg_found_ratio = (avg_found_tasks / avg_total_tasks) * 100 if avg_total_tasks > 0 else 0.0

    print(f"\n[✓] {n}회 실행 결과가 '{csv_path}'에 저장되었습니다.")
    print(f"📊 평균 탐색된 맵 비율: {avg_observed_ratio:.2f}%")
    print(f"📊 최대 탐색 비율: {max_observed_ratio:.2f}%")
    print(f"📊 최소 탐색 비율: {min_observed_ratio:.2f}%")
    print(f"📊 평균 발견 Task 수: {avg_found_tasks:.2f} / {avg_total_tasks:.2f} ({avg_found_ratio:.2f}%)")



if __name__ == "__main__":
    run_experiments(n=1000)
