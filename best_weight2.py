import os
import re
import csv
import time
from tqdm import tqdm
from concurrent.futures import ThreadPoolExecutor, as_completed
import subprocess
import datetime
import pandas as pd
import random

CSV_PATH = "best_weight_summary.csv"

THRESHOLD = 9.12

def load_weight_sets_from_csv(path, threshold):
    df = pd.read_csv(path, index_col=None)
    df_filtered = df[df["AvgCompletedTasks"] >= threshold]
    subset = df_filtered.iloc[:, 0:7]
    return [tuple(row) for row in subset.values]

MRTA_CMD_TEMPLATE = "WEIGHT_TICK={tick} WEIGHT_DIST_OTHER={other} WEIGHT_DIST_SELF={self} WEIGHT_FRONTIER_CONFLICT={conflict} WEIGHT_UNKNOWN_COUNT={unknown} WEIGHT_EXHAUSTED_PERCENT={exhausted} UNKNOWN_SCOPE={scope} ./MRTA parse {seed} > '{log_path}'"


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
            task_lines = [line for line in task_lines if line.strip()]
            total_tasks = int(task_lines[1].split(":")[1].split(",")[0].strip())
            completed_tasks = int(task_lines[1].split(",")[-1].strip().split()[-1])
            active_tasks = int(task_lines[1].split(",")[2].strip().split()[-1])
            found_tasks = active_tasks + completed_tasks
        elif in_task_block:
            task_lines.append(l)

    return latest_observed_ratio, found_tasks, total_tasks, completed_tasks


def run_for_weight(weight_set, n, map_size):
    tick, other, self_w, conflict, unknown, exhausted, scope = weight_set
    total_observed = total_found = total_total = total_completed = 0
    valid_runs = 0

    start_time = time.time()
    
    for i in tqdm(range(n), desc=f"WEIGHT {weight_set}", leave=False):
        seed = random.randint(0, 2**31 - 1)  # 완전 랜덤 32비트 정수
        
        log_path = f"/tmp/MRTA_weight_{tick}_{other}_{self_w}_{conflict}_{unknown}_{exhausted}_{scope}_run{i}.log"
        cmd = MRTA_CMD_TEMPLATE.format(
            tick=tick, other=other, self=self_w,
            conflict=conflict, unknown=unknown, seed=seed, log_path=log_path, exhausted=exhausted, scope=scope
        )

        try:
            proc = subprocess.Popen(cmd, shell=True)
            proc.wait(timeout=60)  # 혹시라도 무한 대기 방지

            # 세그폴트 혹은 파일 생성 실패 체크
            if not os.path.exists(log_path):
                print(f"[⚠️] Log file missing for seed {i + 100}, weights {weight_set}")
                continue
            if "Algorithm time :" not in open(log_path).read():
                print(f"[⚠️] Incomplete output for seed {i + 100}, weights {weight_set}")
                continue

            observed, found, total, completed = parse_latest_metrics(log_path, map_size)
            total_observed += observed
            total_found += found
            total_total += total
            total_completed += completed
            valid_runs += 1

            # ✅ 로그 파일 삭제
            try:
                os.remove(log_path)
            except Exception as e:
                print(f"[⚠️] Failed to delete log file {log_path}: {e}")
                
        except Exception as e:
            print(f"[❌] Failed run {i} for weights {weight_set}: {e}")
            continue

    # 평균 계산 시 유효한 실행 수로 나누기
    if valid_runs == 0:
        return weight_set, 0.0, 0.0, 0.0

    avg_observed = total_observed / valid_runs
    avg_found = total_found / valid_runs
    avg_completed = total_completed / valid_runs

    end_time = time.time()
    elapsed = end_time - start_time
    print(f"\n⏱️  Weight {weight_set} 실험 시간: {elapsed:.2f}초")
    print(f"✅ Weight {weight_set} - 유효 실행 수: {valid_runs}/{n}")
    print(f"  평균 관측 비율: {avg_observed:.2f}%, "
          f"평균 발견된 작업: {avg_found:.2f}, "
          f"평균 완료된 작업: {avg_completed:.2f}")
    
    return weight_set, avg_observed, avg_found, avg_completed



def main(n=200, map_size=30, max_workers=12):
    start_global = time.time()
    
    # initial_weight = (2, 6, 9, 6, 3, 85, 1) # 초기 가중치 설정
    # weight_combinations = [initial_weight]
    
    weight_combinations = load_weight_sets_from_csv(CSV_PATH, THRESHOLD)
    
    results = []
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        future_to_weight = {
            executor.submit(run_for_weight, weights, n, map_size): weights
            for weights in weight_combinations
        }

        for future in tqdm(as_completed(future_to_weight), total=len(future_to_weight), desc="Overall Progress"):
            weight_set, avg_obs, avg_found, avg_comp = future.result()
            results.append((weight_set, avg_obs, avg_found, avg_comp))

    with open("best_weight_summary.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "WEIGHT_TICK", "WEIGHT_DIST_OTHER", "WEIGHT_DIST_SELF",
            "WEIGHT_FRONTIER_CONFLICT", "WEIGHT_UNKNOWN_COUNT",
            "AvgObserved(%)", "AvgFoundTasks", "AvgCompletedTasks"
        ])
        for weight_set, avg_obs, avg_found, avg_comp in results:
            writer.writerow([*weight_set, f"{avg_obs:.2f}", f"{avg_found:.2f}", f"{avg_comp:.2f}"])

    print("✅ 모든 결과가 best_weight_summary.csv에 저장되었습니다.")
    end_global = time.time()
    total_elapsed = end_global - start_global
    print(f"\n🕒 전체 실험 소요 시간: {total_elapsed:.2f}초")

if __name__ == "__main__":
    main(n=1000, map_size=30, max_workers=12)
