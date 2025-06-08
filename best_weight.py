import os
import re
import csv
import time
from tqdm import tqdm
from concurrent.futures import ThreadPoolExecutor, as_completed
import subprocess
import datetime

test_seed = list(range(0, 10000))

MRTA_CMD_TEMPLATE = "WEIGHT_TICK={tick} WEIGHT_DIST_OTHER={other} WEIGHT_DIST_SELF={self} WEIGHT_DIST_ROBOT={robot} WEIGHT_FRONTIER_CONFLICT={conflict} WEIGHT_UNKNOWN_COUNT={unknown} WEIGHT_EXHAUSTED_PERCENT={exhausted} UNKNOWN_SCOPE={scope} ./MRTA parse {seed} > '{log_path}'"


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
    tick, other, self_w, robot, conflict, unknown, exhausted, scope = weight_set
    total_observed = total_found = total_total = total_completed = 0
    valid_runs = 0

    start_time = time.time()
    
    for i in tqdm(range(n), desc=f"WEIGHT {weight_set}", leave=False):
        log_path = f"/tmp/MRTA_weight_{tick}_{other}_{self_w}_{robot}_{conflict}_{unknown}_{exhausted}_{scope}_run{i}.log"
        cmd = MRTA_CMD_TEMPLATE.format(
            tick=tick, other=other, self=self_w, robot=robot,
            conflict=conflict, unknown=unknown, seed=i+100, log_path=log_path, exhausted=exhausted, scope=scope
        )

        try:
            proc = subprocess.Popen(cmd, shell=True)
            proc.wait(timeout=60)  # 혹시라도 무한 대기 방지

            # 세그폴트 혹은 파일 생성 실패 체크
            if not os.path.exists(log_path):
                print(f"[⚠️] Log file missing for seed {i}, weights {weight_set}")
                continue
            if "Algorithm time :" not in open(log_path).read():
                print(f"[⚠️] Incomplete output for seed {i}, weights {weight_set}")
                continue

            observed, found, total, completed = parse_latest_metrics(log_path, map_size)
            total_observed += observed
            total_found += found
            total_total += total
            total_completed += completed
            valid_runs += 1

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



def main(n=100, map_size=20, max_workers=4):
    start_global = time.time()
    
    initial_weight = (2, 6, 9, 3, 6, 3, 85, 1) # 초기 가중치 설정
    weight_combinations = [initial_weight]
    ranges = {
        0: range(0, 3), # tick weight
        1: range(0, 3), # other weight
        2: range(0, 4), # self weight
        3: range(0, 1), # robot weight
        4: range(0, 3), # conflict weight
        5: range(0, 2), # unknown count
        6: range(0, 13, 3), # exhausted weight
        7: range(0, 3) # scope weight
    }

    for idx in range(8):
        new_combinations = []
        for weight_set in weight_combinations:
            for delta in ranges[idx]:
                new_weight = list(weight_set)
                new_weight[idx] += delta
                new_combinations.append(tuple(new_weight))
        weight_combinations = new_combinations

    print(f"총 {len(weight_combinations)}개의 가중치 조합이 생성되었습니다.")
    print(weight_combinations)
    
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
            "WEIGHT_TICK", "WEIGHT_DIST_OTHER", "WEIGHT_DIST_SELF", "WEIGHT_DIST_ROBOT",
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
    main(n=100, map_size=20, max_workers=12)
