import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 파일 경로
file_path = 'summary_10000.csv'  # 현재 디렉토리에 있어야 함

# 데이터 로드
df = pd.read_csv(file_path)

# 통계 정보 계산
found_mean = df['AvgFoundTasks'].mean()
found_max = df['AvgFoundTasks'].max()
found_min = df['AvgFoundTasks'].min()
found_std = df['AvgFoundTasks'].std()

completed_mean = df['AvgCompletedTasks'].mean()
completed_max = df['AvgCompletedTasks'].max()
completed_min = df['AvgCompletedTasks'].min()
completed_std = df['AvgCompletedTasks'].std()

# 특정 임계값 이상인 샘플 개수 확인
n = 9.05  # 원하는 값으로 수정 가능
completed_count = (df['AvgCompletedTasks'] >= n).sum()
print(f'AvgCompletedTasks >= {n} 인 sample 개수: {completed_count}')

# AvgFoundTasks 그래프
plt.figure(figsize=(10, 6))
plt.bar(np.arange(len(df)), df['AvgFoundTasks'], width=0.6)
plt.axhline(found_mean, color='red', linestyle='--', label=f'Mean = {found_mean:.2f}')
plt.xlabel('Samples')
plt.ylabel('AvgFoundTasks')
plt.title('Distribution of AvgFoundTasks')
plt.text(0.5, found_max + 0.2,
         f'Max = {found_max:.2f}\nMin = {found_min:.2f}\nStd = {found_std:.2f}',
         fontsize=10, bbox=dict(facecolor='white', alpha=0.7))
plt.ylim(found_min - 1, found_max + 2)
plt.legend()
plt.tight_layout()
plt.show()

# AvgCompletedTasks 그래프
plt.figure(figsize=(10, 6))
plt.bar(np.arange(len(df)), df['AvgCompletedTasks'], width=0.6)
plt.axhline(completed_mean, color='red', linestyle='--', label=f'Mean = {completed_mean:.2f}')
plt.xlabel('Samples')
plt.ylabel('AvgCompletedTasks')
plt.title('Distribution of AvgCompletedTasks')
plt.text(0.5, completed_max + 0.2,
         f'Max = {completed_max:.2f}\nMin = {completed_min:.2f}\nStd = {completed_std:.2f}',
         fontsize=10, bbox=dict(facecolor='white', alpha=0.7))
plt.ylim(completed_min - 1, completed_max + 2)
plt.legend()
plt.tight_layout()
plt.show()
