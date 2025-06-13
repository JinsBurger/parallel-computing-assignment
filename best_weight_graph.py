import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from matplotlib.patches import Patch

# ① 사용할 한글 폰트 지정 (시스템에 설치된 것으로 대체)
plt.rc('font', family='AppleGothic')  # Mac이면 'AppleGothic', Windows는 'Malgun Gothic'
plt.rc('axes', unicode_minus=False)

# ② 데이터 로드
df = pd.read_excel("best_weight_summary.xlsx")

# ③ 파이차트 생성 함수
def create_pie_chart(values, bins, labels, title, output_file):
    cats = pd.cut(values, bins=bins, labels=labels)
    counts = cats.value_counts().sort_index()
    avg, std = values.mean(), values.std()
    min_val, max_val = values.min(), values.max()

    fig, ax = plt.subplots(figsize=(12, 10))
    wedges, texts, autotexts = ax.pie(counts, labels=labels, autopct='%1.1f%%', startangle=140)
    ax.set_title(title)

    legend_elements = [
        Patch(facecolor=wedges[i].get_facecolor(), label=f'{labels[i]}: {counts[i]}개')
        for i in range(len(labels))
    ]
    ax.legend(handles=legend_elements, loc='lower center', bbox_to_anchor=(0.5, -0.25),
              ncol=2, fontsize=11, frameon=False)

    stat_text = f'Max: {max_val:.2f}   Min: {min_val:.2f}   Mean: {avg:.2f}   Std: {std:.2f}'
    plt.text(0, -1.45, stat_text, fontsize=12, ha='center')

    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"✅ {output_file} 저장 완료.")

# ④ AvgCompletedTasks 분석
completed_values = df["AvgCompletedTasks"]
completed_bins = [completed_values.min()-1, 8.79, 8.87, 8.95, 9.03, 9.11, completed_values.max()+1]
completed_labels = ['~8.79', '8.80~8.87', '8.88~8.95', '8.96~9.03', '9.04~9.11', '9.12~']
create_pie_chart(completed_values, completed_bins, completed_labels,
                 'AvgCompletedTasks 분포 (가중치 set - 3240개)', 'output_completedtasks.png')

# ⑤ AvgFoundTasks 분석
found_values = df["AvgFoundTasks"]
found_bins = [found_values.min()-1, 9.59, 9.69, 9.79, 9.89, 9.99, found_values.max()+1]
found_labels = ['~9.59', '9.60~9.69', '9.70~9.79', '9.80~9.89', '9.90~9.99', '10.00~']
create_pie_chart(found_values, found_bins, found_labels,
                 'AvgFoundTasks 분포 (가중치 set - 3240개)', 'output_foundtasks.png')
