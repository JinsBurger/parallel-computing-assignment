import subprocess


if __name__ == '__main__':
    proc = subprocess.Popen("./MRTA", shell=True, stdout=subprocess.PIPE)
    print(proc.stdout.readline())


import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

# 예시 문자열 배열 (시간에 따른 변화)
array_states = [
    np.array([['A', 'B', 'C', 'D'],
              ['E', 'F', 'G', 'H'],
              ['J', 'I', 'K', 'L']]),
    
    np.array([['A', 'B', ' ', 'D'],
              ['E', ' ', 'G', 'H'],
              ['J', 'I', 'K', 'L']]),
    
    np.array([[' ', 'B', ' ', ' '],
              ['E', ' ', ' ', 'H'],
              [' ', 'I', ' ', 'L']]),
]
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

# 문자열 배열 상태 변화 (시간 순서)
array_states = [
    np.array([['A', 'B', 'C', 'D'],
              ['E', 'F', 'G', 'H'],
              ['J', 'I', 'K', 'L']]),
    
    np.array([['A', 'B', ' ', 'D'],
              ['E', ' ', 'G', 'H'],
              ['J', 'I', 'K', 'L']]),
    
    np.array([[' ', 'B', ' ', ' '],
              ['E', ' ', ' ', 'H'],
              [' ', 'I', ' ', 'L']]),
]

# 이미지 프레임 저장 리스트
frames = []

# 각 상태를 이미지로 그림
for state in array_states:
    fig, ax = plt.subplots(figsize=(4, 3), dpi=100)
    
    # 바둑판 격자 테두리만 그리기 (패턴용)
    ax.set_xticks(np.arange(-0.5, state.shape[1], 1))
    ax.set_yticks(np.arange(-0.5, state.shape[0], 1))
    ax.grid(color='black', linestyle='-', linewidth=1)
    
    # 눈금 없애기
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.tick_params(left=False, bottom=False)
    
    # 격자 안에 문자 표시 (공백은 빈칸)
    for (i, j), val in np.ndenumerate(state):
        if val != ' ':
            ax.text(j, i, val, ha='center', va='center', fontsize=20, family='monospace')
    
    plt.tight_layout(pad=0)
    
    # 이미지로 저장
    fig.canvas.draw()
    img = np.array(fig.canvas.buffer_rgba())
    img = Image.fromarray(img)
    frames.append(img)
    
    plt.close(fig)

# GIF로 저장
frames[0].save('grid_text_animation.gif', save_all=True, append_images=frames[1:], duration=500, loop=0)
