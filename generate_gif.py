from PIL import Image
import subprocess
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import os
import re
import random


MRTA_LOG_PATH = "/tmp/MRTA.log"


'''
  ---------------------------------------------------------------------------------
19|   |   |WAL|WAL|   |   |   |   |   |   |   |   |   |   |   |WAL|   |   |   |   |
  ---------------------------------------------------------------------------------
18|   |WAL|   |WAL|WAL|   |T15|WAL|   |   |   |   |   |   |   |   |   |   |RC4|   |
  ---------------------------------------------------------------------------------
17|   |   |   |   |   |   |WAL|   |   |T06|   |WAL|   |   |   |   |   |T11|   |T05|
  ---------------------------------------------------------------------------------
16|   |   |   |WAL|T01|   |   |   |   |   |   |   |   |   |   |WAL|   |   |   |   |
  ---------------------------------------------------------------------------------
15|   |   |   |   |   |WAL|WAL|   |   |   |   |   |   |WAL|   |WAL|T08|   |   |   |
  ---------------------------------------------------------------------------------
14|WAL|   |WAL|   |   |   |   |WAL|   |WAL|WAL|   |   |   |   |   |WAL|   |   |T07|
  ---------------------------------------------------------------------------------
13|WAL|   |WAL|WAL|WAL|   |   |   |   |   |   |   |   |   |   |WAL|   |WAL|   |WAL|
  ---------------------------------------------------------------------------------
12|WAL|   |   |   |WAL|   |   |   |   |RW5|WAL|WAL|   |   |   |   |   |   |   |WAL|
  ---------------------------------------------------------------------------------
11|   |   |   |   |RD3|   |WAL|   |   |   |WAL|   |   |   |RD0|WAL|   |   |   |   |
  ---------------------------------------------------------------------------------
10|   |   |   |WAL|   |   |   |   |WAL|   |   |T03|   |   |   |   |   |WAL|   |   |
  ---------------------------------------------------------------------------------
 9|   |   |   |WAL|   |   |   |   |   |   |   |   |   |WAL|   |   |   |WAL|   |WAL|
  ---------------------------------------------------------------------------------
 8|   |WAL|   |   |WAL|   |   |WAL|   |WAL|WAL|   |   |   |   |   |   |   |WAL|RC1|
  ---------------------------------------------------------------------------------
 7|WAL|   |T12|   |WAL|WAL|   |   |WAL|   |WAL|   |   |   |   |   |   |   |   |WAL|
  ---------------------------------------------------------------------------------
 6|   |   |   |   |   |T00|   |   |   |T13|T14|   |WAL|   |T02|   |WAL|   |   |   |
  ---------------------------------------------------------------------------------
 5|   |   |   |   |   |WAL|   |WAL|   |T04|   |   |   |   |   |   |   |WAL|   |   |
  ---------------------------------------------------------------------------------
 4|T10|   |   |   |   |   |   |   |WAL|   |   |   |   |   |   |WAL|WAL|   |   |   |
  ---------------------------------------------------------------------------------
 3|   |WAL|   |   |WAL|WAL|   |   |WAL|   |WAL|WAL|   |   |   |   |   |   |   |   |
  ---------------------------------------------------------------------------------
 2|   |   |WAL|   |   |   |   |WAL|   |   |   |   |   |WAL|   |   |   |   |   |   |
  ---------------------------------------------------------------------------------
 1|   |   |WAL|   |   |   |   |   |WAL|   |   |   |T09|   |   |WAL|   |WAL|   |   |
  ---------------------------------------------------------------------------------
 0|   |WAL|   |RW2|   |   |   |   |WAL|   |   |   |   |   |WAL|   |   |   |   |WAL|
  ---------------------------------------------------------------------------------
     0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19
'''


# CSS4_COLORS 중에서 밝기(luminance)가 낮은 색상만 추출
def get_dark_colors(threshold=0.6):
    dark_colors = []
    for name, hexval in mcolors.CSS4_COLORS.items():
        rgb = mcolors.to_rgb(hexval)
        # 밝기 계산 (perceived luminance formula)
        luminance = 0.2126 * rgb[0] + 0.7152 * rgb[1] + 0.0722 * rgb[2]
        if luminance < threshold:
            dark_colors.append(name)
    return dark_colors



class MAP_GIF:
    def __init__(self, filename, map_size, traces=[]):
        self.frames = []
        self.filename = filename
        self.map_size = map_size
        self.trace_map = [[0 for _ in range(map_size)] for _ in range(map_size)]
        self.traces = {}
        self.zorder = {
            "WALL": 100,
            "OBSERVED": 100,
            "TRACE": 101,
            "TEXT": 102
        }

        # 어두운 색상 리스트
        color_names = get_dark_colors()
        color_names.remove("black")
        color_names.remove("red")

        for t in traces:
            color = random.choice(color_names)
            self.traces[t] = color
            color_names.remove(color)

    def add_object_map(self, tick, map, observed_cnt, task_info):
        print(f"TICK: {tick}")
      

        state = np.array(map)
        # 각 상태를 이미지로 그림
        fig, ax = plt.subplots(figsize=(4, 3), dpi=400)
        real_width = len(map)
        
        # 바둑판 격자 테두리만 그리기 (패턴용)
        ax.set_xticks(np.arange(-0.5, state.shape[1], 1))
        ax.set_yticks(np.arange(-0.5, state.shape[0], 1))
        ax.grid(color='black', linestyle='-', linewidth=1)
        
        # 눈금 없애기
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.tick_params(left=False, bottom=False)
        
        # 격자 안에 문자 표시 (공백은 빈칸)
        for (i, j), obj in np.ndenumerate(state):
            obj_name = obj
            if len(obj_name) > 0 and obj_name[0] == "@":
                obj_name = obj_name[1:]

            if obj_name in self.traces.keys():
                self.trace_map[j][i] = obj_name

            if obj_name == "WAL":
                if obj[0] == '@':
                  ax.add_patch(plt.Rectangle((j - 0.5, i - 0.5), 1, 1, color='black', zorder=self.zorder["WALL"]))
                else:
                  #Not observed
                  ax.add_patch(plt.Rectangle((j - 0.5, i - 0.5), 1, 1, color='black', alpha=0.3, zorder=self.zorder["WALL"]))
                  ax.text(j, i, obj_name, ha='center', va='center', fontsize=3, family='monospace', zorder=self.zorder["TEXT"])
            else:
              if self.trace_map[j][i] != 0:
                ax.add_patch(plt.Rectangle((j - 0.4, i - 0.4), 0.8, 0.8, color=self.traces[self.trace_map[j][i]], zorder=self.zorder["TRACE"]))

              # Observed task
              if obj != '':
                if obj[0] == '@':
                    '''
                    Found task names start with `@!Txx`  (e.g., @!T05)
                    Unfound task names start with `@Txx` (e.g., @T05)
                    '''
                    if len(obj_name) > 0 and obj_name.startswith('!T') and obj_name[2:].isnumeric():
                        # 발견된 Task → 주황색
                        ax.add_patch(plt.Rectangle((j - 0.5, i - 0.5), 1, 1, color='orange', alpha=0.4, zorder=self.zorder["OBSERVED"]))
                    elif len(obj_name) > 0 and obj_name.startswith('T') and obj_name[1:].isnumeric():
                        task_id = obj_name
                        if task_id in task_info["tasks"]:
                            if not task_info["tasks"][task_id]["found"]:
                                # 초기 생성 Task지만 아직 발견되지 않은 경우 → 연두색
                                ax.add_patch(plt.Rectangle((j - 0.5, i - 0.5), 1, 1, color='greenyellow', alpha=0.3, zorder=self.zorder["OBSERVED"]))
                        else:
                            # task_info에 등록되지 않은 → 최근 생성된 Task (아직 발견되지 않음) → 파란색
                            ax.add_patch(plt.Rectangle((j - 0.5, i - 0.5), 1, 1, color='blue', alpha=0.3, zorder=self.zorder["OBSERVED"]))
                    else:
                        # 기타 객체 → 빨간색
                        ax.add_patch(plt.Rectangle((j - 0.5, i - 0.5), 1, 1, color='red', alpha=0.3, zorder=self.zorder["OBSERVED"]))
                ax.text(j, i, obj_name, ha='center', va='center', fontsize=3, family='monospace', zorder=self.zorder["TEXT"])


            
            
        
        plt.tight_layout(pad=0)
        
        # 이미지로 저장
        plt.subplots_adjust(bottom=0.3)
        fig.text(0.5, 0.2, f"Tick: {tick}", ha='center', fontsize=12)
        fig.text(0.5, 0.15, f"Observed: {observed_cnt} / {real_width}x{real_width} (%.2f%%)"%((observed_cnt/real_width**2)*100), ha='center', fontsize=7)
        if "max" in latest_task_info:
          tasks = task_info['tasks']
          fig.text(0.5, 0.1, f"Found tasks(MAX: {task_info['max']}): {task_info['found']} / {task_info['created']}", ha='center', fontsize=7)
          fig.text(0.5, 0.05, f"Not Found: {','.join([t for t in tasks if not tasks[t]['found']])}", ha='center', fontsize=7)
        fig.canvas.draw()
        img = np.array(fig.canvas.buffer_rgba())
        img = Image.fromarray(img)
        self.frames.append(img)

        plt.close(fig)
    
    def save(self):
        self.frames[0].save(self.filename, save_all=True, append_images=self.frames[1:], duration=230, loop=0)

def parse_map(lines):
    map = []
    for l in lines[::-1]:
        row = []
        for e in l.split("|")[1:-1]:
            row.append(e.strip())
        map.append(row)
    return map
    
def parse_task(lines):
    max_task, created_task, active_task, completed_task = [int(k.split(":")[1].strip()) for k in lines[1].split(",")]
    found_taskN = 0
    tasks = {}
    for line in lines[3:-1]:
        t_info = re.match(r'\s*(\d+)\s*\(\s*(\d+)\s*,\s*(\d+)\s*\)\s*(True|False)\s*(True|False)\s*(\d|No)\s*(\d+)\s*(\d+)', line)
        task_id = "T%02d"%int(t_info.group(1))
        task_x, task_y = t_info.group(2), t_info.group(3)
        task_found = t_info.group(4) == "True"
        task_done = t_info.group(5)
        task_assigned = t_info.group(6)
        tasks[task_id] ={
            "found": task_found,
            "done": task_done,
            "assigned": task_assigned
        }

        if task_found == True:
          found_taskN += 1

    task_info = {
        "max": max_task,
        "created": created_task,
        "active": active_task,
        "completed": completed_task,
        "found": found_taskN,
        "tasks": tasks
    }

    return task_info

if __name__ == '__main__':
    os.system(f"./MRTA parse > {MRTA_LOG_PATH}")

    map_gif = MAP_GIF("./rd0_230.gif", 30, traces=['RD0', 'RD3'])
    with open(MRTA_LOG_PATH, "r") as f:
        map_flag = False
        map_lines = []
        task_flag = False
        task_lines = []
        latest_task_info = {}

        tick = 0
        for l in f.read().splitlines():        
            if l.startswith("Start Task Info"):
                task_flag = True
            
            elif l.startswith("End Task Info"):
                latest_task_info = parse_task(task_lines)
                task_flag = False
                task_lines = []

            elif task_flag:
                task_lines.append(l)

            elif l.startswith("Start Object map: "):
                map_flag = True
                tick = int(l[len("Start Object map: "):])

            elif l.startswith("End Object map"):
                observed_cnt = int(l[len("End Object map: "):])
                map_flag = False
                map_gif.add_object_map(tick, parse_map(map_lines), observed_cnt, latest_task_info)
                # map_gif.save()
                # break
                map_lines = []

            elif map_flag:
                if len(l) > 0 and not (l.startswith("0-") or l.startswith("  ")):
                    map_lines.append(l)
                

    map_gif.save()
    #print(proc.stdout.readline())



