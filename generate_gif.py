from PIL import Image
from moviepy.editor import VideoFileClip
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
    def __init__(self, filename, map_size, drones=[]):
        self.frames = []
        self.filename = filename
        self.map_size = map_size
        self.trace_map = [[0 for _ in range(map_size)] for _ in range(map_size)]
        self.drones_color = {}
        self.seed = -1
        self.zorder = {
            "WALL": 100,
            "OBSERVED": 100,
            "TRACE": 101,
            "ROBOT_LINE": 103,
            "TEXT": 104,
        }

        # 어두운 색상 리스트
        color_names = ["purple", "blue", "yellow"] #get_dark_colors()
        # color_names.remove("black")
        # color_names.remove("red")

        for t in drones:
            color = random.choice(color_names)
            self.drones_color[t] = color
            color_names.remove(color)

    def get_direction_type(self, last_pos, next_pos):
        # y, x
        if last_pos[0] == next_pos[0]:
            return "HOR"
        elif last_pos[1] == next_pos[1]:
            return "VER"
        else:
            return "ERR"

    def add_object_map(self, tick, map, observed_cnt, task_info, latest_robot_dstar, latest_drone_dstar):
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

        robots = {}
        
        # 격자 안에 문자 표시 (공백은 빈칸)
        for (i, j), obj in np.ndenumerate(state):
            obj_name = obj
            if len(obj_name) > 0 and obj_name[0] == "@":
                obj_name = obj_name[1:]
                
            r_name = re.match(r'[A-Za-z]{1,100}', obj_name)
            if r_name:
              r_name = r_name[0]
              if r_name not in ["T", "WAL"]:
                r_id = obj_name[len(r_name):]
                robots[int(r_id)] = [j,i]
            if obj_name in self.drones_color.keys():
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
                ax.add_patch(plt.Rectangle((j - 0.4, i - 0.4), 0.8, 0.8, color=self.drones_color[self.trace_map[j][i]], zorder=self.zorder["TRACE"]))

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

        ax.set_ylim(self.map_size - 0.5, -0.5)
        ax.set_xlim(-0.5, self.map_size - 0.5)


        ## Print robot path
        full_dstar = {}
        full_dstar.update(latest_robot_dstar)
        full_dstar.update(latest_drone_dstar)
        for r_id in full_dstar:
          if r_id in latest_robot_dstar:
              path = latest_robot_dstar[r_id]
              color = 'green'
          else:
              path = latest_drone_dstar[r_id]
              color = self.drones_color[f"RD{r_id}"]
              
          
          for (y1, x1), (y2, x2) in zip(path, path[1:]):
              real_y1 = self.map_size - y1 - 1
              real_y2 = self.map_size - y2 - 1

              if y1 == y2 or x1 == x2:
                ax.plot([x1, x2], [real_y1, real_y2], color=color, linewidth=1,  zorder=self.zorder["ROBOT_LINE"])  # straight
              else:        
                  ax.plot([x1, x2], [real_y1, real_y1], color=color, linewidth=1, zorder=self.zorder["ROBOT_LINE"])
                  ax.plot([x2, x2], [real_y1, real_y2], color=color, linewidth=1, zorder=self.zorder["ROBOT_LINE"])
              
        plt.tight_layout(pad=0)
        
        # Save as GIF and Video
        plt.subplots_adjust(bottom=0.3)
        if tick == 0:
          fig.text(0.5, 0.2, f"Tick: {tick}, SEED: {self.seed}", ha='center', fontsize=12)
        else:
          fig.text(0.5, 0.2, f"Tick: {tick}", ha='center', fontsize=12)
        fig.text(0.5, 0.15, f"Observed: {observed_cnt} / {real_width}x{real_width} (%.2f%%)"%((observed_cnt/real_width**2)*100), ha='center', fontsize=7)
        if "max" in latest_task_info:
          tasks = task_info['tasks']
          done_tasks = [t for t in tasks if tasks[t]['done']]
          fig.text(0.5, 0.1, f"Found tasks(MAX: {task_info['max']}): {task_info['found']} / {task_info['created']}", ha='center', fontsize=7)
          fig.text(0.5, 0.05, f"Finished {len(done_tasks)}: {','.join(done_tasks)}", ha='center', fontsize=7)
        fig.canvas.draw()
        img = np.array(fig.canvas.buffer_rgba())
        img = Image.fromarray(img)
        self.frames.append(img)

        plt.close(fig)
    
    def save(self):
        self.frames[0].save(self.filename, save_all=True, append_images=self.frames[1:], duration=230, loop=0)
        clip = VideoFileClip(self.filename)

        # Write to MP4 (H.264 codec, recommended for compatibility)
        clip.write_videofile(os.path.basename(self.filename)+".mp4", codec="libx264")


def parse_map(lines):
    map = []
    for l in lines:
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
        task_done = t_info.group(5) == "True"
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
    os.system(f"./MRTA parse | tee {MRTA_LOG_PATH} ")
    input("Enter after MRTA is finished")
    MAP_SIZE = 20
    map_gif = MAP_GIF("./rd0_230.gif", MAP_SIZE, drones=['RD0', 'RD3'])

    with open(MRTA_LOG_PATH, "r") as f:
        map_flag = False
        map_lines = []
        task_flag = False
        task_lines = []
        latest_task_info = {}
        latest_robot_dstar = {}
        robot_path_flag = False
        seed = -1

        latest_drone_frontier_path = {}
        drone_path_flag = False

        tick = 0
        for l in f.read().splitlines():        
            if l.startswith("SEED: "):
                seed = int(l[len("SEED: "):], 16)
                map_gif.seed = seed
            elif l.startswith("Start Task Info"):
                task_flag = True
            
            elif l.startswith("End Task Info"):
                latest_task_info = parse_task(task_lines)
                task_flag = False

            elif task_flag:
                task_lines.append(l)

            elif l.startswith("Start Object map: "):
                map_flag = True
                tick = int(l[len("Start Object map: "):])

            elif l.startswith("End Object map"):
                observed_cnt = int(l[len("End Object map: "):])
                map_flag = False
                map_gif.add_object_map(tick, parse_map(map_lines), observed_cnt, latest_task_info, latest_robot_dstar, latest_drone_frontier_path)
                
                # if(len(latest_robot_dstar) > 0):
                #     map_gif.save()
                #     exit(-1)
                
                map_lines = []
                latest_robot_dstar = {}
                task_lines = []
                latest_drone_frontier_path = {}
              
            elif l.startswith("Robot Path Info"):
                robot_path_flag = True

            elif l.startswith("End Path Info"):
                robot_path_flag = False
              
            elif robot_path_flag:
                r_id, path = l.split(":")
                path = [list(map(int, c.strip()[1:-1].split(","))) for c in path.strip().split("|")[:-1]]
                if len(path) > 0:
                  latest_robot_dstar[int(r_id)] = path
            
            elif l.startswith("Drone Path End"):
                drone_path_flag = False
            elif l.startswith("Drone Path Info"):
                drone_path_flag = True
            elif drone_path_flag:
                r_id, path = l.split(":")
                path = [list(map(int, c.strip()[1:-1].split(","))) for c in path.strip().split("|")[:-1]]
                if len(path) > 0:
                  latest_drone_frontier_path[int(r_id)] = path
                

            elif map_flag:
                if len(l) > 0 and not (l.startswith("0-") or l.startswith("  ")):
                    map_lines.append(l)
                

    map_gif.save()
    #print(proc.stdout.readline())



