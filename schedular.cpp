#include "schedular.h"
#include <stack>
#include <random>
#include <map>

void Scheduler::on_info_updated(const set<Coord> &observed_coords,
                                const set<Coord> &updated_coords,
                                const vector<vector<vector<int>>> &known_cost_map,
                                const vector<vector<OBJECT>> &known_object_map,
                                const vector<shared_ptr<TASK>> &active_tasks,
                                const vector<shared_ptr<ROBOT>> &robots)
{
}

bool Scheduler::on_task_reached(const set<Coord> &observed_coords,
                                const set<Coord> &updated_coords,
                                const vector<vector<vector<int>>> &known_cost_map,
                                const vector<vector<OBJECT>> &known_object_map,
                                const vector<shared_ptr<TASK>> &active_tasks,
                                const vector<shared_ptr<ROBOT>> &robots,
                                const ROBOT &robot,
                                const TASK &task)
{
    return robot.type != ROBOT::TYPE::DRONE;
}


// 방향 벡터: 상, 하, 좌, 우
static const Coord directions[4] = {
    {0, 1},  // UP
    {0, -1}, // DOWN
    {-1, 0}, // LEFT
    {1, 0}   // RIGHT
};

// 반대 방향 인덱스
static const int opposite[4] = {1, 0, 3, 2};

static std::map<int, std::stack<Coord>> drone_stack;
static std::map<int, std::vector<std::vector<bool>>> visited;
static std::map<int, bool> initialized;
static std::map<int, ROBOT::ACTION> initial_direction;

ROBOT::ACTION Scheduler::idle_action(const set<Coord> &observed_coords,
                                     const set<Coord> &updated_coords,
                                     const vector<vector<vector<int>>> &known_cost_map,
                                     const vector<vector<OBJECT>> &known_object_map,
                                     const vector<shared_ptr<TASK>> &active_tasks,
                                     const vector<shared_ptr<ROBOT>> &robots,
                                     const ROBOT &robot)
{
    if (robot.type != ROBOT::TYPE::DRONE)
        return static_cast<ROBOT::ACTION>(rand() % 5);

    int map_size = static_cast<int>(known_object_map.size());
    int id = robot.id;
    Coord curr = robot.get_coord();

    // 초기 이동 방향 설정 (맵을 8구역으로 나누고 중앙으로 향함)
    if (!initialized[id]) {
        initialized[id] = true;
        int cx = curr.x, cy = curr.y;
        int mid = map_size / 2;

        if (cx < mid && cy < mid) // 좌하
            initial_direction[id] = ROBOT::ACTION::UP;
        else if (cx >= mid && cy < mid) // 우하
            initial_direction[id] = ROBOT::ACTION::LEFT;
        else if (cx < mid && cy >= mid) // 좌상
            initial_direction[id] = ROBOT::ACTION::RIGHT;
        else // 우상
            initial_direction[id] = ROBOT::ACTION::DOWN;

        cout << "[Drone " << id << "] initial position: " << curr << ", direction set to " << initial_direction[id] << endl;
        return initial_direction[id];
    }

    // visited 초기화
    if (visited.count(id) == 0)
    {
        visited[id] = vector<vector<bool>>(map_size, vector<bool>(map_size, false));
        drone_stack[id].push(curr);
        cout << "[Drone " << id << "] DFS initialized at " << curr << endl;
    }

    visited[id][curr.x][curr.y] = true;

    // dfs에서 다음 갈 수 있는 방향 후보
    vector<int> candidate_dir;
    vector<int> unexplored_count(4, 0);

    for (int dir = 0; dir < 4; ++dir)
    {
        Coord next = curr + directions[dir];
        //cout << "[Drone " << id << "] Checking direction " << dir << " → target: " << next;

        if (next.x < 0 || next.x >= map_size || next.y < 0 || next.y >= map_size)
        {
            //cout << " → out of bounds." << endl;
            continue;
        }
        if (known_object_map[next.x][next.y] == OBJECT::WALL)
        {
            //cout << " → wall encountered." << endl;
            continue;
        }
        if (!visited[id][next.x][next.y])
        {
            //cout << " → candidate for 5x1 check." << endl;
            int cnt = 0;
            for (int i = -2; i <= 2; ++i)
            {
                Coord check;
                if (dir == 0 || dir == 1)
                    check = {curr.x + i, curr.y + (dir == 0 ? 3 : -3)};
                else
                    check = {curr.x + (dir == 2 ? -3 : 3), curr.y + i};

                if (check.x < 0 || check.x >= map_size || check.y < 0 || check.y >= map_size)
                    continue;

                if (known_object_map[check.x][check.y] == OBJECT::UNKNOWN)
                {
                    //cout << "    [5x1] Unexplored at " << check << endl;
                    cnt++;
                }
            }

            if (cnt > 0)
            {
                candidate_dir.push_back(dir);
                unexplored_count[dir] = cnt;
                //cout << "[Drone " << id << "] Direction " << dir << " has " << cnt << " unexplored tiles" << endl;
            }
            else
            {
                //cout << "[Drone " << id << "] Direction " << dir << " fully explored." << endl;
            }
        }
        else
        {
            //cout << " → already visited." << endl;
        }
    }

    if (!candidate_dir.empty())
    {
        int best_dir = candidate_dir[0];
        for (int dir : candidate_dir)
            if (unexplored_count[dir] > unexplored_count[best_dir])
                best_dir = dir;

        vector<int> best_candidates;
        for (int dir : candidate_dir)
            if (unexplored_count[dir] == unexplored_count[best_dir])
                best_candidates.push_back(dir);

        if (best_candidates.size() > 1)
            best_dir = best_candidates[rand() % best_candidates.size()];

        Coord next = curr + directions[best_dir];
        drone_stack[id].push(curr);
        cout << "[Drone " << id << "] Move from " << curr << " to " << next << " (dir=" << best_dir << ")" << endl;
        return static_cast<ROBOT::ACTION>(best_dir);
    }
    else
    {
        if (!drone_stack[id].empty())
        {
            Coord parent = drone_stack[id].top();
            drone_stack[id].pop();

            for (int dir = 0; dir < 4; ++dir)
            {
                if (curr + directions[dir] == parent)
                {
                    cout << "[Drone " << id << "] Backtrack from " << curr << " to " << parent << " (dir=" << dir << ")" << endl;
                    return static_cast<ROBOT::ACTION>(dir);
                }
            }
        }
        return ROBOT::ACTION::HOLD;
    }
}

