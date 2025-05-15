#include "schedular.h"
#include <stack>
#include <random>
#include <map>

// 매크로 선언 (필요 시 컴파일 시 -Dgravity_mode 추가)
//#define gravity_mode

static int global_tick = 0;
static std::map<int, std::vector<std::vector<int>>> last_seen_time;

void Scheduler::on_info_updated(const set<Coord> &observed_coords,
                                const set<Coord> &updated_coords,
                                const vector<vector<vector<int>>> &known_cost_map,
                                const vector<vector<OBJECT>> &known_object_map,
                                const vector<shared_ptr<TASK>> &active_tasks,
                                const vector<shared_ptr<ROBOT>> &robots)
{
    global_tick++;
    int map_size = static_cast<int>(known_object_map.size());
    for (int id = 0; id < static_cast<int>(robots.size()); ++id) {
        if (last_seen_time.count(id) == 0) {
            last_seen_time[id] = std::vector<std::vector<int>>(map_size, std::vector<int>(map_size, -1));
        }
        for (const auto& coord : updated_coords) {
            if (coord.x >= 0 && coord.x < map_size && coord.y >= 0 && coord.y < map_size) {
                last_seen_time[id][coord.x][coord.y] = global_tick;
            }
        }
    }
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

    if (!initialized[id]) {
        initialized[id] = true;
        int cx = curr.x, cy = curr.y;
        int mid = map_size / 2;

        if (cx < mid && cy < mid)
            initial_direction[id] = ROBOT::ACTION::UP;
        else if (cx >= mid && cy < mid)
            initial_direction[id] = ROBOT::ACTION::LEFT;
        else if (cx < mid && cy >= mid)
            initial_direction[id] = ROBOT::ACTION::RIGHT;
        else
            initial_direction[id] = ROBOT::ACTION::DOWN;

        cout << "[Drone " << id << "] initial position: " << curr << ", direction set to " << initial_direction[id] << endl;
        return initial_direction[id];
    }

    if (visited.count(id) == 0)
    {
        visited[id] = std::vector<std::vector<bool>>(map_size, std::vector<bool>(map_size, false));
        drone_stack[id].push(curr);
        cout << "[Drone " << id << "] DFS initialized at " << curr << endl;
    }

    visited[id][curr.x][curr.y] = true;

    std::vector<int> candidate_dir;
    std::vector<int> unexplored_count(4, 0);

    for (int dir = 0; dir < 4; ++dir)
    {
        Coord next = curr + directions[dir];
        if (next.x < 0 || next.x >= map_size || next.y < 0 || next.y >= map_size)
            continue;
        if (known_object_map[next.x][next.y] == OBJECT::WALL)
            continue;
        if (!visited[id][next.x][next.y])
        {
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
                    cnt++;
            }

            if (cnt > 0)
            {
                candidate_dir.push_back(dir);
                unexplored_count[dir] = cnt;
            }
        }
    }

    if (!candidate_dir.empty())
    {
        int best_dir = candidate_dir[0];
        for (int dir : candidate_dir)
            if (unexplored_count[dir] > unexplored_count[best_dir])
                best_dir = dir;

        std::vector<int> best_candidates;
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
#ifdef gravity_mode
        std::array<int, 4> direction_weights = {0, 0, 0, 0};
        for (int x = 0; x < map_size; ++x) {
            for (int y = 0; y < map_size; ++y) {
                if (known_object_map[x][y] == OBJECT::WALL) continue;
                int weight = (last_seen_time[id][x][y] == -1) ? global_tick : (global_tick - last_seen_time[id][x][y]);
                if (y > x && y > -x + 2 * curr.x) direction_weights[0] += weight; // UP
                else if (y < x && y < -x + 2 * curr.x) direction_weights[1] += weight; // DOWN
                else if (y < x && y > -x + 2 * curr.x) direction_weights[2] += weight; // LEFT
                else direction_weights[3] += weight; // RIGHT
            }
        }
        int max_dir = 0;
        for (int i = 1; i < 4; ++i) {
            if (direction_weights[i] > direction_weights[max_dir])
                max_dir = i;
        }
        Coord next = curr + directions[max_dir];
        cout << "[Drone " << id << "] DFS complete, gravity mode → directional weight max at " << max_dir << " → move " << max_dir << endl;
        return static_cast<ROBOT::ACTION>(max_dir);
#else
        int rand_dir = rand() % 4;
        Coord next = curr + directions[rand_dir];
        cout << "[Drone " << id << "] DFS complete, fallback random move → dir " << rand_dir << endl;
        return static_cast<ROBOT::ACTION>(rand_dir);
#endif
    }
}
