#include "schedular.h"
#include <stack>
#include <random>
#include <map>
#include <vector>
#include <queue>
#include <limits>


using namespace std;

// 매크로 선언 (필요 시 컴파일 시 -Dgravity_mode 추가)
//#define gravity_mode

static int global_tick = 0;
static map<int, vector<vector<int>>> last_seen_time;

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
            last_seen_time[id] = vector<vector<int>>(map_size, vector<int>(map_size, -1));
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

static map<int, stack<Coord>> drone_stack;
static map<int, vector<vector<bool>>> visited;
static map<int, bool> initialized;
static map<int, ROBOT::ACTION> initial_direction;

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
        visited[id] = vector<vector<bool>>(map_size, vector<bool>(map_size, false));
        drone_stack[id].push(curr);
        cout << "[Drone " << id << "] DFS initialized at " << curr << endl;
    }

    visited[id][curr.x][curr.y] = true;

    vector<int> candidate_dir;
    vector<int> unexplored_count(4, 0);

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
#ifdef gravity_mode
        array<int, 4> direction_weights = {0, 0, 0, 0};
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




/* 
  ***************************************
  ************* D STAR LITE *************
  ***************************************
*/

class DstarLite {
    public:
    void update_map() {

    }
    
    private:


};






/* 
  ***************************************
  ***************** MCMF ****************
  ***************************************
*/
//--------------------------------------------
// 1.  최소비용 최대유량 (MCMF)  ‑‑  간단한 SPFA‑based 구현
//--------------------------------------------
struct Edge {
    int to, rev;   // rev: 역간선 인덱스
    int cap;       // 잔여 용량
    int cost;      // 단위 유량 당 비용 (+,‑ 모두 허용)
};

class MCMF {
    int N;                                       // 정점 수
    vector<vector<Edge>> G;            // 인접 리스트
public:
    explicit MCMF(int n) : N(n), G(n) {}

    // 정방향용 cap, 역방향용 cap=0 / cost=-cost 생성
    void addEdge(int s, int t, int cap, int cost) {
        Edge fwd{t, (int)G[t].size(), cap,  cost};
        Edge rev{s, (int)G[s].size(), 0,  -cost};
        G[s].push_back(fwd);
        G[t].push_back(rev);
    }

    // (flow, cost) 반환. 음수 cost 간선 허용.
    pair<int,int> minCostMaxFlow(int s, int t) {
        const int INF = numeric_limits<int>::max()/4;
        int flow = 0, cost = 0;
        vector<int> dist(N), pvN(N), pvE(N);

        while (true) {
            fill(dist.begin(), dist.end(), INF);
            dist[s] = 0;
            vector<bool> inQ(N, false);
            queue<int> q; q.push(s); inQ[s] = true;

            // SPFA
            while (!q.empty()) {
                int v = q.front(); q.pop(); inQ[v] = false;
                for (int i = 0; i < (int)G[v].size(); ++i) {
                    const Edge &e = G[v][i];
                    if (e.cap && dist[e.to] > dist[v] + e.cost) {
                        dist[e.to] = dist[v] + e.cost;
                        pvN[e.to]  = v;
                        pvE[e.to]  = i;
                        if (!inQ[e.to]) { q.push(e.to); inQ[e.to] = true; }
                    }
                }
            }
            if (dist[t] == INF) break;          // 더 이상 증가 경로 없음

            // 증대량 찾기
            int aug = INF;
            for (int v = t; v != s; v = pvN[v])
                aug = min(aug, G[pvN[v]][pvE[v]].cap);

            // 잔여 용량/비용 갱신
            flow += aug;
            cost += aug * dist[t];
            for (int v = t; v != s; v = pvN[v]) {
                Edge &e = G[pvN[v]][pvE[v]];
                e.cap -= aug;
                G[v][e.rev].cap += aug;
            }
        }
        return {flow, cost};
    }

    const vector<vector<Edge>>& graph() const { return G; }
};

//--------------------------------------------
// 2.  assign_tasks_mcmf()
//--------------------------------------------
/**
 * distRT[r][t] : 로봇 r  ↔ task t  거리 (R×T)
 * distTT[i][j] : task i  ↔ task j  거리 (T×T)
 *
 * 로봇 R 대, task T 개에 대해
 *  • Layer 수  = T              (task 가 하나씩 빠질 때마다 layer 전진)
 *  • source‑robot 간선 cap = 1
 *  • robot‑layer0         cap = T
 *  • layer k‑>k+1         cap = T‑(k+1)
 *  • 마지막 layer‑>ID      cap = 1
 *  • ID‑>sink              cap = 1
 */

//robotPath에 각 로봇이 순서대로 맡는 task(0~T-1)들을 계산하여 넣어주는 함수.
void assign_tasks_mcmf(const vector<vector<int>>& distRT,
                            const vector<vector<int>>& distTT, vector<vector<int>>& robotPath)
{
    const int R = static_cast<int>(distRT.size());
    const int T = distRT.empty() ? 0 : (int)distRT[0].size();
    if (R == 0 || T == 0) return;              // 예외 처리

    // -------- 노드 인덱스 매핑 --------

    const int SRC       = 0;
    const int ROBOT_BEG = 1;                   // R 개 (0‑based offset)
    const int TASK_BEG  = ROBOT_BEG + R;       // T*T 개 (flattened layers)
    const int ID_BEG    = TASK_BEG + T * T;    // T 개 (task 번호 노드)
    const int SINK      = ID_BEG + T;
    const int NODES     = SINK + 1;

    MCMF mcmf(NODES);

    //  source → robot (cap=1, cost=0)
    for (int r = 0; r < R; ++r)
        mcmf.addEdge(SRC, ROBOT_BEG + r, 1, 0);

    //  robot → 첫 task layer (cap=T, cost=dist)
    for (int r = 0; r < R; ++r)
        for (int t = 0; t < T; ++t)
            mcmf.addEdge(ROBOT_BEG + r,
                         TASK_BEG + /*layer0*/ t,
                         T,
                         distRT[r][t]);

    //  task layer k → k+1 (cap=T‑(k+1), cost=distTT)
    for (int k = 0; k < T - 1; ++k) {
        int capHere = T - (k + 1);             // 3,2,1,…
        int fromBeg = TASK_BEG + k * T;
        int toBeg   = TASK_BEG + (k + 1) * T;

        for (int i = 0; i < T; ++i)
            for (int j = 0; j < T; ++j)
                mcmf.addEdge(fromBeg + i,
                             toBeg   + j,
                             capHere,
                             distTT[i][j]);
    }

    //  마지막 task layer → ID (cap=1, cost=0)
    int lastBeg = TASK_BEG + (T - 1) * T;
    for (int t = 0; t < T; ++t)
        mcmf.addEdge(lastBeg + t, ID_BEG + t, 1, 0);

    //  ID → sink (cap=1, cost=0)
    for (int t = 0; t < T; ++t)
        mcmf.addEdge(ID_BEG + t, SINK, 1, 0);

    // ---------- 실행 ----------
    
    auto result = mcmf.minCostMaxFlow(SRC, SINK);
    int flow       = result.first;
    int totalCost  = result.second;

    cout << "MCMF result : flow=" << flow << ", cost=" << totalCost << endl;


    /* ---------- 경로 추출 ---------- */

    const auto& G = mcmf.graph();
    robotPath.clear();
    robotPath.resize(R);

    for (int r = 0; r < R; ++r) {
        int node = ROBOT_BEG + r;                 // 로봇 노드부터 시작
        while (true) {
            bool advanced = false;
            for (const Edge& e : G[node]) {
                // forward 간선으로 실제 flow가 흘렀다면 역간선 cap > 0
                if (G[e.to][e.rev].cap > 0) {
                    // 다음 노드가 task-layer 라인인가?
                    if (e.to >= TASK_BEG && e.to < TASK_BEG + T * T) {
                        int taskIdx = (e.to - TASK_BEG) % T;     // 0 ~ T-1
                        robotPath[r].push_back(taskIdx);
                    }
                    node     = e.to;
                    advanced = true;
                    break;
                }
            }
            // ID-노드(→ SINK)까지 오면 종료
            if (!advanced || (node >= ID_BEG && node < ID_BEG + T)) break;
        }
    }

    /* ---------- 결과 확인용 출력 ---------- */
    cout << "MCMF result : flow=" << flow
              << ", cost="   << totalCost << '\n';
    for (int r = 0; r < R; ++r) {
        cout << "  Robot " << r << " :";
        for (int t : robotPath[r]) cout << ' ' << t;
        cout << '\n';
    }
}
