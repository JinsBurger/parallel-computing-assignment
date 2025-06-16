#include <cassert>
#include <cstdlib>  // getenv
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <queue>
#include <random>
#include <stack>
#include <vector>

#include "schedular.h"
#include "simulator.h"



using namespace std;

MapManager::MapManager(): tick(0), certain_coordN(0), observedN(0) {}


int MapManager::update_map_info(vector<vector<OBJECT>> object_map, vector<vector<vector<int>>> cost_map, set<Coord> observed_coords) {
    this->object_map = object_map;
    this->cost_map = cost_map;
    this->latest_observed_coords.clear();

    // Not g_initialized yet
    if(observed_map.size() != this->object_map.size()) {
        int wh = this->w = this->h = this->object_map.size();
        this->observed_map = vector<vector<int>>(wh, vector<int>(wh, -50*this->w)); // UNKNOWN MAP tick-> -1000
    }

    for(auto coord : observed_coords) {
        // If not observed
        if(observed_map[coord.x][coord.y] < 0) {
            this->latest_observed_coords.push_back(coord);
            if(object_map[coord.x][coord.y] != OBJECT::WALL) {
                avg_costs[ROBOT::TYPE::WHEEL] += cost_map[coord.x][coord.y][static_cast<int>(ROBOT::TYPE::WHEEL)];
                avg_costs[ROBOT::TYPE::CATERPILLAR] += cost_map[coord.x][coord.y][static_cast<int>(ROBOT::TYPE::CATERPILLAR)];
                avg_costs[ROBOT::TYPE::DRONE] += cost_map[coord.x][coord.y][static_cast<int>(ROBOT::TYPE::DRONE)];
                certain_coordN += 1;
            }
            observedN += 1;
        }
        this->observed_map[coord.x][coord.y] = tick;
        this->objects[object_map[coord.x][coord.y]].push_back(coord);   
    }
    return this->latest_observed_coords.size();
}

// known_cost_map sets uncertain coords to -1 and block to INT_MAX(std::numeric_limits<int>::max();) 
int MapManager::cost_at(Coord pos, ROBOT::TYPE type) {
    int cost = this->cost_map[pos.x][pos.y][static_cast<int>(type)];
    if(cost == INFINITE) {
        cost = g_infinity_cost;
    } else if (cost < 0) {
        cost = avg_costs[type] / certain_coordN;
    }
    return cost;
}

double MapManager::observed_pt() {
    return (double)observedN / (double)(w*h);
}


/* 
  ***************************************
  ************* D STAR LITE *************
  ***************************************
  Improved version from https://github.com/ykwang11/ENPM808X_Midterm-Project_D-Star-Lite-Path-Planning/blob/master/app/main.cpp
*/


// -------------------- DStarOpenList --------------------
void DStarOpenList::Insert(Key new_key,
                      const std::pair<int, int> &new_node) {
    priority_queue.push_back(std::make_tuple(
                             new_key, new_node.first, new_node.second));
    std::push_heap(priority_queue.begin(),
                   priority_queue.end(), std::greater<>());
}

void DStarOpenList::UpdateKey(Key new_key,
                         const std::pair<int, int> &position) {
    for (auto &node : priority_queue) {
        if (std::get<1>(node) == position.first &&
            std::get<2>(node) == position.second) {
            std::get<0>(node) = new_key;
            break;
        }
    }
    std::make_heap(priority_queue.begin(),
                   priority_queue.end(), std::greater<>());
}


void DStarOpenList::Remove(const std::pair<int, int> &node) {
    UpdateKey(Key(-1, -1), node);
    std::pop_heap(priority_queue.begin(),
                  priority_queue.end(), std::greater<>());
    priority_queue.pop_back();
}


std::pair<Key, std::pair<int, int>> DStarOpenList::Top() const {
    auto key = std::get<0>(priority_queue.front());
    auto position = std::make_pair(std::get<1>(priority_queue.front()),
                                   std::get<2>(priority_queue.front()));
    return std::make_pair(key, position);
}


bool DStarOpenList::Empty() const {
    return priority_queue.empty();
}

std::pair<Key, std::pair<int, int>> DStarOpenList::Pop() {
    std::pop_heap(priority_queue.begin(),
                  priority_queue.end(), std::greater<>());
    auto top_node = priority_queue.back();
    priority_queue.pop_back();
    auto key = std::get<0>(top_node);
    auto position = std::make_pair(std::get<1>(top_node),
                                   std::get<2>(top_node));
    return std::make_pair(key, position);
}


bool DStarOpenList::Find(const std::pair<int, int> &node_to_find) const {
    for (auto const &node : priority_queue) {
        if (std::get<1>(node) == node_to_find.first &&
            std::get<2>(node) == node_to_find.second)
            return true;
    }
    return false;
}



// -------------------- DStarMap --------------------
DStarMap::DStarMap(MapManager &mm, ROBOT::TYPE robot_type) : mm(mm), robot_type(robot_type) {
    std::vector<std::vector<DStarCell>> new_grid(
        mm.h,
        std::vector<DStarCell>(mm.w, DStarCell(g_infinity_cost)));
    grid = new_grid;
    map_size = std::make_pair(mm.h, mm.w);

    for(int i=0; i < mm.h; i++)
        for(int j=0; j < mm.w; j++)
            UpdateCellStatus({i, j}, "$");
}


void DStarMap::SetStart(const std::pair<int, int> &new_start) {
    start = new_start;
    km += Heuristic(new_start, goal);
    UpdateCellStatus(start, start_mark);
    
}

void DStarMap::SetGoal(const std::pair<int, int> &new_goal) {
    goal = new_goal;
    UpdateCellStatus(new_goal, goal_mark);
}


Key DStarMap::CalculateCellKey(const std::pair<int, int> &position) const {
    int h = Heuristic(position, this->goal);
    int key2 = std::min(CurrentCellG(position), CurrentCellRhs(position));
    int key1 = key2+h+km;
    return Key(key1, key2);
}


uint32_t DStarMap::ComputeCost(const std::pair<int, int> &pos) {
    if (!Availability(pos)) return g_infinity_cost;
    return this->mm.cost_at({pos.second, pos.first}, this->robot_type);
}


std::vector<std::pair<int, int>> DStarMap::FindNeighbors(
    const std::pair<int, int> & position) {
    std::vector<std::pair<int, int>> neighbors = {};
    std::vector<std::pair<int, int>> search_neighbor = {
        {1,0}, {-1,0}, {0,1},{0,-1}
    };

    for (auto const &i : search_neighbor) {
        auto neighbor = std::make_pair(position.first + i.first,
                                        position.second + i.second);
        auto cost = ComputeCost(neighbor);
        if (cost < g_infinity_cost)
            neighbors.push_back(neighbor);
    }
    return neighbors;
}


bool DStarMap::Availability(const std::pair<int, int> & position) {
    if (position.first < 0 || position.first >= map_size.first) return false;
    if (position.second < 0 || position.second >= map_size.second) return false;
    return true;
}

void DStarMap::PrintValue() {
    std::vector<std::string> lines(map_size.second, "-------------");
    std::cout << "Value for shortest path:" << std::endl
        << "(g, rhs): " << std::endl<< " -";
    for (auto line : lines) std::cout << line;
    std::cout << std::endl;
    for (auto const &row : grid) {
        std::cout << " | ";
        for (auto const &cell : row) {
            std::cout << "(" << std::setfill(' ') << std::setw(3)
                      << cell.CurrentG() << ", "  << std::setfill(' ')
                      << std::setw(3) << cell.CurrentRhs() << ") | ";
        }
        std::cout << std::endl << " -";
        for (auto line : lines) std::cout << line;
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void DStarMap::PrintResult(std::vector<std::vector<DStarCell>> &tgrid) {
    std::vector<std::string> lines(map_size.second, "----");
    std::cout << "Result: " << std::endl
        << "start: " << start_mark << " goal: " << goal_mark << " robot: "
        << robot_mark << " obstacle: " << obstacle_mark << " unknown: "
        << unknown_mark << std::endl;
    std::cout << " -";
    for (auto line : lines) std::cout << line;
    std::cout << std::endl;
    for (auto const &row : tgrid) {
        std::cout << " | ";
        for (auto const &cell : row) {
            std::cout << cell.CurrentStatus() << " | ";
        }
        std::cout << std::endl << " -";
        for (auto line : lines) std::cout << line;
        std::cout << std::endl;
    }
    std::cout << std::endl;
}


// -------------------- DStarImpl --------------------
DStarImpl::DStarImpl(MapManager &mm, ROBOT::TYPE robot_type)
    : map(DStarMap(mm, robot_type)), robot_type(robot_type) {}

void DStarImpl::Initialize() {
    auto goal_rhs = 0.0;
    this->map.UpdateCellRhs(this->map.GetGoal(), goal_rhs);
    auto new_key = this->map.CalculateCellKey(this->map.GetGoal());
    this->openlist.Insert(new_key, this->map.GetGoal());
}

void DStarImpl::ComputeShortestPath() {
    std::pair<int, int> robot_pos = this->map.GetStart();

    while (!this->openlist.Empty() && (this->openlist.Top().first <
        this->map.CalculateCellKey(robot_pos) ||
        this->map.CurrentCellRhs(robot_pos) !=
        this->map.CurrentCellG(robot_pos))) {
        auto key_and_node = this->openlist.Pop();
        auto node = key_and_node.second;

        auto old_key = key_and_node.first;
        auto new_key = this->map.CalculateCellKey(node);

        if (old_key < new_key) {
            this->openlist.Insert(new_key, node);
        } else if (this->map.CurrentCellG(node) >
                this->map.CurrentCellRhs(node)) {
            this->map.UpdateCellG(node, this->map.CurrentCellRhs(node));
            for (const auto &vertex : this->map.FindNeighbors(node)) {
                UpdateVertex(vertex);
            }
        } else {
            this->map.SetInfiityCellG(node);
            UpdateVertex(node);
            for (const auto &vertex : this->map.FindNeighbors(node)) {
                UpdateVertex(vertex);
            }
        }
    }
}

void DStarImpl::UpdateVertex(const std::pair<int, int> &vertex) {
    if (vertex != this->map.GetGoal()) {
        this->map.UpdateCellRhs(vertex, ComputeMinRhs(vertex));
    }
    if (this->openlist.Find(vertex)) {
        this->openlist.Remove(vertex);
    }
    if (this->map.CurrentCellG(vertex) != this->map.CurrentCellRhs(vertex)) {
        this->openlist.Insert(this->map.CalculateCellKey(vertex), vertex);
    }
}

int DStarImpl::ComputeMinRhs(const std::pair<int, int> &vertex) {
    uint32_t min_rhs = g_infinity_cost;
    auto neibors = this->map.FindNeighbors(vertex);
    for (const auto &next_vertex : neibors) {
        uint32_t cost = this->map.ComputeCost(next_vertex);
        auto temp_rhs = cost + this->map.CurrentCellG(next_vertex);
        if (temp_rhs < min_rhs) min_rhs = temp_rhs;
    }
    return min_rhs;
}

std::pair<int, int> DStarImpl::ComputeNextPotision(const std::pair<int, int> &current_position) {
    auto next_position = current_position;
    uint32_t cheaest_cost = g_infinity_cost;
    for (const auto &candidate : this->map.FindNeighbors(current_position)) {
        uint32_t cost = map.ComputeCost(candidate) +
                map.CurrentCellG(candidate);
        if (cost < cheaest_cost) {
            cheaest_cost = cost;
            next_position = candidate;
        }
    }
    return next_position;
}

void DStarImpl::UpdateCellAndNeighbors(std::pair<int, int> pos) {
    UpdateVertex(pos);
    for (const auto &candidate_neighbor : this->map.FindNeighbors(pos)) {
        UpdateVertex(candidate_neighbor);
    }
}

void DStarImpl::AddObstacle(std::pair<int, int> obstacle_pos) {
    this->map.AddObstacle(obstacle_pos);
    UpdateCellAndNeighbors(obstacle_pos);
    this->map.UpdateCellRhs(obstacle_pos, g_infinity_cost);
    this->map.UpdateCellG(obstacle_pos, g_infinity_cost);
}


// -------------------- TaskDstarLite --------------------
TaskDstarLite::TaskDstarLite(int x, int y, MapManager &mm)
    : task_x(x), task_y(y), mm(mm) {

    RUN_All_Types(
        dstars_map[_type] = std::make_unique<DStarImpl>(mm, _type);
        dstars_map[_type]->map.SetGoal(std::make_pair(task_y, task_x));
        dstars_map[_type]->Initialize();
    );

    load_maps();
    replanning();
}


int TaskDstarLite::calculate_cost(Coord pos, ROBOT::TYPE robot_type, std::vector<Coord> &path) {
    std::pair<int, int> robot_pos = {pos.y, pos.x};
    uint32_t cost = 0;
    dstars_map[robot_type]->map.SetStart(robot_pos);
    dstars_map[robot_type]->ComputeShortestPath();
    if (dstars_map[robot_type]->map.CurrentCellG(robot_pos) >= g_infinity_cost)
        return -1;

#ifdef DSTAR_VERBOSE
    std::vector<std::vector<DStarCell>> new_grid;
    dstars_map[robot_type]->map.clone_grid(new_grid);
#endif

    while (robot_pos != dstars_map[robot_type]->map.GetGoal()) {
        robot_pos = dstars_map[robot_type]->ComputeNextPotision(robot_pos);
        path.push_back({robot_pos.second, robot_pos.first});
        cost += mm.cost_at({robot_pos.second, robot_pos.first}, robot_type);

#ifdef DSTAR_VERBOSE
        if (robot_pos != dstars_map[robot_type]->map.GetGoal()) {
            if (new_grid.at(robot_pos.first).at(robot_pos.second).CurrentStatus() == "$") {
                new_grid.at(robot_pos.first).at(robot_pos.second).UpdateStatus("\033[32;43m■\033[0m");
            } else {
                new_grid.at(robot_pos.first).at(robot_pos.second).UpdateStatus("\033[31;43m■\033[0m");
            }
        }
#endif
    }

#ifdef DSTAR_VERBOSE
    std::cout << "COST: " << cost << std::endl;
    dstars_map[robot_type]->map.PrintResult(new_grid);
#endif

    return cost;
}

void TaskDstarLite::update_object(std::pair<int, int> pos) {
    if (mm.object_map[pos.second][pos.first] == OBJECT::WALL) {
        RUN_All_Types(
            dstars_map[_type]->AddObstacle(pos);
        );
    } else {
        RUN_All_Types(
            dstars_map[_type]->map.UpdateCellStatus(pos, " ");
            dstars_map[_type]->UpdateCellAndNeighbors(pos);
        );
    }
}

void TaskDstarLite::load_maps() {
    for (size_t x = 0; x < mm.observed_map.size(); x++) {
        for (size_t y = 0; y < mm.observed_map.size(); y++) {
            if (mm.observed_map[x][y] >= 0)
                update_object({y, x});
        }
    }
}

void TaskDstarLite::update_latest_coords() {
    for (Coord v : mm.latest_observed_coords) {
        update_object({v.y, v.x});
    }
}





map<int, TaskDstarLite> g_tasks_dstar;

MapManager g_map_manager;

// 매크로 선언 (필요 시 컴파일 시 -Dgravity_mode 추가)
//#define gravity_mode


map<int, int> g_original_energy;
static map<int, vector<vector<int>>> g_last_seen_time;
map<int, queue<Coord>> g_robot_task;
map<int, vector<Coord>> g_drone_path;

map<int,int> g_record_start;
map<int,Coord> g_record_target_coord;

map<int, DRONE_MODE> g_drone_mode;
map<int, Coord> g_best_frontiers;
map<int, vector<Coord>> g_unreachable_frontiers;
int g_last_task_reach_tick = -1;

static map<int, deque<Coord>> g_drone_stack;
static map<int, vector<vector<bool>>> g_visited;
static map<int, bool> g_initialized;


void Scheduler::on_info_updated(const set<Coord> &observed_coords,
                                const set<Coord> &updated_coords,
                                const vector<vector<vector<int>>> &known_cost_map,
                                const vector<vector<OBJECT>> &known_object_map,
                                const vector<shared_ptr<TASK>> &active_tasks,
                                const vector<shared_ptr<ROBOT>> &robots)
{

    if(g_map_manager.tick == 0) {
        for(auto &r : robots) {
            g_original_energy[r->id] = r->get_energy();
        }
    }
    g_map_manager.tick += 1;


    //Replanning
    int new_observed_coords = g_map_manager.update_map_info(known_object_map, known_cost_map, observed_coords);
    int is_replanning_needed = new_observed_coords > 0;

    for(auto task : active_tasks) {
        if(g_tasks_dstar.find(task->id) == g_tasks_dstar.end()) {
            g_tasks_dstar.insert(make_pair(task->id, TaskDstarLite(task->coord.x, task->coord.y, g_map_manager)));
        } else if(is_replanning_needed) {
            //None of coords found newly, not need to conduct replanning.
            g_tasks_dstar.at(task->id).replanning();
        }
    }

    
    
    if(g_last_task_reach_tick == (g_map_manager.tick-1) || (is_replanning_needed && active_tasks.size()>0)){

        SCH_LOG("mcmf part starts\n");
        vector<vector<int>> distRT, distTT, robotPath;
        vector<vector<vector<Coord>>> RtoT;
        int cnt=0;
        for(const auto& robotPtr : robots){
            if(robotPtr->type != ROBOT::TYPE::DRONE) cnt++;
        }
        distRT = vector<vector<int>>(cnt, vector<int>(active_tasks.size()));
        distTT = vector<vector<int>>(active_tasks.size(), vector<int>(active_tasks.size()));
        RtoT = vector<vector<vector<Coord>>>(cnt,vector<vector<Coord>>(active_tasks.size()));
        robotPath.resize(cnt);
        //SCH_LOG("resize done\n");
        size_t i=0, j=0;
        int index_to_task[100] = {0,};
        for(auto task : active_tasks){
            auto &task_dstar = g_tasks_dstar.at(task->id);
            j=0;
            index_to_task[i]=task->id;
            for(const auto& robotPtr : robots){
                if(robotPtr->type == ROBOT::TYPE::CATERPILLAR || robotPtr->type == ROBOT::TYPE::WHEEL) {
                    distRT[j][i] = task_dstar.calculate_cost(robotPtr->get_coord(), robotPtr->type, RtoT[j][i]);

                    if(distRT[j][i] == -1) 
                        distRT[j][i] = g_infinity_cost;
                    else{ 
                        if(g_record_start.find(robotPtr->id) == g_record_start.end()){
                            g_record_start[robotPtr->id] = 0;
                        }
                        if(robotPtr->get_status() == ROBOT::STATUS::MOVING && g_record_target_coord.find(robotPtr->id) != g_record_target_coord.end()){
                            distRT[j][i] += max(0, g_map_manager.cost_at(g_record_target_coord[robotPtr->id], robotPtr->type)
                                            - ROBOT::energy_per_tick_list[static_cast<size_t>(robotPtr->type)] * (g_map_manager.tick - g_record_start[robotPtr->id]));
                        }
                        if(robotPtr->get_status() == ROBOT::STATUS::WORKING){
                            auto it = active_tasks.begin();
                            for (; it != active_tasks.end(); ++it){
                                if ((*it)->coord == robotPtr->get_coord() && !(*it)->is_done())
                                    break;
                            }
                            if(it != active_tasks.end()){
                                distRT[j][i] += (*it)->get_cost(robotPtr->type)
                                                - ROBOT::energy_per_tick_list[static_cast<size_t>(robotPtr->type)] * (g_map_manager.tick - g_record_start[robotPtr->id]);
                            }
                        }
                        if(distRT[j][i] > robotPtr->get_energy())
                            distRT[j][i] = g_infinity_cost;
                    }
                    j++;
                }
            }
            i++;
        }


        SCH_LOG("call assign_tasks_mcmf\n");
        assign_tasks_mcmf(distRT, distTT, robotPath);

        for(int i=0; i<cnt; i++){
            if(robotPath[i].size()==2){
                if(distRT[i][robotPath[i][0]] > distRT[i][robotPath[i][1]]){
                    swap(robotPath[i][0], robotPath[i][1]);
                }
            }
        }
     
        SCH_LOG("index_to_task: ");
        for(i=0; i<active_tasks.size(); i++){
            SCH_LOG(index_to_task[i] << " ");
        }
        SCH_LOG(endl);


        g_robot_task.clear();
        i=0;
        vector<pair<int, Coord>> drone_info;
        SCH_LOG("MCMF Results:" << endl);
        for(const auto& robotPtr : robots){
            if(robotPtr->type == ROBOT::TYPE::DRONE){
                drone_info.push_back({robotPtr->id, robotPtr->get_coord()});
                continue;
            }
            SCH_LOG(robotPtr->id << ": ");
            for(size_t j=0; j<robotPath[i].size(); j++){
                SCH_LOG("{" << robotPath[i][j] << ", " << index_to_task[robotPath[i][j]] << "} ");
                for(size_t k=0; k<RtoT[i][robotPath[i][j]].size(); k++){
                    g_robot_task[robotPtr->id].push(RtoT[i][robotPath[i][j]][k]);
                }
            }
            i++;
            SCH_LOG(endl);
        }
        
        i=0;

        SCH_LOG("g_robot_task created\n");
    }

    int map_size = static_cast<int>(known_object_map.size());
    for (size_t id = 0; id < robots.size(); ++id) {
        if (g_last_seen_time.count(id) == 0) {
            g_last_seen_time[id] = vector<vector<int>>(map_size, vector<int>(map_size, -1));
        }
        for (const auto& coord : updated_coords) {
            if (coord.x >= 0 && coord.x < map_size && coord.y >= 0 && coord.y < map_size) {
                g_last_seen_time[id][coord.x][coord.y] = g_map_manager.tick;
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
    bool res = robot.type != ROBOT::TYPE::DRONE;
    if(res){
        g_record_start[robot.id] = g_last_task_reach_tick = g_map_manager.tick;
    }
    return res;
}

// Helper: Check if coord is inside map bounds
bool is_valid_coord(const Coord& c, int map_size) {
    return c.x >= 0 && c.x < map_size && c.y >= 0 && c.y < map_size;
}


int get_env_or_default(const char* name, int default_value) {
    const char* val = std::getenv(name);
    if (val) {
        try {
            return std::stoi(val);
        } catch (...) {
            return default_value;
        }
    }
    return default_value;
}


// Helper: Compute frontier score (higher is better)
int frontier_score(const Coord& c, const vector<vector<OBJECT>>& map, const vector<vector<int>>& observed_map,
                   int map_size, const vector<shared_ptr<ROBOT>>& robots,
                   const Coord& self_coord, const Coord& other_frontier_target) {
    if (!is_valid_coord(c, map_size)) return -1;
    if (map[c.x][c.y] == OBJECT::WALL) return -1;

    bool has_unknown = false;
    int tick_sum = 0, tick_cnt = 0;
    int unknown_count = 0;

    for (int dx = -2; dx <= 2; ++dx) {
        for (int dy = -2; dy <= 2; ++dy) {
            Coord check = {c.x + dx, c.y + dy};
            if (!is_valid_coord(check, map_size)) continue;
            if (map[check.x][check.y] == OBJECT::UNKNOWN)
                unknown_count++;
        }
    }

    int unknown_scope = get_env_or_default("UNKNOWN_SCOPE", 3);

    for (int dx = -1 * unknown_scope; dx <= unknown_scope; ++dx) {
        for (int dy = -1 * unknown_scope; dy <= unknown_scope; ++dy) {
            Coord check = {c.x + dx, c.y + dy};
            if (!is_valid_coord(check, map_size)) continue;
            if (map[check.x][check.y] == OBJECT::UNKNOWN)
                has_unknown = true;
            tick_sum += observed_map[check.x][check.y];
            tick_cnt++;
        }
    }
    if (!has_unknown) return -1;

    int dist_to_other = -1;
    for (const auto& r : robots) {
        if (r->type != ROBOT::TYPE::DRONE) continue;
        Coord rc = r->get_coord();
        if (rc == self_coord) continue; // 본인 제외
        int d = abs(c.x - rc.x) + abs(c.y - rc.y);
        if (dist_to_other == -1 || d < dist_to_other)
            dist_to_other = d;
    }
    if (dist_to_other == -1) dist_to_other = 0; // 다른 드론이 없을 경우
    
    double tick_avg = (tick_cnt > 0) ? static_cast<double>(tick_sum) / tick_cnt : 0.0;
    int dist_to_self = abs(c.x - self_coord.x) + abs(c.y - self_coord.y);

    int tick_weight             = get_env_or_default("WEIGHT_TICK", 4);
    int dist_to_other_weight   = get_env_or_default("WEIGHT_DIST_OTHER", 7);
    int dist_to_self_weight    = get_env_or_default("WEIGHT_DIST_SELF", 11);
    int frontier_conflict_weight = get_env_or_default("WEIGHT_FRONTIER_CONFLICT", 7);
    int unknown_count_weight   = get_env_or_default("WEIGHT_UNKNOWN_COUNT", 4);

    int frontier_conflict_dist = 0;
    if (is_valid_coord(other_frontier_target, map_size)) {
        frontier_conflict_dist = abs(c.x - other_frontier_target.x) + abs(c.y - other_frontier_target.y);
    }

    int final_score = static_cast<int>(10000
        - (tick_avg / map_size) * tick_weight
        + dist_to_other * dist_to_other_weight
        - dist_to_self * dist_to_self_weight
        + frontier_conflict_dist * frontier_conflict_weight
        + unknown_count * unknown_count_weight);

    SCH_LOG("Frontier score for " << c << " = " << final_score << " -> 10000 - "
         << (tick_avg / map_size) * tick_weight << " (tick_avg: " << tick_avg << ") + "
         << dist_to_other * dist_to_other_weight << " (dist_to_other: " << dist_to_other << ") - "
         << dist_to_self * dist_to_self_weight << " (dist_to_self: " << dist_to_self << ") - "
         << frontier_conflict_dist * frontier_conflict_weight << " (conflict_dist: " << frontier_conflict_dist << ") + "
         << unknown_count * unknown_count_weight << " (unknown7x7: " << unknown_count << ")"
         << endl);

    return static_cast<int>(final_score);
}


ROBOT::ACTION Scheduler::idle_action(const set<Coord> &observed_coords,
                                     const set<Coord> &updated_coords,
                                     const vector<vector<vector<int>>> &known_cost_map,
                                     const vector<vector<OBJECT>> &known_object_map,
                                     const vector<shared_ptr<TASK>> &active_tasks,
                                     const vector<shared_ptr<ROBOT>> &robots,
                                     const ROBOT &robot)
{   
    // WHEEL, CATERPILLAR
    if (robot.type != ROBOT::TYPE::DRONE) {
        bool robot_move_flag;
        bool drone_alive = true;

        //Check if all drones are exhausted
        int exhausted_percent = get_env_or_default("WEIGHT_EXHAUSTED_PERCENT", 85);
        for(auto &r : robots) {
            //if(r->type == ROBOT::TYPE::DRONE && r->get_status() != ROBOT::STATUS::EXHAUSTED) {
            if(r->type == ROBOT::TYPE::DRONE && ((double)r->get_energy() / (double)g_original_energy[r->id]) < 0.01*(100-exhausted_percent)) {
                drone_alive = false;
                break;
            }
        }
        robot_move_flag = !drone_alive; //|| !(g_map_manager.observed_pt() > 0.8);
        
        ROBOT::ACTION res = ROBOT::ACTION::HOLD;
        if (g_robot_task.find(robot.id) != g_robot_task.end() && !g_robot_task[robot.id].empty()) {
            for (int dir = 0; dir < 4; dir++) {
                if (robot.get_coord() + DIRECTIONS[dir] == g_robot_task[robot.id].front()) {
                    res = static_cast<ROBOT::ACTION>(dir);
                    break;
                }
            }
            g_robot_task[robot.id].pop();
        }

        if (res != ROBOT::ACTION::HOLD) {
            g_record_target_coord[robot.id] = robot.get_coord() + DIRECTIONS[static_cast<int>(res)];
            g_record_start[robot.id] = g_map_manager.tick;
            return res;
        } else if (g_robot_task[robot.id].size() != 0 || !robot_move_flag) {
            return res;
        }
        

        // ROBOT runs drone's mechanism if all drones are dead
    }

    /************** DRONE **************/
    int map_size = static_cast<int>(known_object_map.size());
    int id = robot.id;
    Coord curr = robot.get_coord();

    // Scan MAP with DFS & FRONTIER
    g_drone_stack[id].push_back(curr);

    int is_drone = static_cast<int>(robot.type == ROBOT::TYPE::DRONE);

    if (!g_initialized[id]) {
        g_initialized[id] = true;
        if(g_visited[is_drone].size() == 0)
            g_visited[is_drone] = vector<vector<bool>>(map_size, vector<bool>(map_size, false));
        g_drone_mode[robot.id] = DRONE_MODE::DFS;
        SCH_LOG("[Drone " << id << "] Initialized at " << curr << endl);
    }
    
    g_visited[is_drone][curr.x][curr.y] = true;

    if(g_drone_mode[robot.id] == DRONE_MODE::FRONTIER && observed_coords.find(g_best_frontiers[robot.id]) != observed_coords.end()) {
        g_drone_mode[robot.id] = DRONE_MODE::DFS;
    }

    if(g_drone_mode[robot.id] == DRONE_MODE::DFS){
        vector<int> candidate_dir;
        vector<int> unexplored_count(4, 0);

        for (int dir = 0; dir < 4; ++dir) {
            Coord next = curr + DIRECTIONS[dir];
            if (!is_valid_coord(next, map_size)) continue;
            if (known_object_map[next.x][next.y] == OBJECT::WALL) continue;
            if (!g_visited[is_drone][next.x][next.y]) {
                int cnt = 0;
                for (int i = -2; i <= 2; ++i) {
                    for (int j = 2; j <= 3; ++j) {
                        Coord check = next;
                        if (dir == 0) check = {next.x + i, next.y + j};
                        else if (dir == 1) check = {next.x + i, next.y - j};
                        else if (dir == 2) check = {next.x - j, next.y + i};
                        else check = {next.x + j, next.y + i};

                        if (!is_valid_coord(check, map_size)) continue;
                        if (known_object_map[check.x][check.y] == OBJECT::UNKNOWN)
                            cnt++;
                    }
                }
                if (cnt > 0) {
                    candidate_dir.push_back(dir);
                    unexplored_count[dir] = cnt;
                }
            }
        }


        if (!candidate_dir.empty()) {
            int best_dir = candidate_dir[0];
            for (int dir : candidate_dir)
                if (unexplored_count[dir] > unexplored_count[best_dir])
                    best_dir = dir;

            vector<int> best_candidates;
            for (int dir : candidate_dir)
                if (unexplored_count[dir] == unexplored_count[best_dir])
                    best_candidates.push_back(dir);

            if (best_candidates.size() > 1 && robots.size() >= 2) {
                Coord other = (robots[0]->id == id) ? robots[1]->get_coord() : robots[0]->get_coord();
                int dx = other.x - curr.x;
                int dy = other.y - curr.y;
                int toward = 0;
                if (abs(dx) > abs(dy)) toward = (dx > 0) ? 3 : 2;
                else toward = (dy > 0) ? 0 : 1;
                int avoid = (toward ^ 1);

                for (int cand : best_candidates) {
                    if (cand == avoid) {
                        best_dir = cand;
                        break;
                    }
                }
                if(best_dir == toward && best_candidates.size() > 1) {
                    for (int cand : best_candidates) {
                        if (cand != toward && cand != avoid) {
                            best_dir = cand;
                            break;
                        }
                    }
                }
            }

            Coord next = curr + DIRECTIONS[best_dir];
            SCH_LOG("[Drone " << id << "] DFS → Move " << best_dir << " to " << next << endl);
            return static_cast<ROBOT::ACTION>(best_dir);
        }
        else{
            // If no unexplored neighbors, try to find a frontier
            g_drone_mode[robot.id] = DRONE_MODE::FRONTIER;
            Coord best_frontier = {-1, -1};
            
            int best_score = -1;
            const auto& obs_map = g_map_manager.observed_map;

            Coord conflict_target = {-1, -1};
            for (const auto& [other_id, mode] : g_drone_mode) {
                if (other_id != id && mode == DRONE_MODE::FRONTIER && g_best_frontiers.count(other_id)) {
                    conflict_target = g_best_frontiers.at(other_id);
                    break;
                }
            }

            for (int x = 0; x < map_size; ++x) {
                for (int y = 0; y < map_size; ++y) {
                    Coord c = {x, y};
                    if(find(g_unreachable_frontiers[robot.id].begin(), g_unreachable_frontiers[robot.id].end(), c) != g_unreachable_frontiers[robot.id].end()) {
                        continue;
                    }
                    //if (known_object_map[x][y] == OBJECT::UNKNOWN || known_object_map[x][y] == OBJECT::WALL) continue;
                    int score = frontier_score(c, known_object_map, obs_map, map_size, robots, curr, conflict_target);
                    if (score > best_score) {
                        best_score = score;
                        best_frontier = c;
                    }
                }
            }
            SCH_LOG("[Drone " << id << "] Best frontier: " << best_frontier << " with score " << best_score << " from " << curr << endl);

            if(best_frontier.x == -1){
                // Backtrack if no unexplored neighbors
                g_drone_mode[robot.id] = DRONE_MODE::WORK_DONE;
            } else {
                g_best_frontiers[robot.id] = best_frontier;
            }
        }
    }


    if(g_drone_mode[robot.id] == DRONE_MODE::FRONTIER){
        if (known_object_map[g_best_frontiers[robot.id].x][g_best_frontiers[robot.id].y] == OBJECT::WALL) {
            SCH_LOG("[Drone " << id << "] Best frontier is a wall, switching to DFS" << endl);
            g_drone_mode[robot.id] = DRONE_MODE::DFS;
            return ROBOT::ACTION::HOLD;
        }
        TaskDstarLite tmp_dstar(g_best_frontiers[robot.id].x, g_best_frontiers[robot.id].y, g_map_manager);
        vector<Coord> frontier_path;
        int av = tmp_dstar.calculate_cost(robot.get_coord(), robot.type, frontier_path);
        if(av == -1 || frontier_path.size() == 0){
            g_unreachable_frontiers[robot.id].push_back(g_best_frontiers[robot.id]);
            SCH_LOG("[Drone " << id << "] is trapped!" << endl);
            g_drone_mode[robot.id] = DRONE_MODE::DFS;
            return ROBOT::ACTION::HOLD;
        }

        g_drone_path[robot.id] = frontier_path;
        ROBOT::ACTION res = ROBOT::ACTION::HOLD;
        Coord next_dir = frontier_path[0];
        for (int dir = 0; dir < 4; dir++) {
            if (robot.get_coord() + DIRECTIONS[dir] == next_dir) {
                res = static_cast<ROBOT::ACTION>(dir);
                break;
            }
        }
        return res;
    }

    if(g_drone_mode[robot.id] == DRONE_MODE::WORK_DONE){
        g_drone_stack[id].pop_back();
        if (!g_drone_stack[id].empty()) {
            Coord parent = g_drone_stack[id].back();
            for (int dir = 0; dir < 4; ++dir) {
                if (curr + DIRECTIONS[dir] == parent) {
                    SCH_LOG("[Drone " << id << "] Final fallback → Backtrack to " << parent << " dir=" << dir << endl);
                    return static_cast<ROBOT::ACTION>(dir);
                }
            }
        }

        SCH_LOG( "[Drone " << id << "] No move available → HOLD" << endl);
        return ROBOT::ACTION::HOLD;
    }
}



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
public:
    int N;                                       // 정점 수
    std::vector<std::vector<Edge>> G;            // 인접 리스트
    explicit MCMF(int n) : N(n), G(n) {}

    // 정방향용 cap, 역방향용 cap=0 / cost=-cost 생성
    void addEdge(int s, int t, int cap, int cost) {
        Edge fwd{t, static_cast<int>(G[t].size()), cap,  cost};
        Edge rev{s, static_cast<int>(G[s].size()), 0,  -cost};
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
    const int MAX_TASK_PER_ROBOT = 2;

    SCH_LOG("R=" << R << ", T=" << T << endl);
    if (R == 0 || T == 0) return;              // 예외 처리

    // -------- 노드 인덱스 매핑 --------

    const int SRC       = 0;
    const int ROBOT_BEG = 1;                   // R 개 (0‑based offset)
    const int TASK_BEG  = ROBOT_BEG + R;       // T*T 개 (flattened layers)
    const int SINK      = TASK_BEG + T;
    const int NODES     = SINK + 1;

    MCMF mcmf(NODES);

    //  source → robot (cap=1, cost=0)
    for (int r = 0; r < R; ++r)
        mcmf.addEdge(SRC, ROBOT_BEG + r, MAX_TASK_PER_ROBOT, 0);

    //  robot → task layer (cap=1, cost=dist)
    for (int r = 0; r < R; ++r){
        for (int t = 0; t < T; ++t){
            mcmf.addEdge(ROBOT_BEG + r, TASK_BEG + t, 1, distRT[r][t]);
        }
    }

    //  task layer → sink (cap=1, cost=0)
    for (int t = 0; t < T; ++t)
        mcmf.addEdge(TASK_BEG + t, SINK, 1, 0);
 
    /* 
        SCH_LOG("before MCMF:\n");
        for(int i=0; i<mcmf.G.size(); i++){
            for(int j=0; j<mcmf.G[i].size(); j++){
                SCH_LOG("%d %d %d %d\n",i,mcmf.G[i][j].to,mcmf.G[i][j].cap,mcmf.G[i][j].cost);
            }
        }
    */
    // ---------- 실행 ----------
    
    auto result = mcmf.minCostMaxFlow(SRC, SINK);
    int flow       = result.first;
    int totalCost  = result.second;


    /* ---------- 경로 추출 ---------- */

    const auto& G = mcmf.graph();
    robotPath.clear();
    robotPath = vector<vector<int>>(R, vector<int>(0));

    /*  
        SCH_LOG("After mcmf:\n");
        for(int i=0; i<mcmf.G.size(); i++){
            for(int j=0; j<mcmf.G[i].size(); j++){
                SCH_LOG("%d %d %d %d\n",i,mcmf.G[i][j].to,mcmf.G[i][j].cap,mcmf.G[i][j].cost);
            }
        }
    */
    
    for (int r = 0; r < R; ++r) {
        int node = ROBOT_BEG + r;                 // 로봇 노드부터 시작
        for (const Edge& e : G[node]) {
            if(e.to < node) continue;
            if (G[e.to][e.rev].cap > 0) {
                int taskIdx = (e.to - TASK_BEG) % T;     // 0 ~ T-1
                robotPath[r].push_back(taskIdx);
            }
        }
    }

}