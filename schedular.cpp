#include "schedular.h"
#include "simulator.h"
#include <stack>
#include <random>
#include <map>
#include <vector>
#include <queue>
#include <limits>


using namespace std;
//#define gravity_mode


MapManager::MapManager(): tick(0) {}


int MapManager::update_map_info(vector<vector<OBJECT>> object_map, vector<vector<vector<int>>> cost_map, set<Coord> observed_coords, set<Coord> updated_coords) {
    this->object_map = object_map;
    this->cost_map = cost_map;
    this->latest_observed_coords.clear();

    // Not initialized yet
    if(observed_map.size() != this->object_map.size()) {
        int wh = this->w = this->h = this->object_map.size();
        this->observed_map = vector<vector<int>>(wh, vector<int>(wh, -1));
    }

    for(auto coord : observed_coords) {
        // If not observed
        if(observed_map[coord.x][coord.y] < 0) {
            this->latest_observed_coords.push_back(coord);
            if(object_map[coord.x][coord.y] != OBJECT::WALL) {
                avg_costs[ROBOT::TYPE::WHEEL] += cost_map[coord.x][coord.y][static_cast<int>(ROBOT::TYPE::WHEEL)];
                avg_costs[ROBOT::TYPE::CATERPILLAR] += cost_map[coord.x][coord.y][static_cast<int>(ROBOT::TYPE::CATERPILLAR)];
                certain_coordN += 1;
            }
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
    }
    else {
        // TODO: if cost < 0 (= uncertain cells yet ),  returns the average of the costs of the "movable" cells.
        //   - cost = avg_costs[type] / certain_coordN;
        cost = 1;
    }
    return cost;
}




/* 
  ***************************************
  ************* D STAR LITE *************
  ***************************************
  Improved version from https://github.com/ykwang11/ENPM808X_Midterm-Project_D-Star-Lite-Path-Planning/blob/master/app/main.cpp
*/


/**
 * @brief Constructor.
 * @param initial_num a number for the inital g-value and rhs-value
 * @return none
 */
DStarCell::DStarCell(const int &initial_num) {
    g = initial_num;
    rhs = initial_num;
    status = " ";
}

/**
 * @brief Get current g-value.
 * @return g-value
 */
int DStarCell::CurrentG() const { return g; }

/**
 * @brief Get current rhs-value.
 * @return rhs-value
 */
int DStarCell::CurrentRhs() const { return rhs; }

/**
 * @brief Get current status: obstacle, unknown obstacle, goal, or start point.
 * @return mark that represent the status
 */
std::string DStarCell::CurrentStatus() const { return status; }

/**
 * @brief Set new g-value.
 * @param new_g new estamated distance to the goal
 * @return none
 */
void DStarCell::UpdateG(const int &new_g) { g = new_g; }

/**
 * @brief Set new rhs-value.
 * @param new_rhs one step lookahead values based on the g-values
 * @return none
 */
void DStarCell::UpdateRhs(const int &new_rhs) { rhs = new_rhs; }

/**
 * @brief Set new status.
 * @param new_status a mark represent new status
 * @return none
 */
void DStarCell::UpdateStatus(const std::string &new_status) { status = new_status; }

/**
 * @brief Inset a node in the open list.
 * @param new_key thepriority of the node to be added
 * @param new_node a candidate node's priority in searching and its position
 * @return none
 */
void DStarOpenList::Insert(Key new_key,
                      const std::pair<int, int> &new_node) {
    priority_queue.push_back(std::make_tuple(
                             new_key, new_node.first, new_node.second));
    std::push_heap(priority_queue.begin(),
                   priority_queue.end(), std::greater<>());
}

/**
 * @brief Update the key of node in the open list.
 * @param new_key thepriority of the node to be changed
 * @param position a candidate node's new priority in searching and its position
 * @return none
 */
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

/**
 * @brief Remove a node from the open list.
 * @param node a node's position
 * @return none
 */
void DStarOpenList::Remove(const std::pair<int, int> &node) {
    UpdateKey(Key(-1, -1), node);
    std::pop_heap(priority_queue.begin(),
                  priority_queue.end(), std::greater<>());
    priority_queue.pop_back();
}

/**
 * @brief Get the node on the top of the open list (a minimum heap).
 * @return the top node's priority in searching and its position
 */
std::pair<Key, std::pair<int, int>> DStarOpenList::Top() const {
    auto key = std::get<0>(priority_queue.front());
    auto position = std::make_pair(std::get<1>(priority_queue.front()),
                                   std::get<2>(priority_queue.front()));
    return std::make_pair(key, position);
}

/**
 * @brief Get the node on the top of the open list and romovee it.
 * @return the top node's priority in searching and its position
 */
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

/**
 * @brief Find if a node is in the open list.
 * @return true if the node exsit and false if not
 */
bool DStarOpenList::Find(const std::pair<int, int> &node_to_find) const {
    for (auto const &node : priority_queue) {
        if (std::get<1>(node) == node_to_find.first &&
            std::get<2>(node) == node_to_find.second)
            return true;
    }
    return false;
}

/**
 * @brief Constructor.
 * @param height the size of the map
 * @param width the size of the map
 * @return none
 */

 
DStarMap::DStarMap(MapManager &mm, ROBOT::TYPE robot_type) : mm(mm), robot_type(robot_type) {
    std::vector<std::vector<DStarCell>> new_grid(
        mm.h,
        std::vector<DStarCell>(mm.w, DStarCell(g_infinity_cost)));
    grid = new_grid;
    map_size = std::make_pair(mm.h, mm.w);
}

/**
 * @brief Add obstacles and change cells's status.
 * @param obstacle a set of obstackes's position
 * @param hidden_obstacle a set of hidden obstackes's position
 * @return none
 */
void DStarMap::AddObstacle(pair<int, int> obstacle) {
    grid.at(obstacle.first).at(obstacle.second).UpdateStatus(obstacle_mark);
}

//y,x
void DStarMap::SetStart(const std::pair<int, int> &new_start) {
    start = new_start;
    km += Heuristic(new_start, goal);
   // UpdateCellStatus(start, start_mark);
}

/**
 * @brief Set the goal and change cells's status.
 * @param new_goal the position of the goal
 * @return none
 */
void DStarMap::SetGoal(const std::pair<int, int> &new_goal) {
    goal = new_goal;
    UpdateCellStatus(new_goal, goal_mark);
}



/**
 * @brief Get the goal's position.
 * @return the position of the goal
 */
std::pair<int, int> DStarMap::GetGoal() const { return goal; }
std::pair<int, int> DStarMap::GetStart() const { return start; }

/**
 * @brief Get the g-value of the cell with given position.
 * @param position the position of of the cell
 * @return cell's g-value
 */
int DStarMap::CurrentCellG(const std::pair<int, int> &position) const {
    return grid.at(position.first).at(position.second).CurrentG();
}

/**
 * @brief Get the rhs-value of the cell with given position.
 * @param position the position of of the cell
 * @return cell's rhs-value
 */
int DStarMap::CurrentCellRhs(const std::pair<int, int> &position) const {
    return grid.at(position.first).at(position.second).CurrentRhs();
}

/**
 * @brief Caculat the key (priority in search) of the cell with given position.
 * @param position the position of of the cell
 * @return the key value, which is the priority in next search
 */


int DStarMap::Heuristic(const std::pair<int, int> & s1, const std::pair<int, int> & s2) const {
     return sqrt(pow((s1.first-s2.first),2) + pow((s1.second-s2.second),2));
}

Key DStarMap::CalculateCellKey(const std::pair<int, int> &position) const {
    int h = Heuristic(this->start, this->goal);
    int key2 = std::min(CurrentCellG(position), CurrentCellRhs(position));
    int key1 = key2+h+km;
    return Key(key1, key2); //TODO
}

/**
 * @brief Get the status of the cell with given position.
 * @param position the position of of the cell
 * @return cell's status
 */
std::string DStarMap::CurrentCellStatus(const std::pair<int, int> &position) const {
    return grid.at(position.first).at(position.second).CurrentStatus();
}

/**
 * @brief Set the g-value of the cell with given position.
 * @param position the position of of the cell
 * @param new_g new estamated distance to the goal
 * @return none
 */
void DStarMap::UpdateCellG(const std::pair<int, int> &position,
                      const int &new_g) {
    grid.at(position.first).at(position.second).UpdateG(new_g);
}

/**
 * @brief Set the rhs-value of the cell with given position.
 * @param position the position of of the cell
 * @param new_rhs one step lookahead values based on the g-values
 * @return none
 */
void DStarMap::UpdateCellRhs(const std::pair<int, int> &position,
                        const int &new_rhs) {
    grid.at(position.first).at(position.second).UpdateRhs(new_rhs);
}

/**
 * @brief Set the status of the cell with given position.
 * @param position the position of of the cell
 * @param new_status a mark that represent the new status of the cell
 * @return none
 */
void DStarMap::UpdateCellStatus(const std::pair<int, int> &position,
                           const std::string &new_status) {
    grid.at(position.first).at(position.second).UpdateStatus(new_status);
}

/**
 * @brief Set the g-value to infinity of the cell with given position.
 * @param position the position of of the cell
 * @return none
 */
void DStarMap::SetInfiityCellG(const std::pair<int, int> &position) {
    UpdateCellG(position, g_infinity_cost);
}

/**
 * @brief Compust the cost of from current node to next node.
 * @param current_position current position of of the cell
 * @param next_position next position of of the cell
 * @return cost to travel
 */

uint32_t DStarMap::ComputeCost(const std::pair<int, int> &current_position,
                        const std::pair<int, int> &next_position) {
    //printf("Availability(y:%d, x:%d) - %d\n", next_position.first, next_position.second, Availability(next_position));
    if (!Availability(next_position)) return g_infinity_cost;
    //printf("\t- cost_at(y:%d, x:%d) - %d\n", next_position.first, next_position.second, this->mm.cost_at({next_position.second, next_position.first}, this->robot_type));
    return this->mm.cost_at({next_position.second, next_position.first}, this->robot_type);
}


/**
 * @brief Find eight neighbos that are reachable: not a obstacle nor outside.
 * @param position current position of of the cell
 * @return a set of eight neighbors that are reachable
 */
std::vector<std::pair<int, int>> DStarMap::FindNeighbors(
    const std::pair<int, int> & position) {
    std::vector<std::pair<int, int>> neighbors = {};
    std::vector<std::pair<int, int>> search_neighbor = {
        {1,0}, {-1,0}, {0,1},{0,-1}
    };

    for (auto const &i : search_neighbor) {
        auto neighbor = std::make_pair(position.first + i.first,
                                        position.second + i.second);
        auto cost = ComputeCost(position, neighbor);
        if (cost != g_infinity_cost) // TODO
            neighbors.push_back(neighbor);
    }
    return neighbors;
}

/**
 * @brief Check if the node is out scope of map
 * @param position the position of next position
 * @return true if accessible and flase if not 
 */
bool DStarMap::Availability(const std::pair<int, int> & position) {
    if (position.first < 0 || position.first >= map_size.first) return false;
    if (position.second < 0 || position.second >= map_size.second) return false;
    return true;
}

/**
 *
 * @brief Visualize all g-values and rhs-values in the map on the terminal.
 *  
 */
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

/**
 *
 * @brief Visualize the map and the path the robot has traveled on the terminal.
 *  
 */
void DStarMap::PrintResult() {
    std::vector<std::string> lines(map_size.second, "----");
    std::cout << "Result: " << std::endl
        << "start: " << start_mark << " goal: " << goal_mark << " robot: "
        << robot_mark << " obstacle: " << obstacle_mark << " unknown: "
        << unknown_mark << std::endl;
    std::cout << " -";
    for (auto line : lines) std::cout << line;
    std::cout << std::endl;
    for (auto const &row : grid) {
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


class DStarImpl {
    public:
    DStarMap map;
    ROBOT::TYPE robot_type;

    DStarImpl(MapManager &mm, ROBOT::TYPE robot_type): map(DStarMap(mm, robot_type)), robot_type(robot_type) { 
    }
    /**
     * @brief Initialize the map and the open list
     * @param this->map the pointer of the map
     * @param DStaropenlist_ptr the pointer of the open list
     * @return none
     */
    void Initialize() {
        // One lookahead cost of the goal must be zero
        auto goal_rhs = 0.0;
        this->map.UpdateCellRhs(this->map.GetGoal(), goal_rhs);
        // Insert the goal to open list
        auto new_key = this->map.CalculateCellKey(this->map.GetGoal());
        this->openlist.Insert(new_key, this->map.GetGoal());
    }

    /**
     * @brief Compute the shortest path
     * @param robot the robot
     * @param this->map the pointer of the map
     * @param openlist_ptr the pointer of the open list
     * @return none
     */
    void ComputeShortestPath() {
        pair<int, int> robot_pos = this->map.GetStart();
        this->map.PrintResult();
        //this->map.PrintValue();

        //TODO: If this->openlist is empty, invalid heap-read occurs.
        while (this->openlist.Top().first <
            this->map.CalculateCellKey(robot_pos) ||
            this->map.CurrentCellRhs(robot_pos) !=
            this->map.CurrentCellG(robot_pos)) {
            //this->map.PrintValue();

            auto key_and_node = this->openlist.Pop();
            auto node = key_and_node.second;

            auto old_key = key_and_node.first;
            auto new_key = this->map.CalculateCellKey(node);

            if (old_key < new_key) {
                this->openlist.Insert(new_key, node);
            } else if (this->map.CurrentCellG(node) >
                    this->map.CurrentCellRhs(node)) {
                this->map.UpdateCellG(node, this->map.CurrentCellRhs(node));
                for (auto const &vertex : this->map.FindNeighbors(node)) {
                    UpdateVertex(vertex);
                }
            } else {
                this->map.SetInfiityCellG(node);
                UpdateVertex(node);
                for (auto const &vertex : this->map.FindNeighbors(node)) {
                    UpdateVertex(vertex);
                }
            }
        }
        
        // Show the new computed shortest path.
    }

    /**
     * @brief Update node of interest
     * @param vertex the position of the node
     * @param this->map the pointer of the map
     * @param DStaropenlist_ptr the pointer of the open list
     * @return none
     */
    void UpdateVertex(const std::pair<int, int> &vertex) {
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

    /**
     * @brief Find the numimum rhs of amoung node's neighbors.
     * @param vertex the position of the node
     * @param this->map the pointer of the map
     * @return minimum rhs 
     */
    int ComputeMinRhs(const std::pair<int, int> &vertex) {
        int min_rhs = g_infinity_cost;
        auto neibors = this->map.FindNeighbors(vertex);
        for (auto const &next_vertex : neibors) {
            uint32_t cost = this->map.ComputeCost(vertex, next_vertex);
            auto temp_rhs = cost + this->map.CurrentCellG(next_vertex);
            if (temp_rhs < min_rhs) min_rhs = temp_rhs;
        }
        return min_rhs;
    }

    /**
     * @brief Find next position with minimum g-value plus travel cost
     * @param current_position the position of the current node
     * @param this->map the pointer of the map
     * @return next position in the shortest path
     */
    std::pair<int, int> ComputeNextPotision(
                        const std::pair<int, int> &current_position) {
        auto next_position = current_position;
        int cheaest_cost = g_infinity_cost;
        for (auto const &candidate : this->map.FindNeighbors(current_position)) {
            auto cost = map.ComputeCost(current_position, candidate) +
                    map.CurrentCellG(candidate);
            if (cost < cheaest_cost) {
                cheaest_cost = cost;
                next_position = candidate;
            }
        }
        return next_position;
    }

    void AddObstacle(std::pair<int, int> obstacle_pos) {
        this->map.AddObstacle(obstacle_pos);
        // Update node's status
        UpdateVertex(obstacle_pos);

        for (auto const &candidate_neighbor :
                            this->map.FindNeighbors(obstacle_pos)) {
            UpdateVertex(candidate_neighbor);
        }
        this->map.UpdateCellRhs(obstacle_pos, g_infinity_cost);
        this->map.UpdateCellG(obstacle_pos, g_infinity_cost);
    }

    private:
    DStarOpenList openlist;
};

/**
 * @brief Update hidden obstacle
 * @param current_position robot's current position
 * @param this->map the pointer of the map
 * @param DStaropenlist_ptr the pointer of the open list
 * @return if there are hidden obstacle around
 */


class TaskDstarLite {
    public:
    TaskDstarLite(int x, int y, MapManager &mm): task_x(x), task_y(y), mm(mm) {
        dstars_map[ROBOT::TYPE::CATERPILLAR] = make_unique<DStarImpl>(mm, ROBOT::TYPE::CATERPILLAR);
        dstars_map[ROBOT::TYPE::WHEEL] = make_unique<DStarImpl>(mm, ROBOT::TYPE::WHEEL);

        // Set the task position and initialie DStar
        dstars_map[ROBOT::TYPE::CATERPILLAR]->map.SetGoal(make_pair(task_y, task_x));
        dstars_map[ROBOT::TYPE::CATERPILLAR]->Initialize();
        dstars_map[ROBOT::TYPE::WHEEL]->map.SetGoal(make_pair(task_y, task_x));
        dstars_map[ROBOT::TYPE::WHEEL]->Initialize();

        // Conduct path plannign
       update_all_walls();
       replanning();
    }
    
    void replanning() {
        //Update Map(Wall) latest nformation
        update_latest_walls();    
        dstars_map[ROBOT::TYPE::WHEEL]->map.SetStart({5,3});
        dstars_map[ROBOT::TYPE::WHEEL]->ComputeShortestPath();
        dstars_map[ROBOT::TYPE::WHEEL]->map.PrintResult();
        //dstars_map[ROBOT::TYPE::WHEEL]->map.PrintValue();
        auto robot_pos = dstars_map[ROBOT::TYPE::WHEEL]->map.GetStart();
        while (robot_pos != dstars_map[ROBOT::TYPE::WHEEL]->map.GetGoal()) {
            robot_pos = dstars_map[ROBOT::TYPE::WHEEL]->ComputeNextPotision(robot_pos);
            cout << "(y: " << robot_pos.first << ", x: " << robot_pos.second << "),";
        }
        cout << endl;

    }

    int calculate_cost(ROBOT robot, vector<Coord>& RtoT) {
        //TODO: Calculate costs of each ROBOT::Type(CATERPILLAR, WHEEL), Local update only on the updated_walls
        return 10;
    }
    private:
    void update_wall(pair<int, int> pos) {
        dstars_map[ROBOT::TYPE::WHEEL]->AddObstacle(pos);
        dstars_map[ROBOT::TYPE::CATERPILLAR]->AddObstacle(pos);
    }
    void update_all_walls() {
        for(Coord v : mm.objects[OBJECT::WALL])
            update_wall({v.y, v.x});
    }

    void update_latest_walls() {
        for(Coord v : mm.latest_observed_coords) {
            if(mm.object_map[v.x][v.y] == OBJECT::WALL) {
                update_wall({v.y, v.x});
            }   
        }
    }

    map<ROBOT::TYPE, unique_ptr<DStarImpl>> dstars_map;
    int task_x, task_y;
    int is_initialized;
    MapManager &mm;
};
map<int, TaskDstarLite> tasks_dstar;


MapManager map_manager;

// 매크로 선언 (필요 시 컴파일 시 -Dgravity_mode 추가)
//#define gravity_mode

static map<int, vector<vector<int>>> last_seen_time;
vector<queue<vector<Coord>>> robot_task;

void Scheduler::on_info_updated(const set<Coord> &observed_coords,
                                const set<Coord> &updated_coords,
                                const vector<vector<vector<int>>> &known_cost_map,
                                const vector<vector<OBJECT>> &known_object_map,
                                const vector<shared_ptr<TASK>> &active_tasks,
                                const vector<shared_ptr<ROBOT>> &robots)
{

    map_manager.tick += 1;

    //Replanning
    int new_observed_coords = map_manager.update_map_info(known_object_map, known_cost_map, observed_coords, updated_coords);
    int is_replanning_needed = new_observed_coords > 0;

    for(auto task : active_tasks) {
        if(tasks_dstar.find(task->id) == tasks_dstar.end()) {
            tasks_dstar.insert(make_pair(task->id, TaskDstarLite(task->coord.x, task->coord.y, map_manager)));
        } else if(is_replanning_needed) {
            //None of coords found newly, not need to conduct replanning.
            if(task->id == 8)
                tasks_dstar.at(task->id).replanning();
        }
    }
    
    if(is_replanning_needed && active_tasks.size()>0){
    //TODO: MCMF
        printf("mcmf part starts\n");
        vector<vector<int>> distRT, distTT, robotPath;
        vector<vector<vector<Coord>>> RtoT;
        distRT = vector<vector<int>>(robots.size(), vector<int>(tasks_dstar.size()));
        distTT = vector<vector<int>>(tasks_dstar.size(), vector<int>(tasks_dstar.size()));
        RtoT = vector<vector<vector<Coord>>>(robots.size(),vector<vector<Coord>>(tasks_dstar.size()));
        robotPath.resize(robots.size());
        //printf("resize done\n");
        int i=0, j=0;
        for(auto& [task_id,task] : tasks_dstar){
            j=0;
            for(const auto& robotPtr : robots){
                distRT[j][i] = task.calculate_cost(*robotPtr, RtoT[j][i]);
                j++;
            }
            i++;
        }
        printf("call assign_tasks_mcmf\n");
        assign_tasks_mcmf(distRT, distTT, robotPath);

        robot_task.clear();
        robot_task.resize(robots.size());
        for(int i=0; i<robots.size(); i++){
            for(int j=0; j<robotPath[i].size(); j++){
                robot_task[i].push(RtoT[i][robotPath[i][j]]);
            }
        }
        printf("robot_task created\n");
    }

    // int map_size = static_cast<int>(known_object_map.size());
    // for (int id = 0; id < static_cast<int>(robots.size()); ++id) {
    //     if (last_seen_time.count(id) == 0) {
    //         last_seen_time[id] = vector<vector<int>>(map_size, vector<int>(map_size, -1));
    //     }
    //     for (const auto& coord : updated_coords) {
    //         if (coord.x >= 0 && coord.x < map_size && coord.y >= 0 && coord.y < map_size) {
    //             last_seen_time[id][coord.x][coord.y] = map_manager.tick;
    //         }
    //     }
    // }
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
                for (int j = 3; j <= 4; ++j) // 기존 1~2 → 3~4로 수정
                {
                    Coord check;
                    if (dir == 0)       check = {curr.x + i, curr.y + j}; // UP
                    else if (dir == 1)  check = {curr.x + i, curr.y - j}; // DOWN
                    else if (dir == 2)  check = {curr.x - j, curr.y + i}; // LEFT
                    else                check = {curr.x + j, curr.y + i}; // RIGHT

                    if (check.x < 0 || check.x >= map_size || check.y < 0 || check.y >= map_size)
                        continue;

                    if (known_object_map[check.x][check.y] == OBJECT::UNKNOWN)
                        cnt++;
                }
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
        int current_tick = map_manager.tick; // tick 사용
        for (int x = 0; x < map_size; ++x) {
            for (int y = 0; y < map_size; ++y) {
                if (known_object_map[x][y] == OBJECT::WALL) continue;
                int weight = (last_seen_time[id][x][y] == -1) ? current_tick : (current_tick - last_seen_time[id][x][y]);
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
        // 중력 모드가 아닐 때: 무작위 이동이 아니라, 맵의 중앙으로 향하는 방향 선택
        int mid_x = map_size / 2;
        int mid_y = map_size / 2;
        int dx = mid_x - curr.x;
        int dy = mid_y - curr.y;

        vector<ROBOT::ACTION> dir_priority;
        if (abs(dx) > abs(dy)) {
            if (dx > 0) dir_priority.push_back(ROBOT::ACTION::RIGHT);
            else if (dx < 0) dir_priority.push_back(ROBOT::ACTION::LEFT);
            if (dy > 0) dir_priority.push_back(ROBOT::ACTION::UP);
            else if (dy < 0) dir_priority.push_back(ROBOT::ACTION::DOWN);
        } else {
            if (dy > 0) dir_priority.push_back(ROBOT::ACTION::UP);
            else if (dy < 0) dir_priority.push_back(ROBOT::ACTION::DOWN);
            if (dx > 0) dir_priority.push_back(ROBOT::ACTION::RIGHT);
            else if (dx < 0) dir_priority.push_back(ROBOT::ACTION::LEFT);
        }

        // 모든 방향을 넣어서 fallback 시에도 반드시 이동하도록
        for (int i = 0; i < 4; ++i)
        {
            if (std::find(dir_priority.begin(), dir_priority.end(), static_cast<ROBOT::ACTION>(i)) == dir_priority.end())
                dir_priority.push_back(static_cast<ROBOT::ACTION>(i));
        }

        for (ROBOT::ACTION dir : dir_priority) {
            Coord next = curr + directions[static_cast<int>(dir)];
            if (next.x < 0 || next.x >= map_size || next.y < 0 || next.y >= map_size)
                continue;
            if (known_object_map[next.x][next.y] == OBJECT::WALL)
                continue;
            cout << "[Drone " << id << "] DFS complete, center-seeking or fallback → dir " << static_cast<int>(dir) << endl;
            return dir;
        }

        cout << "[Drone " << id << "] DFS complete, all directions blocked → HOLD" << endl;
        return ROBOT::ACTION::HOLD;
#endif
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

    printf("R=%d T=%d\n",R,T);
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
        mcmf.addEdge(SRC, ROBOT_BEG + r, T, 0);

    //  robot → 첫 task layer (cap=T, cost=dist)
    for (int r = 0; r < R; ++r){
        for (int t = 0; t < T; ++t){
            mcmf.addEdge(ROBOT_BEG + r,
                         TASK_BEG + /*layer0*/ t,
                         T,
                         distRT[r][t]);
            //printf("add %d %d %d %d\n",ROBOT_BEG + r,TASK_BEG + /*layer0*/ t,T,distRT[r][t]);
        }
    }

    printf("hmm\n");

    //  task layer k → k+1 (cap=T‑(k+1), cost=distTT)
    for (int k = 0; k < T - 1; ++k) {
        int capHere = T - (k + 1);             // 3,2,1,…
        int fromBeg = TASK_BEG + k * T;
        int toBeg   = TASK_BEG + (k + 1) * T;

        for (int i = 0; i < T; ++i) //TODO: Mismatch the size of distTT
            for (int j = 0; j < T; ++j)
                mcmf.addEdge(fromBeg + i,
                             toBeg   + j,
                             capHere,
                             distTT[i][j]);
    }

    printf("hmm\n");

    //  마지막 task layer → ID (cap=1, cost=0)
    int lastBeg = TASK_BEG + (T - 1) * T;
    for (int t = 0; t < T; ++t)
        mcmf.addEdge(lastBeg + t, ID_BEG + t, 1, 0);

    //  ID → sink (cap=1, cost=0)
    for (int t = 0; t < T; ++t)
        mcmf.addEdge(ID_BEG + t, SINK, 1, 0);
    
/*    printf("before MCMF:\n");
    for(int i=0; i<mcmf.G.size(); i++){
        for(int j=0; j<mcmf.G[i].size(); j++){
            printf("%d %d %d %d\n",i,mcmf.G[i][j].to,mcmf.G[i][j].cap,mcmf.G[i][j].cost);
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

/*    printf("After mcmf:\n");
    for(int i=0; i<mcmf.G.size(); i++){
        for(int j=0; j<mcmf.G[i].size(); j++){
            printf("%d %d %d %d\n",i,mcmf.G[i][j].to,mcmf.G[i][j].cap,mcmf.G[i][j].cost);
        }
    }
*/
    //TODO: 수정
    for (int r = 0; r < R; ++r) {
        //printf("%d\n",r);
        int node = ROBOT_BEG + r;                 // 로봇 노드부터 시작
        while (true) {
            //printf("node: %d\n",node);
            bool advanced = false;
            for (const Edge& e : G[node]) {
                // forward 간선으로 실제 flow가 흘렀다면 역간선 cap > 0
                if(e.to < node) continue;
                if (G[e.to][e.rev].cap > 0) {
                    //printf("seeing %d %d %d %d\n",e.to,G[e.to][e.rev].to,G[e.to][e.rev].cap);
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
