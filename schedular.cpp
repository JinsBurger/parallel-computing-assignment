#include "schedular.h"
#include "simulator.h"
#include <stack>
#include <random>
#include <map>
#include <vector>
#include <queue>
#include <limits>


using namespace std;


class MapManager {
    public:
        int tick;
        int w, h;
        MapManager(): tick(0) {}

        int update_map_info(vector<vector<OBJECT>> object_map, vector<vector<vector<int>>> cost_map, set<Coord> observed_coords, set<Coord> updated_coords) {
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
                }
                this->observed_map[coord.x][coord.y] = tick;
                return this->latest_observed_coords.size();
            }
        }

    private:
    vector<vector<int>> observed_map;
    vector<Coord> latest_observed_coords;
    vector<vector<OBJECT>> object_map;
    vector<vector<vector<int>>> cost_map;
};


/* 
  ***************************************
  ************* D STAR LITE *************
  ***************************************
*/


class DStarRobot {
 public:
    explicit DStarRobot(const std::pair<int, int> &);
    std::pair<int, int> CurrentPosition() const;
    void Move(const std::pair<int, int> &);
 private:
    std::pair<int, int> position;
};



class DStarCell{
 public:
    explicit DStarCell(const double &);
    double CurrentG() const;
    double CurrentRhs() const;
    std::string CurrentStatus() const;
    void UpdateG(const double &);
    void UpdateRhs(const double &);
    void UpdateStatus(const std::string &);

 private:
    double g = 0;
    double rhs = 0;
    std::string status = "";
};

class DStarOpenList {
 public:
    void Insert(const double &, const std::pair<int, int> &);
    void UpdateKey(const double &, const std::pair<int, int> &);
    void Remove(const std::pair<int, int> &);
    std::pair<double, std::pair<int, int>> Top() const;
    std::pair<double, std::pair<int, int>> Pop();
    bool Find(const std::pair<int, int> &) const;
 private:
    std::vector<std::tuple<double, int, int>> priority_queue;
};



class DStarMap {
 public:
    // different costs
    const double infinity_cost = 100.0;
    const double diagonal_cost = 2.5;
    const double transitional_cost = 1.0;

    // different status marks
    const std::string robot_mark = ".";
    const std::string goal_mark = "g";
    const std::string start_mark = "s";
    const std::string obstacle_mark = "x";
    const std::string unknown_mark = "?";

    // constructor and environment initializing
    explicit DStarMap(const int &, const int &);
    void AddObstacle(const std::vector<std::pair<int, int>> &,
                     const std::vector<std::pair<int, int>> &);
    void SetGoal(const std::pair<int, int> &);

    // get method
    std::pair<int, int> GetGoal() const;
    double CurrentCellG(const std::pair<int, int> &) const;
    double CurrentCellRhs(const std::pair<int, int> &) const;
    double CalculateCellKey(const std::pair<int, int> &) const;
    std::string CurrentCellStatus(const std::pair<int, int> &) const;

    // set method
    void UpdateCellG(const std::pair<int, int> &, const double &);
    void UpdateCellRhs(const std::pair<int, int> &, const double &);
    void UpdateCellStatus(const std::pair<int, int> &, const std::string &);
    void SetInfiityCellG(const std::pair<int, int> &);

    double ComputeCost(const std::pair<int, int> &,
                       const std::pair<int, int> &);

    std::vector<std::pair<int, int>> FindNeighbors(const std::pair<int, int> &);
    bool Availability(const std::pair<int, int> &);

    // print method
    void PrintValue();
    void PrintResult();

 private:
    std::pair<int, int> map_size;
    std::vector<std::vector<DStarCell>> grid;
    std::pair<int, int> goal;
};




/**
 * @brief Constructor
 * @param start_point the start point of the robot
 * @return none
 */
DStarRobot::DStarRobot(const std::pair<int, int> &start_point) { position = start_point; }

/**
 * @brief Get current position.
 * @return the position of the robot
 */
std::pair<int, int> DStarRobot::CurrentPosition() const { return position; }

/**
 * @brief Move the robot.
 * @param next_position next position
 * @return none
 */
void DStarRobot::Move(const std::pair<int, int> &next_position) {
    position = next_position;
}






/**
 * @brief Constructor.
 * @param initial_num a number for the inital g-value and rhs-value
 * @return none
 */
DStarCell::DStarCell(const double &initial_num) {
    g = initial_num;
    rhs = initial_num;
    status = " ";
}

/**
 * @brief Get current g-value.
 * @return g-value
 */
double DStarCell::CurrentG() const { return g; }

/**
 * @brief Get current rhs-value.
 * @return rhs-value
 */
double DStarCell::CurrentRhs() const { return rhs; }

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
void DStarCell::UpdateG(const double &new_g) { g = new_g; }

/**
 * @brief Set new rhs-value.
 * @param new_rhs one step lookahead values based on the g-values
 * @return none
 */
void DStarCell::UpdateRhs(const double &new_rhs) { rhs = new_rhs; }

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
void DStarOpenList::Insert(const double &new_key,
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
void DStarOpenList::UpdateKey(const double &new_key,
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
    double min_key = -1.0;
    UpdateKey(min_key, node);
    std::pop_heap(priority_queue.begin(),
                  priority_queue.end(), std::greater<>());
    priority_queue.pop_back();
}

/**
 * @brief Get the node on the top of the open list (a minimum heap).
 * @return the top node's priority in searching and its position
 */
std::pair<double, std::pair<int, int>> DStarOpenList::Top() const {
    auto key = std::get<0>(priority_queue.front());
    auto position = std::make_pair(std::get<1>(priority_queue.front()),
                                   std::get<2>(priority_queue.front()));
    return std::make_pair(key, position);
}

/**
 * @brief Get the node on the top of the open list and romovee it.
 * @return the top node's priority in searching and its position
 */
std::pair<double, std::pair<int, int>> DStarOpenList::Pop() {
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
DStarMap::DStarMap(const int &height, const int &width) {
    std::vector<std::vector<DStarCell>> new_grid(
        height,
        std::vector<DStarCell>(width, DStarCell(infinity_cost)));
    grid = new_grid;
    map_size = std::make_pair(height, width);
}

/**
 * @brief Add obstacles and change cells's status.
 * @param obstacle a set of obstackes's position
 * @param hidden_obstacle a set of hidden obstackes's position
 * @return none
 */
void DStarMap::AddObstacle(const std::vector<std::pair<int, int>> &obstacle,
                      const std::vector<std::pair<int, int>> &hidden_obstacle) {
    for (auto const &node : obstacle)
        grid.at(node.first).at(node.second).UpdateStatus(obstacle_mark);
    for (auto const &node : hidden_obstacle)
        grid.at(node.first).at(node.second).UpdateStatus(unknown_mark);
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

/**
 * @brief Get the g-value of the cell with given position.
 * @param position the position of of the cell
 * @return cell's g-value
 */
double DStarMap::CurrentCellG(const std::pair<int, int> &position) const {
    return grid.at(position.first).at(position.second).CurrentG();
}

/**
 * @brief Get the rhs-value of the cell with given position.
 * @param position the position of of the cell
 * @return cell's rhs-value
 */
double DStarMap::CurrentCellRhs(const std::pair<int, int> &position) const {
    return grid.at(position.first).at(position.second).CurrentRhs();
}

/**
 * @brief Caculat the key (priority in search) of the cell with given position.
 * @param position the position of of the cell
 * @return the key value, which is the priority in next search
 */
double DStarMap::CalculateCellKey(const std::pair<int, int> &position) const {
    return std::min(CurrentCellG(position), CurrentCellRhs(position));
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
                      const double &new_g) {
    grid.at(position.first).at(position.second).UpdateG(new_g);
}

/**
 * @brief Set the rhs-value of the cell with given position.
 * @param position the position of of the cell
 * @param new_rhs one step lookahead values based on the g-values
 * @return none
 */
void DStarMap::UpdateCellRhs(const std::pair<int, int> &position,
                        const double &new_rhs) {
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
    UpdateCellG(position, infinity_cost);
}

/**
 * @brief Compust the cost of from current node to next node.
 * @param current_position current position of of the cell
 * @param next_position next position of of the cell
 * @return cost to travel
 */
double DStarMap::ComputeCost(const std::pair<int, int> &current_position,
                        const std::pair<int, int> &next_position) {
    if (!Availability(next_position)) return infinity_cost;
    if (std::abs(current_position.first - next_position.first) +
        std::abs(current_position.second - next_position.second) == 1)
        return transitional_cost;
    if (std::abs(current_position.first - next_position.first) +
        std::abs(current_position.second - next_position.second) == 2)
        return diagonal_cost;
    else
        return infinity_cost;
}

/**
 * @brief Find eight neighbos that are reachable: not a obstacle nor outside.
 * @param position current position of of the cell
 * @return a set of eight neighbors that are reachable
 */
std::vector<std::pair<int, int>> DStarMap::FindNeighbors(
    const std::pair<int, int> & position) {
    std::vector<std::pair<int, int>> neighbors = {};
    std::vector<int> search_neighbor = {-1, 0, 1};
    for (auto const &i : search_neighbor) {
        for (auto const &j : search_neighbor) {
            auto neighbor = std::make_pair(position.first + i,
                                           position.second + j);
            auto cost = ComputeCost(position, neighbor);
            if (cost == transitional_cost || cost == diagonal_cost)
                neighbors.push_back(neighbor);
        }
    }
    return neighbors;
}

/**
 * @brief Check if the node is not a obstacle nor outside.
 * @param position the position of next position
 * @return true if accessible and flase if not 
 */
bool DStarMap::Availability(const std::pair<int, int> & position) {
    if (position.first < 0 || position.first >= map_size.first) return false;
    if (position.second < 0 || position.second >= map_size.second) return false;
    return grid.at(position.first).at(position.second).CurrentStatus()
           != obstacle_mark;
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



void Initialize(DStarMap *, DStarOpenList *);
void ComputeShortestPath(const DStarRobot &, DStarMap *, DStarOpenList *);
void UpdateVertex(const std::pair<int, int> &, DStarMap *, DStarOpenList *);
double ComputeMinRhs(const std::pair<int, int> &, DStarMap *);
std::pair<int, int> ComputeNextPotision(const std::pair<int, int> &, DStarMap *);
bool DetectHiddenObstacle(const std::pair<int, int> &, DStarMap *, DStarOpenList *);

/**
 * @brief Initialize the map and the open list
 * @param map_ptr the pointer of the map
 * @param DStaropenlist_ptr the pointer of the open list
 * @return none
 */
void Initialize(DStarMap* map_ptr, DStarOpenList* openlist_ptr) {
    // One lookahead cost of the goal must be zero
    auto goal_rhs = 0.0;
    map_ptr->UpdateCellRhs(map_ptr->GetGoal(), goal_rhs);
    // Insert the goal to open list
    auto new_key = map_ptr->CalculateCellKey(map_ptr->GetGoal());
    openlist_ptr->Insert(new_key, map_ptr->GetGoal());
}

/**
 * @brief Compute the shortest path
 * @param robot the robot
 * @param map_ptr the pointer of the map
 * @param openlist_ptr the pointer of the open list
 * @return none
 */
void ComputeShortestPath(const DStarRobot& robot,
                         DStarMap* map_ptr, DStarOpenList* openlist_ptr) {
    while (openlist_ptr->Top().first <
           map_ptr->CalculateCellKey(robot.CurrentPosition()) ||
           map_ptr->CurrentCellRhs(robot.CurrentPosition()) !=
           map_ptr->CurrentCellG(robot.CurrentPosition())) {
        auto key_and_node = openlist_ptr->Pop();
        auto node = key_and_node.second;

        auto old_key = key_and_node.first;
        auto new_key = map_ptr->CalculateCellKey(node);

        if (old_key < new_key) {
            openlist_ptr->Insert(new_key, node);
        } else if (map_ptr->CurrentCellG(node) >
                   map_ptr->CurrentCellRhs(node)) {
            map_ptr->UpdateCellG(node, map_ptr->CurrentCellRhs(node));
            for (auto const &vertex : map_ptr->FindNeighbors(node)) {
                UpdateVertex(vertex, map_ptr, openlist_ptr);
            }
        } else {
            map_ptr->SetInfiityCellG(node);
            UpdateVertex(node, map_ptr, openlist_ptr);
            for (auto const &vertex : map_ptr->FindNeighbors(node)) {
                UpdateVertex(vertex, map_ptr, openlist_ptr);
            }
        }
    }
    // Show the new computed shortest path.
    map_ptr->PrintValue();
    map_ptr->PrintResult();
}

/**
 * @brief Update node of interest
 * @param vertex the position of the node
 * @param map_ptr the pointer of the map
 * @param DStaropenlist_ptr the pointer of the open list
 * @return none
 */
void UpdateVertex(const std::pair<int, int> &vertex,
                  DStarMap *map_ptr, DStarOpenList *DStaropenlist_ptr) {
    if (vertex != map_ptr->GetGoal()) {
        map_ptr->UpdateCellRhs(vertex, ComputeMinRhs(vertex, map_ptr));
    }
    if (DStaropenlist_ptr->Find(vertex)) {
        DStaropenlist_ptr->Remove(vertex);
    }
    if (map_ptr->CurrentCellG(vertex) != map_ptr->CurrentCellRhs(vertex)) {
        DStaropenlist_ptr->Insert(map_ptr->CalculateCellKey(vertex), vertex);
    }
}

/**
 * @brief Find the numimum rhs of amoung node's neighbors.
 * @param vertex the position of the node
 * @param map_ptr the pointer of the map
 * @return minimum rhs 
 */
double ComputeMinRhs(const std::pair<int, int> &vertex, DStarMap *map_ptr) {
    double min_rhs = map_ptr->infinity_cost;
    auto neibors = map_ptr->FindNeighbors(vertex);
    for (auto const &next_vertex : neibors) {
        auto temp_rhs = map_ptr->ComputeCost(vertex, next_vertex) +
                        map_ptr->CurrentCellG(next_vertex);
        if (temp_rhs < min_rhs) min_rhs = temp_rhs;
    }
    return min_rhs;
}

/**
 * @brief Find next position with minimum g-value plus travel cost
 * @param current_position the position of the current node
 * @param map_ptr the pointer of the map
 * @return next position in the shortest path
 */
std::pair<int, int> ComputeNextPotision(
                    const std::pair<int, int> &current_position, DStarMap *map_ptr) {
    auto next_position = current_position;
    double cheaest_cost = map_ptr->infinity_cost;
    for (auto const &candidate : map_ptr->FindNeighbors(current_position)) {
        auto cost = map_ptr->ComputeCost(current_position, candidate) +
                    map_ptr->CurrentCellG(candidate);
        if (cost < cheaest_cost) {
            cheaest_cost = cost;
            next_position = candidate;
        }
    }
    return next_position;
}

/**
 * @brief Find hidden obstacle and recognize it a obstacle
 * @param current_position robot's current position
 * @param map_ptr the pointer of the map
 * @param DStaropenlist_ptr the pointer of the open list
 * @return if there are hidden obstacle around
 */
bool DetectHiddenObstacle(const std::pair<int, int> &current_position,
                          DStarMap *map_ptr, DStarOpenList *DStaropenlist_ptr) {
                            
    auto is_changed = false;
    for (auto const &candidate : map_ptr->FindNeighbors(current_position)) {
        if (map_ptr->CurrentCellStatus(candidate) == map_ptr->unknown_mark) {
            map_ptr->UpdateCellStatus(candidate, map_ptr->obstacle_mark);

            is_changed = true;
            // Update node's status
            UpdateVertex(candidate, map_ptr, DStaropenlist_ptr);

            for (auto const &candidate_neighbor :
                             map_ptr->FindNeighbors(candidate)) {
                UpdateVertex(candidate_neighbor, map_ptr, DStaropenlist_ptr);
            }
            map_ptr->UpdateCellRhs(candidate, map_ptr->infinity_cost);
            map_ptr->UpdateCellG(candidate, map_ptr->infinity_cost);
        }
    }
    return is_changed;
}


class TaskDstarLite {
    public:
    TaskDstarLite(int x, int y, MapManager &mm): task_x(x), task_y(y), mm(mm) {
        this->dstars_map[ROBOT::TYPE::CATERPILLAR] = make_unique<DStarMap>(mm.w, mm.h);
        this->dstars_map[ROBOT::TYPE::WHEEL] = make_unique<DStarMap>(mm.w, mm.h);
        this->replanning();
    }
    
    void replanning() {
    }

    int calculate_cost(ROBOT robot, vector<Coord>& RtoT) {
        //TODO: Calculate costs of each ROBOT::Type(CATERPILLAR, WHEEL), Local update only on the updated_walls
    }

    private:
    map<ROBOT::TYPE, unique_ptr<DStarMap>> dstars_map;
    int task_x, task_y;
    int is_initialized;
    MapManager &mm;
};
map<int, TaskDstarLite> tasks_dstar;


MapManager map_manager;

// 매크로 선언 (필요 시 컴파일 시 -Dgravity_mode 추가)
//#define gravity_mode

static map<int, vector<vector<int>>> last_seen_time;

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
            tasks_dstar.at(task->id).replanning();
        }
    }

    //TODO: MCMF
    vector<vector<int>> distRT, distTT, robotPath;
    vector<vector<vector<Coord>>> RtoT;
    distRT = vector<vector<int>>(robots.size(), vector<int>(robots.size()));
    distTT = vector<vector<int>>(active_tasks.size(), vector<int>(active_tasks.size()));
    RtoT = vector<vector<vector<Coord>>>(robots.size(),vector<vector<Coord>>(active_tasks.size()));

    int i=0, j=0;
    for(auto& [task_id,task] : tasks_dstar){
        for(const auto& robotPtr : robots){
            distRT[i][j] = task.calculate_cost(*robotPtr, RtoT[i][j]);
            j++;
        }
        i++;
    }

    assign_tasks_mcmf(distRT, distTT, robotPath);

    int map_size = static_cast<int>(known_object_map.size());
    for (int id = 0; id < static_cast<int>(robots.size()); ++id) {
        if (last_seen_time.count(id) == 0) {
            last_seen_time[id] = vector<vector<int>>(map_size, vector<int>(map_size, -1));
        }
        for (const auto& coord : updated_coords) {
            if (coord.x >= 0 && coord.x < map_size && coord.y >= 0 && coord.y < map_size) {
                last_seen_time[id][coord.x][coord.y] = map_manager.tick;
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
                int weight = (last_seen_time[id][x][y] == -1) ? map_manager.tick : (map_manager.tick - last_seen_time[id][x][y]);
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

        for (int i = 0; i < T; ++i) //TODO: Mismatch the size of distTT
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
    robotPath = vector<vector<int>>(R, vector<int>(R));

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
