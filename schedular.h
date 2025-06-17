#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "simulator.h"
#include <memory>
#include <vector>
#include <map>
#include <algorithm>
#include <stdio.h>
#include <cmath>

using namespace std;

#ifdef SCH_VERBOSE
#define SCH_LOG(x) do { \
   cout << x; \
} while(0)
#else
#define SCH_LOG(x) do {} while(0)
#endif



// 방향 벡터: 상, 하, 좌, 우
static const Coord DIRECTIONS[4] = {
    {0, 1},  // UP
    {0, -1}, // DOWN
    {-1, 0}, // LEFT
    {1, 0}   // RIGHT
};

enum DRONE_MODE {
    NOT_INIT, DFS, FRONTIER, WORK_DONE, GOBACK
};

/* 
  ***************************************
  ************* Map Manager *************
  ***************************************
*/

const uint32_t g_infinity_cost = 0x13371337; //Must be not negative-range of INT

class MapManager {
public:
   int tick;
   int w, h;
   vector<vector<int>> observed_map;
   vector<Coord> latest_observed_coords;
   vector<vector<OBJECT>> object_map;
   map<OBJECT, vector<Coord>> objects;
   map<ROBOT::TYPE, int> avg_costs;
   int certain_coordN;
   int observedN;

   MapManager();

   int update_map_info(vector<vector<OBJECT>>, vector<vector<vector<int>>>, set<Coord>);
   int cost_at(Coord, ROBOT::TYPE);
   double observed_pt();

private:
   vector<vector<vector<int>>> cost_map;
};



/* 
  ***************************************
  ************* D STAR LITE *************
  ***************************************
  Improved version from https://github.com/ykwang11/ENPM808X_Midterm-Project_D-Star-Lite-Path-Planning/blob/master/app/main.cpp
*/



struct Key {
public:
   int key1;
   int key2;

   Key(int key1, int key2) : key1(key1), key2(key2) {};
   bool operator<(Key const & rhs) const {
      return tie(this->key1, this->key2) < tie(rhs.key1, rhs.key2);
   } 
};

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
/**
    * @brief Constructor.
    * @param initial_num a number for the inital g-value and rhs-value
    * @return none
    */
   explicit DStarCell(const int &initial_num) {
      g = initial_num;
      rhs = initial_num;
      status = " ";
   } 
   /**
   * @brief Get current g-value.
   * @return g-value
   */
   int CurrentG() const { return g; }
   /**
    * @brief Get current rhs-value.
    * @return rhs-value
    */
   int CurrentRhs() const { return rhs; }

   /**
    * @brief Get current status: obstacle, unknown obstacle, goal, or start point.
    * @return mark that represent the status
    */
   std::string CurrentStatus() const { return status; }
   /**
    * @brief Set new g-value.
    * @param new_g new estamated distance to the goal
    * @return none
    */
   void UpdateG(const int &new_g) { g = new_g; }

   /**
    * @brief Set new rhs-value.
    * @param new_rhs one step lookahead values based on the g-values
    * @return none
    */
   void UpdateRhs(const int &new_rhs) { rhs = new_rhs; }
   /**
    * @brief Set new status.
    * @param new_status a mark represent new status
    * @return none
    */
   void UpdateStatus(const std::string &new_status) { status = new_status; }

private:
   int g = 0;
   int rhs = 0;
   std::string status = "";
};

class DStarOpenList {
public:
   /**
    * @brief Inset a node in the open list.
    * @param new_key thepriority of the node to be added
    * @param new_node a candidate node's priority in searching and its position
    * @return none
    */
   void Insert(Key, const std::pair<int, int> &);
   /**
    * @brief Update the key of node in the open list.
    * @param new_key thepriority of the node to be changed
    * @param position a candidate node's new priority in searching and its position
    * @return none
   */
   void UpdateKey(Key, const std::pair<int, int> &);
   /**
    * @brief Remove a node from the open list.
    * @param node a node's position
    * @return none
    */
   void Remove(const std::pair<int, int> &);
   /**
    * @brief Get the node on the top of the open list (a minimum heap).
    * @return the top node's priority in searching and its position
   */
   std::pair<Key, std::pair<int, int>> Top() const;
   /**
      * @brief Get the node on the top of the open list and romovee it.
      * @return the top node's priority in searching and its position
   */
   std::pair<Key, std::pair<int, int>> Pop();
   /**
    * @brief Find if a node is in the open list.
    * @return true if the node exsit and false if not
   */
   bool Find(const std::pair<int, int> &) const;
   bool Empty() const;
   
private:
   std::vector<std::tuple<Key, int, int>> priority_queue;
};



class DStarMap {
 public:
   // different costs

   // different status marks
   const std::string robot_mark = ".";
   const std::string goal_mark = "g";
   const std::string start_mark = "s";
   const std::string obstacle_mark = "x";
   const std::string unknown_mark = "?";

   // constructor and environment initializing
   /**
    * @brief Constructor.
    * @param height the size of the map
    * @param width the size of the map
    * @return none
    */
   explicit DStarMap(MapManager &, ROBOT::TYPE);
   /**
    * @brief Add obstacles and change cells's status.
    * @param obstacle a set of obstackes's position
    * @param hidden_obstacle a set of hidden obstackes's position
    * @return none
    */
   void AddObstacle(pair<int, int> obstacle) { grid.at(obstacle.first).at(obstacle.second).UpdateStatus(obstacle_mark); }
   /**
    * @brief Set the start and change cells's status.
    * @param new_start the position of the start
    * @return none
    */
   void SetStart(const std::pair<int, int> &);
   /**
    * @brief Set the goal and change cells's status.
    * @param new_goal the position of the goal
    * @return none
    */
   void SetGoal(const std::pair<int, int> &);

   // get method

   /**
    * @brief Get the goal's position.
    * @return the position of the goal
    */
   std::pair<int, int> GetGoal() const { return goal; }
   /**
    * @brief Get the start's position.
    * @return the position of the start
    */
   std::pair<int, int> GetStart() const { return start; }


   
   /**
    * @brief Get the g-value of the cell with given position.
    * @param position the position of of the cell
    * @return cell's g-value
    */
   uint32_t CurrentCellG(const std::pair<int, int> &position) const { return grid.at(position.first).at(position.second).CurrentG(); }

   /**
    * @brief Caculat the key (priority in search) of the cell with given position.
    * @param position the position of of the cell
    * @return the key value, which is the priority in next search
   */
   int Heuristic(const std::pair<int, int> & s1, const std::pair<int, int> & s2) const { return sqrt(pow((s1.first-s2.first),2) + pow((s1.second-s2.second),2)); }

   /**
    * @brief Get the rhs-value of the cell with given position.
    * @param position the position of of the cell
    * @return cell's rhs-value
   */
   uint32_t CurrentCellRhs(const std::pair<int, int> &position) const { return grid.at(position.first).at(position.second).CurrentRhs(); }

   /**
    * @brief Calculte Cell key
    * @param position the target for caculating key
    * @return Caulated Key
   */
   Key CalculateCellKey(const std::pair<int, int> &) const;


   /**
    * @brief Get the status of the cell with given position.
    * @param position the position of of the cell
    * @return cell's status
   */
   std::string CurrentCellStatus(const std::pair<int, int> &position) const { return grid.at(position.first).at(position.second).CurrentStatus(); }

   /**
    * @brief Clone grid to new grid variable
    * @param new_grid the reference variable to be copied
   */
   void clone_grid(std::vector<std::vector<DStarCell>> &new_grid) { new_grid = grid; }

   // set method

   /**
    * @brief Set the g-value of the cell with given position.
    * @param position the position of of the cell
    * @param new_g new estamated distance to the goal
    * @return none
   */
   void UpdateCellG(const std::pair<int, int> &position, const int &new_g) { grid.at(position.first).at(position.second).UpdateG(new_g); }

   /**
    * @brief Set the rhs-value of the cell with given position.
    * @param position the position of of the cell
    * @param new_rhs one step lookahead values based on the g-values
    * @return none
   */
   void UpdateCellRhs(const std::pair<int, int> &position, const int &new_rhs) { grid.at(position.first).at(position.second).UpdateRhs(new_rhs); }

   /**
   * @brief Set the status of the cell with given position.
   * @param position the position of of the cell
   * @param new_status a mark that represent the new status of the cell
   * @return none
   */
   void UpdateCellStatus(const std::pair<int, int> &position, const std::string &new_status) { grid.at(position.first).at(position.second).UpdateStatus(new_status); }

   /**
   * @brief Set the g-value to infinity of the cell with given position.
   * @param position the position of of the cell
   * @return none
   */
   void SetInfiityCellG(const std::pair<int, int> &position) { UpdateCellG(position, g_infinity_cost); }

   /**
    * @brief Compust the cost of from current node to next node.
    * @param pos next position of of the cell
    * @return cost to travel
    */
   uint32_t ComputeCost(const std::pair<int, int> &);

   /**
    * @brief Find eight neighbos that are reachable: not a obstacle nor outside.
    * @param position current position of of the cell
    * @return a set of eight neighbors that are reachable
   */
   std::vector<std::pair<int, int>> FindNeighbors(const std::pair<int, int> &);

   /**
    * @brief Check if the node is out scope of map
    * @param position the position of next position
    * @return true if accessible and flase if not 
    */
   bool Availability(const std::pair<int, int> &);

   /** 
   *  @brief Visualize the map and the path the robot has traveled on the terminal.
   */
   void PrintValue();
   void PrintResult() { PrintResult(this->grid); }
   void PrintResult(std::vector<std::vector<DStarCell>> &tgrid);

 private:
   std::pair<int, int> start;
   std::pair<int, int> goal;
   MapManager &mm;
   ROBOT::TYPE robot_type;
   int km = 0;
   std::pair<int, int> map_size;
   std::vector<std::vector<DStarCell>> grid;
};




class DStarImpl {
public:
    DStarMap map;
    ROBOT::TYPE robot_type;

    DStarImpl(MapManager &mm, ROBOT::TYPE robot_type);

    void Initialize();
    void ComputeShortestPath();
    void UpdateVertex(const std::pair<int, int> &vertex);
    int ComputeMinRhs(const std::pair<int, int> &vertex);
    std::pair<int, int> ComputeNextPotision(const std::pair<int, int> &current_position);
    void UpdateCellAndNeighbors(std::pair<int, int> pos);
    void AddObstacle(std::pair<int, int> obstacle_pos);

private:
    DStarOpenList openlist;
};




#define RUN_All_Types(code) do { \
    ROBOT::TYPE _type = ROBOT::TYPE::CATERPILLAR; \
    code \
    _type = ROBOT::TYPE::WHEEL; \
    code \
    _type = ROBOT::TYPE::DRONE; \
    code \
} while(0)

class TaskDstarLite {
public:
    TaskDstarLite(int x, int y, MapManager &mm);

    void replanning() { update_latest_coords(); }
    int calculate_cost(Coord pos, ROBOT::TYPE robot_type, std::vector<Coord> &path);

private:
    void update_object(std::pair<int, int> pos);
    void load_maps();
    void update_latest_coords();

    std::map<ROBOT::TYPE, std::unique_ptr<DStarImpl>> dstars_map;
    int task_x, task_y;
    MapManager &mm;
};



/**
 * @brief Constructor.
 * @param initial_num a number for the inital g-value and rhs-value
 * @return none
 */


/* 
  ***************************************
  ***************** MCMF ****************
  ***************************************
*/



void assign_tasks_mcmf(
        const std::vector<std::vector<int>> &distRT,
        const std::vector<std::vector<int>> &distTT,
        std::vector<std::vector<int>>& robotPath);
class Scheduler
{
public:
    void on_info_updated(const set<Coord> &observed_coords,
                         const set<Coord> &updated_coords,
                         const vector<vector<vector<int>>> &known_cost_map,
                         const vector<vector<OBJECT>> &known_object_map,
                         const vector<shared_ptr<TASK>> &active_tasks,
                         const vector<shared_ptr<ROBOT>> &robots);

    bool on_task_reached(const set<Coord> &observed_coords,
                         const set<Coord> &updated_coords,
                         const vector<vector<vector<int>>> &known_cost_map,
                         const vector<vector<OBJECT>> &known_object_map,
                         const vector<shared_ptr<TASK>> &active_tasks,
                         const vector<shared_ptr<ROBOT>> &robots,
                         const ROBOT &robot,
                         const TASK &task);

    ROBOT::ACTION idle_action(const set<Coord> &observed_coords,
                              const set<Coord> &updated_coords,
                              const vector<vector<vector<int>>> &known_cost_map,
                              const vector<vector<OBJECT>> &known_object_map,
                              const vector<shared_ptr<TASK>> &active_tasks,
                              const vector<shared_ptr<ROBOT>> &robots,
                              const ROBOT &robot);
};



#endif SCHEDULER_H_
