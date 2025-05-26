#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "simulator.h"
#include <memory>
#include <vector>
#include <map>
#include <algorithm>

using namespace std;


const uint32_t g_infinity_cost = 0x13371337; //Must be not negative-range of INT

class MapManager {
    public:
    int tick;
    int w, h;
    vector<vector<int>> observed_map;
    vector<Coord> latest_observed_coords;
    vector<vector<OBJECT>> object_map;
    map<OBJECT, vector<Coord>> objects;
    vector<vector<vector<int>>> cost_map;
    map<ROBOT::TYPE, int> avg_costs;
    int certain_coordN;

    MapManager();

    int update_map_info(vector<vector<OBJECT>>, vector<vector<vector<int>>>, set<Coord>, set<Coord>);
    int cost_at(Coord, ROBOT::TYPE);
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
\
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
    explicit DStarCell(const int &);
    int CurrentG() const;
    int CurrentRhs() const;
    std::string CurrentStatus() const;
    void UpdateG(const int &);
    void UpdateRhs(const int &);
    void UpdateStatus(const std::string &);

 private:
    int g = 0;
    int rhs = 0;
    std::string status = "";
};

class DStarOpenList {
 public:
    void Insert(Key, const std::pair<int, int> &);
    void UpdateKey(Key, const std::pair<int, int> &);
    void Remove(const std::pair<int, int> &);
    std::pair<Key, std::pair<int, int>> Top() const;
    std::pair<Key, std::pair<int, int>> Pop();
    bool Find(const std::pair<int, int> &) const;
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
    explicit DStarMap(MapManager &, ROBOT::TYPE);
    void AddObstacle(pair<int, int> );
    void SetStart(const std::pair<int, int> &);
    void SetGoal(const std::pair<int, int> &);

    // get method
    std::pair<int, int> GetGoal() const;
    std::pair<int, int> GetStart() const;
    int CurrentCellG(const std::pair<int, int> &) const;
    int Heuristic(const std::pair<int, int> &, const std::pair<int, int> &) const;
    int CurrentCellRhs(const std::pair<int, int> &) const;
    Key CalculateCellKey(const std::pair<int, int> &) const;
    std::string CurrentCellStatus(const std::pair<int, int> &) const;

    // set method
    void UpdateCellG(const std::pair<int, int> &, const int &);
    void UpdateCellRhs(const std::pair<int, int> &, const int &);
    void UpdateCellStatus(const std::pair<int, int> &, const std::string &);
    void SetInfiityCellG(const std::pair<int, int> &);

    uint32_t ComputeCost(const std::pair<int, int> &,
                       const std::pair<int, int> &);

    std::vector<std::pair<int, int>> FindNeighbors(const std::pair<int, int> &);
    bool Availability(const std::pair<int, int> &);

    // print method
    void PrintValue();
    void PrintResult();

 private:
    std::pair<int, int> start;
    std::pair<int, int> goal;
    MapManager &mm;
    ROBOT::TYPE robot_type;
    int km = 0;
    std::pair<int, int> map_size;
    std::vector<std::vector<DStarCell>> grid;
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