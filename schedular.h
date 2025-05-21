#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "simulator.h"
#include <memory>
#include <vector>




/* 
  ***************************************
  ************* D STAR LITE *************
  ***************************************
  Reference to https://github.com/ykwang11/ENPM808X_Midterm-Project_D-Star-Lite-Path-Planning/blob/master/app/main.cpp
*/

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