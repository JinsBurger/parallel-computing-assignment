#include "simulator.h"
#include "schedular.h"

int main(int argc, char *argv[])
{
    constexpr int MAP_SIZE = 30;
    constexpr int NUM_ROBOT = 6;
    constexpr int NUM_MAX_TASKS = 16;
    constexpr int NUM_INITIAL_TASKS = NUM_MAX_TASKS / 2;
    constexpr int WALL_DENSITY = 20;
    constexpr int TIME_MAX = MAP_SIZE * 100;
    constexpr int ROBOT_ENERGY = TIME_MAX * 6;
    int parse_flag = 0;
    set<Coord> observed_coords;
    set<Coord> updated_coords;

    if(argc >= 2 && string(argv[1]) == "parse") {
        parse_flag = 1;
    }

    unsigned seed;
    if(argc >= 3) {
        seed = strtol(argv[2], NULL, 16);
    } else {
        seed = static_cast<unsigned int>(time(NULL));
    }
    printf("SEED: 0%x \n", seed);
    srand(seed);
    //srand(0x29);

    TIMER timer;
    MAP map(MAP_SIZE, NUM_ROBOT, NUM_INITIAL_TASKS, NUM_MAX_TASKS, WALL_DENSITY, ROBOT_ENERGY);
    int time = -1;
    auto &robots = map.get_robots();
    auto &known_cost_map = map.get_known_cost_map();
    auto &known_object_map = map.get_known_object_map();
    auto &active_tasks = map.get_active_tasks();
    Scheduler scheduler;
    TASKDISPATCHER taskdispatcher(map, TIME_MAX);

#ifdef VERBOSE
    for (int i = 0; i < ROBOT::NUM_ROBOT_TYPE; ++i)
        map.print_cost_map(static_cast<ROBOT::TYPE>(i));
#endif // VERBOSE

    while (++time < TIME_MAX &&
           robots.size() != map.get_exhausted_robot_num() &&
           map.num_total_task != map.get_completed_task_num())
    {
        taskdispatcher.try_dispatch(time);
        observed_coords = map.observed_coord_by_robot();
        updated_coords = map.update_coords(observed_coords);

        
#ifdef VERBOSE
        cout << "Time : " << time << endl;
        map.print_object_map();
        map.print_robot_summary();
        map.print_task_summary();
#endif // VERBOSE

        timer.start();
        scheduler.on_info_updated(observed_coords,
                                  updated_coords,
                                  known_cost_map,
                                  known_object_map,
                                  active_tasks,
                                  robots);

         if(parse_flag) {
            map.print_object_map_if_changed(time);
        }
        timer.stop();
        for (auto robot : robots)
        {
            auto &status = robot->get_status();
            if (status == ROBOT::STATUS::IDLE)
            {
                auto coord = robot->get_coord();
                bool do_task = false;
                weak_ptr<TASK> task;
                if (bool(known_object_map[coord.x][coord.y] & OBJECT::TASK))
                {
                    task = map.task_at(coord);
                    timer.start();
                    do_task = scheduler.on_task_reached(observed_coords,
                                                        updated_coords,
                                                        known_cost_map,
                                                        known_object_map,
                                                        active_tasks,
                                                        robots,
                                                        *robot,
                                                        *(task.lock()));
                    timer.stop();
                }

                if (do_task)
                {
                    robot->start_working(task);
                }
                else
                {
                    timer.start();
                    ROBOT::ACTION action = scheduler.idle_action(observed_coords,
                                                                 updated_coords,
                                                                 known_cost_map,
                                                                 known_object_map,
                                                                 active_tasks,
                                                                 robots,
                                                                 *robot);
                    timer.stop();
                    robot->start_moving(action);
                }
            }

            if (status == ROBOT::STATUS::MOVING)
            {
                robot->move();
            }
            else if (status == ROBOT::STATUS::WORKING)
            {
                robot->work();
            }
        }
    }

    cout << endl;
    map.print_robot_summary();
    map.print_task_summary();
    size_t unit;
    string units[] = {"ns", "us", "ms", "s"};
    double count = static_cast<double>(timer.time_elapsed.count());
    for (unit = 0; unit < 4 && count >= 1e3; ++unit)
        count /= 1e3;
    cout << "Algorithm time : " << count << units[unit] << endl;
    printf("SEED: 0x%x \n", seed);
}
