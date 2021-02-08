#include "ptp.h"

// Choose whether the trajectories should be plotted (blocking).
Ptp::Ptp() :plot(true){
    robot = &Robot::getInstance();
    trajectory = new Trajectory();
}

Trajectory *Ptp::get_ptp_trajectory(Configuration *_start_cfg, Configuration *_end_cfg, bool sync) {
    // Perform the feasibility checks for the two configurations,
    // i.e. that all joint angles are within the possible range.
    // If they are not, change the end configuration to the limits
    // so that the motion can still be performed
    make_feasible(_start_cfg);
    make_feasible(_end_cfg);

    std::array<Single_trajectory *, NUM_JOINTS> single_trajectories{};

    // Determine for each joint whether a max. velocity profile
    // is sufficient, or a trapezoidal trajectory is needed.
    for (int i = 0; i < NUM_JOINTS; i++) {

        // Distance to be covered
        double diff = (*_end_cfg)[i] - (*_start_cfg)[i];

        Single_trajectory::Type type = Single_trajectory::select_type(
                diff,
                robot->velocities[i],
                robot->accelerations[i]);

        if (type == Single_trajectory::Type::MAX_VEL) {
            single_trajectories[i] = new Max_vel_trajectory(
                    robot->velocities[i],
                    robot->accelerations[i],
                    (*_start_cfg)[i],
                    (*_end_cfg)[i]);
        } else if (type == Single_trajectory::Type::TRAPEZOIDAL) {
            single_trajectories[i] = new Trapezoidal_trajectory(
                    robot->velocities[i],
                    robot->accelerations[i],
                    (*_start_cfg)[i],
                    (*_end_cfg)[i]);
        } else {
            // This should never happen since there are only those two types at the moment.
            assert(false);
        }
    }

    // Determine the total runtime
    double t_max = 0;
    int joint = 0;
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (single_trajectories[i]->get_duration() > t_max) {
            t_max = single_trajectories[i]->get_duration();
            joint = i;
        }
    }

    // In case of a synchronous movement, adjust the other joints
    if (sync) {
        for (int i = 0; i < NUM_JOINTS; i++) {
            // Skip the joint that already has the highest duration
            if (i == joint) {
                continue;
            }
            // The slowed down version of the original trajectory must always be
            // a trapezoidal trajectory, since a max-velocity trajectory cannot be
            // slowed down.
            delete single_trajectories[i];
            single_trajectories[i] = new Trapezoidal_trajectory(
                    robot->velocities[i],
                    robot->accelerations[i],
                    (*_start_cfg)[i],
                    (*_end_cfg)[i], t_max);
        }
    }

    // Sample the values
    auto tmp = static_cast<float>(t_max / robot->time_interval);
    auto cycles = static_cast<size_t>(roundf(tmp) + 3);    // Add some cycles to make sure that all
    // trajectories reach their end. This needs
    // to be done for V-REP. The precision of
    // these methods are good enough.
    vector<Configuration *> configs;
    double t = 0;
    for (size_t c = 0; c < cycles; c++) {
        std::array<double, NUM_JOINTS> joint_values{};
        for (int i = 0; i < NUM_JOINTS; i++) {
            joint_values[i] = single_trajectories[i]->eval(t);
        }
        configs.push_back(new Configuration(joint_values));
        t += robot->time_interval;
    }

    trajectory->set_trajectory(configs);

    if(plot) {
        plot_movement(configs);
    }

    for (auto tra : single_trajectories) {
        delete tra;
    }

    return trajectory;
}

void Ptp::make_feasible(Configuration *cfg) {
    for (int i = 0; i < NUM_JOINTS; i++) {
        if ((*cfg)[i] < robot->limits[i].min) {
            std::cout << "[PTP] Joint " << i + 1 << " is out of range!" << std::endl;
            (*cfg)[i] = robot->limits[i].min;
        }
        if ((*cfg)[i] > robot->limits[i].max) {
            std::cout << "[PTP] Joint " << i + 1 << " is out of range!" << std::endl;
            (*cfg)[i] = robot->limits[i].max;
        }
    }
}

void Ptp::plot_movement(vector<Configuration *> &configs) {
    // The corresponding define can be found in the cmake script. If plotting
    // is not supported on a system, it can be deactivated there to still be
    // abel to build and execute this program.
#ifdef PLOT
    if (configs.empty()) {
        std::cout << "[PTP] Cannot plot configs: Empty!" << std::endl;
        return;
    }

    // Note the start values so that in the plot all joints start at zero
    std::array<double, NUM_JOINTS> startValues{};
    for (int i = 0; i < NUM_JOINTS; i++) {
        startValues[i] = (*(configs[0]))[i];
    }

    // Scale the timeline according to the used refresh rate
    std::vector<double> x(configs.size());
    for (std::vector<double>::size_type i = 0; i < x.size(); i++) {
        x.at(i) = i * Robot::getInstance().time_interval;
    }

    // Create the individual positions for each joint
    std::array<std::vector<double>, NUM_JOINTS> positions;
    for (auto &cfg : configs) {
        for (int j = 0; j < NUM_JOINTS; j++) {
            positions.at(j).push_back(std::abs((*cfg)[j] - startValues[j]));
        }
    }

    // Create the individual velocities for each joint by calculating the differences
    std::array<std::vector<double>, NUM_JOINTS> velocities;
    for (std::vector<double>::size_type i = 0; i < x.size(); i++) {
        for (int j = 0; j < NUM_JOINTS; j++) {
            if (i == 0) {
                velocities.at(j).push_back(0);
            } else {
                velocities.at(j).push_back(std::abs(
                        ((*configs.at(i))[j] - ((*configs.at(i - 1))[j])) / Robot::getInstance().time_interval));
            }
        }

    }

    matplotlibcpp::suptitle("Trajectories of the individual joints:");
    matplotlibcpp::subplot(2, 1, 1);
    matplotlibcpp::title("Position:");

    // Plot all the joints
    for (int j = 0; j < NUM_JOINTS; j++) {
        std::ostringstream labelStream;
        labelStream << "Joint " << (j + 1);
        matplotlibcpp::named_plot(labelStream.str(), x, positions[j]);
    }

    matplotlibcpp::legend("upper left", {1.05, 1});
    matplotlibcpp::subplot(2, 1, 2);
    matplotlibcpp::title("Velocity:");

    // Plot all the joints
    for (int j = 0; j < NUM_JOINTS; j++) {
        std::ostringstream labelStream;
        labelStream << "Joint " << (j + 1);
        matplotlibcpp::named_plot(labelStream.str(), x, velocities[j]);
    }

    matplotlibcpp::legend("upper left", {1.05, 1});
    matplotlibcpp::subplots_adjust({{"hspace", 0.35},
                                    {"right",  0.75}});
    matplotlibcpp::show();
#else
    std::cout << "The plotting functionality of configurations is disabled in the cmake script!" << std::endl;
#endif

}
