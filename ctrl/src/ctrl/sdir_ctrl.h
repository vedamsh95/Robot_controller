#ifndef SDRI_CTRL2019_SDIR_CTRL_H
#define SDRI_CTRL2019_SDIR_CTRL_H

#include <Configuration.h>
#include <Trajectory.h>
#include "SixDPos.h"

/**
 * This is the main entry class for the sdir controller. It handles the api for computing robot specific tasks.
 */
class SdirCtrl {
public:
    /**
     * Computes a {@ref SixDPos} from a given {@ref Configuration}.
     *
     * @param config {@ref Configuration} to compute the position
     * @return {@ref SixDPos} for the passed configuration
     */
    SixDPos* get_pos_from_config(Configuration* config);

    /**
     * Computes a set of {@ref Configuration} from a given {@ref SixDPos}.
     *
     * @param pos {@ref SixDPos} to compute the position
     * @return vector containing all possible configurations for a given positions
     */
    vector<Configuration*>* get_config_from_pos(SixDPos* pos);

    /**
     * Computes a trajectory for a ptp movement from a start and target position
     *
     * NOT IMPLEMENTED SINCE IT DOES NOT GET USED!
     *
     * @param start
     * @param end
     * @return reference of a {@ref Trajectory} for the movement
     */
    Trajectory* move_robot_ptp(SixDPos* start, SixDPos* end);

    /**
     * Computes a trajectory for a ptp movement from a start and target configuration. If either the
     * start- or end-configuration is not within the joint space of the robot. The corresponding joints
     * will be set to the maximums.
     *
     * @param start start {@ref Configuration}
     * @param end   target {@ref Configuration}
     * @param sync  Whether the motion is synchronous, default = False meaning asynchronous movement
     * @return reference of a {@ref Trajectory} for the movement
     */
    Trajectory* move_robot_ptp(Configuration* start, Configuration* end, bool sync = false);

    /**
     * Computes a trajectory for a lin movement from a start and target position
     *
     * NOT IMPLEMENTED SINCE IT DOES NOT GET USED!
     *
     * @param start start {@ref SixDPos}
     * @param end   target {@ref SixDPos}
     * @return reference of a {@ref Trajectory} for the movement
     */
    Trajectory* move_robot_lin(SixDPos* start, SixDPos* end);

    /**
     * Computes a trajectory for a lin movement from a start and target configuration with a trapezoidal
     * velocity profile.
     *
     * @param start start {@ref Configuration}
     * @param end   target {@ref Configuration}
     * @param velocity The velocity at which the robot should move along the spline
     * @param acceleration The acceleration at the start and at the end of the movement
     * @param loopPoints  An empty vector that will be filled with the loop points for plotting purposes.
     *                    Use a nullptr if you do not need that functionality.
     * @return reference of a {@ref Trajectory} for the movement
     *         NOTE: In case the movement could not be finished, a zero configuration got added to the trajectory!
     */
    Trajectory* move_robot_lin(Configuration* start, Configuration* end, double velocity, double acceleration, std::vector<std::vector<SixDPos*>>* loopPoints);

    /**
     * Computes a spline that starts at the first given point and ends
     * at the last one while passing through the other ones in the given order.
     * A trapezoidal velocity profile is used together with a fixed orientation.
     *
     * @param points The points the robot should move to using a spline
     * @param start The start configuration of the robot
     * @param velocity The velocity at which the robot should move along the spline
     * @param acceleration The acceleration at the start and at the end of the movement
     * @param loopPoints  An empty vector that will be filled with the loop points for plotting purposes.
     *                    Use a nullptr if you do not need that functionality.
     * @param elong The scalar elongation factor for calculating the splines
     * @param type The type of spline to compute:
     *                  0: Cubic
     *                  1: Quintic
     * @return reference of a {@ref Trajectory} for the movement
     *         NOTE: In case the movement could not be finished, a zero configuration got added to the trajectory!
     */
    Trajectory* move_robot_spline(vector<SixDPos*> &points, Configuration * start, double velocity, double acceleration, std::vector<std::vector<SixDPos*>>* loopPoints, double _elong = 0.5, int _spline_type = 0);
};


#endif //SDRI_CTRL2019_SDIR_CTRL_H
