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
     * @param start
     * @param end
     * @return reference of a {@ref Trajectory} for the movement
     */
    Trajectory* move_robot_ptp(SixDPos* start, SixDPos* end);

    /**
     * Computes a trajectory for a ptp movement from a start and target configuration
     *
     * @param start start {@ref Configuration}
     * @param end   target {@ref Configuration}
     * @param sync  Whether the motion is synchronous, default = False
     * @return reference of a {@ref Trajectory} for the movement
     */
    Trajectory* move_robot_ptp(Configuration* start, Configuration* end, bool sync = false);

    /**
     * Computes a trajectory for a lin movement from a start and target position
     *
     * @param start start {@ref SixDPos}
     * @param end   target {@ref SixDPos}
     * @return reference of a {@ref Trajectory} for the movement
     */
    Trajectory* move_robot_lin(SixDPos* start, SixDPos* end);

    /**
     * Computes a trajectory for a lin movement from a start and target configuration
     *
     * @param start start {@ref Configuration}
     * @param end   target {@ref Configuration}
     * @param velocity The velocity at which the robot should move along the spline
     * @param acceleration The acceleration at the start and at the end of the movement
     * @return reference of a {@ref Trajectory} for the movement
     */
    Trajectory* move_robot_lin(Configuration* start, Configuration* end, double velocity, double acceleration, std::vector<std::vector<SixDPos*>>* loopPoints);

    /**
     * Computes a spline that starts at the first given point and ends
     * at the last one while passing through the other ones in the given order.
     *
     * @param points The points the robot should move to using a spline
     * @param velocity The velocity at which the robot should move along the spline
     * @param acceleration The acceleration at the start and at the end of the movement
     * @return reference of a {@ref Trajectory} for the movement
     */
    Trajectory* move_robot_spline(vector<SixDPos*> &points, Configuration * start, double velocity, double acceleration, std::vector<std::vector<SixDPos*>>* loopPoints, double _elong = 0.5, int _spline_type = 0);
};


#endif //SDRI_CTRL2019_SDIR_CTRL_H
