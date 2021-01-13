//
// Created by Lukas on 22.12.2020.
//

#ifndef SDIR_CTRL2020_SINGLE_TRAJECTORY_H
#define SDIR_CTRL2020_SINGLE_TRAJECTORY_H

#include <Robot.h>

/**
 * This class acts as the base class for the different kinds of trajectories,
 * which can be used. Itself does not provide any functionality.
 */
class Single_trajectory {

public:

    /**
     * The different possible subclasses.
     */
    enum class Type {
        TRAPEZOIDAL,
        MAX_VEL
    };

    /**
     * Constructor - only stores the data
     *
     * @param v       Final velocity
     * @param a       Used acceleration
     * @param qi      Initial joint angle
     * @param qf      Final joint angle
     */
    Single_trajectory(double v, double a, double qi, double qf);

    virtual ~Single_trajectory() = default;

    /**
     * Evaluates the calculated Trajectory at the given time step.
     *
     * @param t Time at which the trajectory should be evaluated
     * @return  Joint angle at the given time.
     */
    virtual double eval(double t) = 0;

    /**
     * @return The total duration of the trajectory
     */
    double get_duration() const;

    /**
     * Returns which trajectory type must be used
     * @param distance    The distance that needs to be covered
     * @param maxVel      The maximal velocity
     * @param maxAcc      The given acceleration
     * @return            Trajectory type
     */
    static Type select_type(double distance, double maxVel, double maxAcc);

protected:
    /**
     * Essential trajectory variables
     */
    double ve;  // Final velocity
    double ac;  // Used acceleration
    double qi;  // Initial joint angle
    double qf;  // Final joint angle
    double tf;  // Final time / duration

    static Robot *robot;
};

#endif //SDIR_CTRL2020_SINGLE_TRAJECTORY_H
