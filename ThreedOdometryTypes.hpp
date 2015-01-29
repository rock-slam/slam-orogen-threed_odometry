#ifndef THREED_ODOMETRY_TYPES_H
#define THREED_ODOMETRY_TYPES_H

#include <vector>
#include <base/Time.hpp>
#include <base/Eigen.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace threed_odometry
{
    enum ModelType
    {
        ANALYTICAL,
        NUMERICAL
    };

    /** Configuration parameters for the dynamic weighing matrix of the motion model **/
    struct CenterOfMassConfiguration
    {
        bool dynamicOn; /** True if active dynamic weight matrix */
        base::Vector3d coordinates; /** Center of Mass position in the 2D plane on the platform w.r.t. the body center */
        base::VectorXd percentage; /** Initial percentage of the robot chain (wheels or legs) */
    };

    /** Coefficient for the IIR filter **/
    struct IIRCoefficients
    {
        bool iirOn; /** Set to true if want to use it with the following coefficients **/
        base::VectorXd feedForwardCoeff;
        base::VectorXd feedBackCoeff;
    };

    /** Data Type about robot kinematics chain status including contact points **/
    struct RobotContactPoints
    {
        base::Time time;//timestamp
        std::vector< double > modelPositions;//Robot joints and model positions values
        std::vector< int > contactPoints; //Points index in contact per each robot Tree
        std::vector< base::Matrix4d > chain;//Transformation matrix of the points wrt body
        std::vector< base::Matrix6d > cov;//Covariance matrix of the pose wrt body
    };

    /** To visualize the chain forward kinematics (end effector or contact point) **/
    struct RobotContactPointsRbs
    {
        base::Time time; //time-stamp
        std::vector< base::samples::RigidBodyState > rbsChain; //Rbs with the orientation and position of the contact point
    };
}
#endif
