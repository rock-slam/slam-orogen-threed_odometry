/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

//#define DEBUG_PRINTS 1

using namespace threed_odometry;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    /******************************************/
    /*** General Internal Storage Variables ***/
    /******************************************/

    /** Default size for the std_vector for the Cartesian 6DoF velocities variables **/
    vectorCartesianVelocities = std::vector< Eigen::Matrix <double, 6, 1> , Eigen::aligned_allocator < Eigen::Matrix <double, 6, 1> > > (2);
    vectorCartesianVelocities[0].setZero(); vectorCartesianVelocities[1].setZero();

    /***************************/
    /** Input port variables  **/
    /***************************/
    orientationSamples.orientation = Eigen::Quaterniond(base::NaN<double>()*Eigen::Vector4d::Ones());
    orientationSamples.cov_orientation = base::NaN<double>() * Eigen::Matrix3d::Ones();

    /***************************/
    /** Output port variables **/
    /***************************/
    base::samples::RigidBodyState poseInit;
    poseInit.invalidate();
    pose = poseInit;

}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

void Task::joints_samplesCallback(const base::Time &ts, const ::base::samples::Joints &joints_samples_sample)
{
    /** Two different manners to get the delta time **/
    double delta_t = static_cast<const double>(_joints_samples_period.value());
    //double delta_t = joints_samples_sample.time.toSeconds() - jointsSamples.time.toSeconds();

    /** Get the Joints values  **/
    jointsSamples = joints_samples_sample;
    jointPositions.setZero();
    jointVelocities.setZero();

    /** Mechanical joints ordered by jointsName **/
    this->joints_samplesUnpack(jointsSamples, joint_names, jointPositions, jointVelocities);

    #ifdef DEBUG_PRINTS
    std::cout<<"[THREED_ODOMETRY JOINT_SAMPLES] Received time-stamp:\n"<<jointsSamples.time.toMicroseconds()<<"\n";
    std::cout<<"[THREED_ODOMETRY JOINT_SAMPLES] Joint Positions:\n"<<jointPositions<<"\n";
    std::cout<<"[THREED_ODOMETRY JOINT_SAMPLES] Joint Velocities:\n"<<jointVelocities<<"\n";
    #endif

    #ifdef DEBUG_PRINTS
    /** Check the time difference between joint and orientation samples **/
    base::Time diffTime = jointsSamples.time - orientationSamples.time;

    std::cout<<"[THREED_ODOMETRY JOINT_SAMPLES] [ON] ("<<jointsSamples.time.toMicroseconds()<<")\n";
    std::cout<<"[THREED_ODOMETRY JOINT_SAMPLES] diffTime:\n"<<diffTime.toSeconds()<<"\n";
    #endif

    /** Perform the Motion Model and the Dead-reckoning **/
    if (orientationSamples.time.toSeconds() != 0.00)
    {
        this->updateOdometry(delta_t);
    }

}

void Task::orientation_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample)
{
    /** Two different manners to get the delta time **/
    double delta_t = static_cast<const double>(_orientation_samples_period.value());
    //double delta_t = orientation_samples_sample.time.toSeconds() - orientationSamples.time.toSeconds();

    /** Invalidate every step in order to set it from the beginning **/
    deltaPose.invalidate();

    /** Delta quaternion (rotation k-1 - rotation k) **/
    deltaPose.orientation = orientationSamples.orientation.inverse() * orientation_samples_sample.orientation; /** (T0_k-1)^-1 * T0_k **/
    deltaPose.cov_orientation = orientation_samples_sample.cov_orientation - orientationSamples.cov_orientation;

    /** Angular velocity **/
    Eigen::AngleAxisd deltaAngleaxis(deltaPose.orientation);
    Eigen::Vector3d angular_velocity = (deltaAngleaxis.angle() * deltaAngleaxis.axis())/delta_t;

    /** Fill the Cartesian Velocities **/
    cartesianVelocities.block<3,1> (0,0) = Eigen::Matrix<double, 3, 1>::Identity() * base::NaN<double>();
    cartesianVelocities.block<3,1> (3,0) = angular_velocity;//!Angular velocities come from gyros

    /** Fill the Cartesian velocity covariance **/
    cartesianVelCov.block<3,3>(3,3) = deltaPose.cov_orientation / (delta_t);// * delta_t);

    /** Get the orientation readings  **/
    orientationSamples = orientation_samples_sample;

    #ifdef DEBUG_PRINTS
    std::cout<<"[THREED_ODOMETRY INERTIAL_SAMPLES] Received time-stamp:\n"<<orientationSamples.time.toMicroseconds()<<"\n";
    std::cout<<"[THREED_ODOMETRY INERTIAL_SAMPLES] Cartesian Velocities:\n"<<cartesianVelocities<<"\n";
    #endif

    #ifdef DEBUG_PRINTS
    /** Check the time difference between orientation and joint samples **/
    base::Time diffTime = orientationSamples.time - jointsSamples.time;
    std::cout<<"[THREED_ODOMETRY INERTIAL_SAMPLES] diffTime:\n"<<diffTime.toSeconds()<<"\n";
    #endif

}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    ::base::samples::RigidBodyState poseInit;

    /************************/
    /** Read configuration **/
    /************************/
    this->urdfFile = _urdf_file.value();
    this->iirConfig = _iir_filter.value();
    this->contact_point_segments = _contact_point_segments.value();
    this->contact_angle_segments = _contact_angle_segments.value();
    this->joint_names = _joint_names.value();
    this->slip_joints = _slip_joints.value();
    this->contact_joints = _contact_joints.value();

    this->number_robot_joints =  this->joint_names.size()-this->slip_joints.size()-this->contact_joints.size();

    this->centerOfMass = _robot_CoM.value();
    this->modelType = _modelType.value();

    /** Info with regard to the Dynamic Weighting Matrix **/
    if (centerOfMass.dynamicOn)
        RTT::log(RTT::Warning)<<"[THREED_ODOMETRY] Dynamic Weight Matrix [ON]"<<RTT::endlog();
    else
        RTT::log(RTT::Warning)<<"[THREED_ODOMETRY] Dynamic Weight Matrix [OFF]"<<RTT::endlog();

    /** Info with regard to the Infinite Impulse Response Filter **/
    if (iirConfig.iirOn)
        RTT::log(RTT::Warning)<<"[THREED_ODOMETRY] Infinite Impulse Response Filter [ON]"<<RTT::endlog();
    else
        RTT::log(RTT::Warning)<<"[THREED_ODOMETRY] Infinite Impulse Response Filter [OFF]"<<RTT::endlog();

    /*********************************************/
    /** Configure the Motion Model of the Robot **/
    /*********************************************/
    if (modelType == NUMERICAL)
    {
        /** Robot Kinematics Model **/
        robotKinematics.reset(new threed_odometry::KinematicKDL (urdfFile, this->contact_point_segments,
                            this->contact_angle_segments, this->number_robot_joints, this->slip_joints.size(), this->contact_joints.size()));
        RTT::log(RTT::Warning)<<"[THREED_ODOMETRY] Numerical Model selected"<<RTT::endlog();
    }
    else if (modelType == ANALYTICAL)
    {
        /** HERE YOU NEED TO PROVIDE YOUR NUMERICAL "AD HOC" KINEMATICS MODEL **/
        RTT::log(RTT::Warning)<<"[THREED_ODOMETRY] Analytical KDL Model selected"<<RTT::endlog();
    }
    else
        throw std::runtime_error("[THREED_ODOMETRY] Invalid Odometry Model.");


    /** Create the Motion Model **/
    motionModel.reset(new threed_odometry::MotionModel<double> (this->contact_point_segments.size(), this->number_robot_joints,
                                                        this->slip_joints.size(), this->contact_joints.size()));

    /** Weighting Matrix Initialization **/
    WeightMatrix.resize(6*this->contact_point_segments.size(), 6*this->contact_point_segments.size());
    WeightMatrix.setIdentity();
    WeightMatrix =  base::NaN<double>() * WeightMatrix;

    /*************************/
    /** Motion Model Joints **/
    /*************************/
    this->motion_model_joint_names.resize(this->joint_names.size());
    bool result = this->joints_samplesMotionModel(this->motion_model_joint_names, this->joint_names, this->slip_joints, this->contact_joints);
    if (!result)
    {
        throw std::runtime_error("[THREED_ODOMETRY] Error Ordering Motion Model joints");
    }

    /****************/
    /** IIR Filter **/
    /****************/
    Eigen::Matrix <double, NORDER_BESSEL_FILTER+1, 1> besselBCoeff, besselACoeff;
    besselBCoeff = iirConfig.feedForwardCoeff;
    besselACoeff = iirConfig.feedBackCoeff;

    /** Create the Bessel Low-pass filter with the right coefficients **/
    bessel.reset(new threed_odometry::IIR<NORDER_BESSEL_FILTER, 3> (besselBCoeff, besselACoeff));

    /*******************************/
    /** Inertial Noise Covariance **/
    /*******************************/

    /** Resize the Velocity covariance **/
    modelVelCov.resize(robotKinematics->model_dof, robotKinematics->model_dof);

    /** Angular velocity coming from gyros **/
    cartesianVelCov.setZero(); modelVelCov.setZero();

    /***************************/
    /** Input port variables **/
    /***************************/
    orientationSamples.position.setZero();
    orientationSamples.cov_position.setZero();
    orientationSamples.orientation.setIdentity();
    orientationSamples.cov_orientation.setZero();

    /************************************/
    /** Resize variables for Odometry  **/
    /************************************/
    jointPositions.resize(robotKinematics->model_dof, 1);
    jointVelocities.resize(robotKinematics->model_dof, 1);

    /*************************************/
    /** Reset initial Starting position **/
    /*************************************/

    /** Set the initial world to navigation frame transform **/
    poseInit.invalidate();
    poseInit.sourceFrame = _odometry_source_frame.get();
    poseInit.targetFrame = _odometry_target_frame.get();

    /** Staring zero position **/
    poseInit.position.setZero();
    poseInit.velocity.setZero();

    /** Assume well known starting position **/
    poseInit.cov_position = Eigen::Matrix3d::Zero();
    poseInit.cov_velocity = Eigen::Matrix3d::Zero();

    #ifdef DEBUG_PRINTS
    std::cout<< "[THREED_ODOMETRY]\n";
    std::cout<< "******** Initial Position *******"<<"\n";
    std::cout<< poseInit.position<<"\n";
    std::cout<< poseInit.cov_position<<"\n";
    #endif

    /** Orientation with respect to the relative navigation frame**/
    poseInit.orientation = Eigen::Quaterniond::Identity();
    poseInit.angular_velocity.setZero();

    /** Assume very well known initial attitude **/
    poseInit.cov_orientation = Eigen::Matrix3d::Zero();
    poseInit.cov_angular_velocity = Eigen::Matrix3d::Zero();

    /** Get the Initial pose and uncertainty **/
    pose = poseInit.getTransform();
    poseCov << poseInit.cov_position, Eigen::Matrix3d::Zero(),
            Eigen::Matrix3d::Zero(), poseInit.cov_orientation;

    #ifdef DEBUG_PRINTS
    std::cout<< "[THREED_ODOMETRY]\n";
    std::cout<< "******** Initial Position *******"<<"\n";
    std::cout<< pose.translation()<<"\n";
    std::cout<< "Initial Uncertainty"<<"\n";
    std::cout<< poseInit.cov_position<<"\n";
    Eigen::Vector3d euler;
    euler[2] = poseInit.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
    euler[1] = poseInit.orientation.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
    euler[0] = poseInit.orientation.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
    std::cout<< "******** Initial Attitude *******"<<"\n";
    std::cout<< "Initial Roll: "<<euler[0]*R2D<<" Init Pitch: "<<euler[1]*R2D<<" Init Yaw: "<<euler[2]*R2D<<"\n";
    std::cout<< "Initial Uncertainty"<<"\n";
    std::cout<< poseInit.cov_orientation<<"\n";
    std::cout<< "Initial Complete Uncertainty"<<"\n";
    std::cout<< poseCov<<"\n";
    #endif

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
WeightingMatrix Task::dynamicWeightMatrix (CenterOfMassConfiguration &centerOfMass, base::Orientation &orientation)
{
    WeightingMatrix weightLocal; /** Local variable to return */
    std::vector< Eigen::Matrix<double, 3, 1> , Eigen::aligned_allocator < Eigen::Matrix<double, 3, 1> > > chainPosition; /** Chain position of contact points **/
    Eigen::Matrix<double, Eigen::Dynamic, 1> forces; /** forces to calculate */
    double theoretical_g = 9.81; /** It is not important to be exactly the real theoretical g at the location **/

    /** Set to the identity **/
    weightLocal.resize(6*this->contact_point_segments.size(), 6*this->contact_point_segments.size());
    weightLocal.setIdentity();

    /** Size the force vector **/
    forces.resize(this->contact_point_segments.size(), 1);

    if (centerOfMass.dynamicOn)
    {
        /** Resize the chainPosition **/
        chainPosition.resize(this->contact_point_segments.size());

        /** Form the chainPosition vector **/
        for (std::vector<int>::size_type i = 0; i < this->contact_point_segments.size(); ++i)
        {
            chainPosition[i] = this->fkRobotTrans[i].translation();
        }

        /** Compute the forces **/
        //exoter::BodyState::forceAnalysis(centerOfMass.coordinates, chainPosition, static_cast<Eigen::Quaterniond>(orientation), theoretical_g, forces);

        /** Compute the percentages **/
        for (register int i=0; i<static_cast<int>(this->contact_point_segments.size()); ++i)
        {
            if (forces[i] > 0.00)
                centerOfMass.percentage[i] = forces[i] / theoretical_g;
            else
                centerOfMass.percentage[i] = 0.001; //Almost zero value to this wheel.zero cannot be because there is not solution to the motion model
        }
    }

    /** Form the weighting Matrix (static or dynamic it needs to be created) **/
    for (register int i=0; i<static_cast<int>(this->contact_point_segments.size()); ++i)
    {
        weightLocal.block<6,6>(6*i, 6*i) = centerOfMass.percentage[i] * Eigen::Matrix<double, 6, 6>::Identity();
    }

    #ifdef DEBUG_PRINTS
    std::cout<<"[WEIGHT-MATRIX] forces:\n"<< forces <<"\n";
    std::cout<<"[WEIGHT-MATRIX] percentage:\n"<< centerOfMass.percentage<<"\n";
    std::cout<<"[WEIGHT-MATRIX] Weighting Matrix:\n"<< weightLocal<<"\n";
    #endif

    return weightLocal;
}

void Task::updateOdometry (const double &delta_t)
{

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> J;
    std::vector<double> vectorPositions(robotKinematics->model_dof, 0);

    /** Copy Eigen to vector **/
    Eigen::Map <Eigen::Matrix <double, Eigen::Dynamic, 1> > (&(vectorPositions[0]), robotKinematics->model_dof) = jointPositions;

    /** Update the Motion Model (Forward Kinematics and to set Contact Points) **/
    this->robotKinematics->fkSolver(vectorPositions, this->contact_point_segments, this->fkRobotTrans, this->fkRobotCov);

    /** Compute Robot Jacobian matrix **/
    J = this->robotKinematics->jacobianSolver(this->joint_names, vectorPositions);

    /** Compute dynamic Weight matrix depending on the attitude **/
    base::Pose tempPose( pose );
    this->WeightMatrix = dynamicWeightMatrix(centerOfMass, tempPose.orientation);

    /** In case of no noise in properties, set to zero to get the noise from the Motion Model **/
    if (!_orientation_samples_noise_on.value())
        cartesianVelCov.setZero();

    /** Reorganize the Jacobian matrix as required by the motion model **/
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> organized_J;
    organized_J.resize(6*this->contact_point_segments.size(), robotKinematics->model_dof);

    /** Get joints position and velocity ordered by Motion Model joint names **/
    this->joints_samplesUnpack(jointsSamples, motion_model_joint_names, jointPositions, jointVelocities);

    /** Fill the rest of jointVelocities (unknown quantities) **/
    Eigen::Matrix<double, Eigen::Dynamic, 1> Ident;
    Ident.resize(this->slip_joints.size(), 1);
    jointVelocities.block(this->number_robot_joints, 0, this->slip_joints.size(), 1) = Ident * base::NaN<double>();
    Ident.resize(this->contact_joints.size(), 1);
    jointVelocities.block(this->number_robot_joints+this->slip_joints.size(), 0, this->contact_joints.size(),1) = Ident * base::NaN<double>();

    robotKinematics->organizeJacobian(0, motion_model_joint_names, joint_names, J, organized_J);

    #ifdef DEBUG_PRINTS
    std::cout<<"** [UPDATE_ODOMETRY] JACOBIAN KDL is of size "<<organized_J.rows()<<" x "<<organized_J.cols()<<"\n"<< organized_J<<"\n\n";
    #endif

    /** Solve the navigation kinematics **/
    this->motionModel->navSolver(jointPositions, jointVelocities, organized_J, cartesianVelocities,
                                modelVelCov, cartesianVelCov, WeightMatrix);

    /** Bessel IIR Low-pass filter of the linear cartesianVelocities from the Motion Model **/
    if (iirConfig.iirOn)
    {
        Eigen::Matrix<double, 3, 1> velocity = cartesianVelocities.block<3, 1>(0,0);
        Eigen::Matrix<double, 3, 3> velocityCov = cartesianVelCov.block<3, 3>(0,0);
        cartesianVelocities.block<3, 1>(0,0) = this->bessel->perform(velocity, velocityCov, false);

        /** Store the filtered velocity uncertainty (Uncertainty propagation is
         * time-correlated by the IIR) **/
        cartesianVelCov.block<3, 3>(0,0) = velocityCov;
    }

    /** Update the Cartesian velocities on the std_vector **/
    this->vectorCartesianVelocities[1] = this->vectorCartesianVelocities[0];
    this->vectorCartesianVelocities[0] = cartesianVelocities;

    /** Complete the delta pose  (assuming constant acceleration) **/
    deltaPose.position = delta_t * ((vectorCartesianVelocities[1].block<3,1>(0,0) + vectorCartesianVelocities[0].block<3,1>(0,0))/2.0);
    deltaPose.cov_position = cartesianVelCov.block<3, 3>(0,0) * delta_t;// * delta_t;

    /** Take uncertainty on delta orientation from the motion model **/
    if (!_orientation_samples_noise_on.value())
    {
        deltaPose.cov_orientation = cartesianVelCov.block<3, 3>(3,3) * delta_t;// * delta_t;
    }

    /** Perform the velocities integration to get the pose (Dead Reckoning) **/
    Eigen::Matrix<double, 6, 6> deltaPoseCov;
    deltaPoseCov<< deltaPose.cov_position, Eigen::Matrix3d::Zero(),
            Eigen::Matrix3d::Zero(), deltaPose.cov_orientation;

    /** Dead Reckon: Propagate Pose **/
    pose = pose * deltaPose.getTransform();

    /** Adding method of propagating uncertainty **/
    poseCov = poseCov + deltaPoseCov;

    /** Guarantee SPD covariance **/
    Task::guaranteeSPD(poseCov);

    /** Out port the information **/
    this->outputPortSamples (jointPositions, cartesianVelocities, jointVelocities);

    return;
}

void Task::joints_samplesUnpack(const ::base::samples::Joints &original_joints,
                                const std::vector<std::string> &order_names,
                                Eigen::Matrix< double, Eigen::Dynamic, 1  > &joint_positions,
                                Eigen::Matrix< double, Eigen::Dynamic, 1  > &joint_velocities)
{
    assert (joint_positions.size() == original_joints.size());
    assert (joint_velocities.size() == original_joints.size());
    joint_positions.setZero();
    joint_velocities.setZero();

    register int jointIdx = 0;
    for(std::vector<std::string>::const_iterator it = order_names.begin(); it != order_names.end(); it++)
    {
        base::JointState const &state(original_joints[*it]);

        /** Avoid NaN values in position **/
        if (std::isfinite(state.position))
            joint_positions[jointIdx] = state.position;
        else
            joint_positions[jointIdx] = 0.00;

        /** Avoid NaN values in velocity **/
        if (std::isfinite(state.speed))
            joint_velocities[jointIdx] = state.speed;
        else
            throw std::runtime_error("[THREED_ODOMETRY JOINT_SAMPLES] Joint speed cannot be NaN.");

        jointIdx++;
    }
    return;
}

bool Task::joints_samplesMotionModel(std::vector<std::string> &order_names,
                                const std::vector<std::string> &joint_names,
                                const std::vector<std::string> &slip_names,
                                const std::vector<std::string> &contact_names)
{
    /** Clean order joints names **/
    order_names.clear();

    /** Get only the robot physical joints **/
    {
        bool presence;
        for(std::vector<std::string>::const_iterator it = joint_names.begin(); it != joint_names.end(); it++)
        {
            presence = false;

            /** Check in case the joint is not a slip joint **/
            std::vector<std::string>::const_iterator st = find(slip_names.begin(), slip_names.end(), *it);
            if (st != slip_names.end())
                presence = true;

            /** Check in case the joint is not a contact angle joint **/
            std::vector<std::string>::const_iterator ct = find(contact_names.begin(), contact_names.end(), *it);
            if (ct != contact_names.end())
                presence = true;

            if (!presence)
            {
                order_names.push_back(*it);
            }
        }
    }

    /** Concatenate the slip joints at the end **/
    order_names.insert(order_names.end(), slip_joints.begin(), slip_joints.end());

    /** Concatenate the contact joints at the end **/
    order_names.insert(order_names.end(), contact_joints.begin(), contact_joints.end());

    #ifdef DEBUG_PRINTS
    std::cout<<"[JOINTS_SAMPLES_MOTION_MODEL] Order Joints:\n";
    for(std::vector<std::string>::const_iterator it = order_names.begin(); it != order_names.end(); it++)
    {
        std::cout<<*it<<"\n";
    }
    #endif

    /** Check order names joints size **/
    if (order_names.size() != joint_names.size())
        return false;
    else
        return true;
}

void Task::outputPortSamples(const Eigen::Matrix< double, Eigen::Dynamic, 1  > &jointPositions,
                                const Eigen::Matrix< double, 6, 1  > &cartesianVelocities,
                                const Eigen::Matrix< double, Eigen::Dynamic, 1  > &jointVelocities)
{
    base::samples::RigidBodyState poseOut;

    /***************************************/
    /** Port out the OutPorts information **/
    /***************************************/

    /** The Motion Model Estimated pose **/
    //pose.copyToRigidBodyState(poseOut);
    poseOut.setTransform(pose);
    poseOut.sourceFrame = _odometry_source_frame.value();
    poseOut.targetFrame = _odometry_target_frame.value();
    poseOut.time = jointsSamples.time; //!timestamp;
    poseOut.cov_position = poseCov.block<3,3>(0,0);
    poseOut.cov_orientation = poseCov.block<3,3>(3,3);
    poseOut.velocity = poseOut.orientation * cartesianVelocities.block<3,1>(0,0);
    poseOut.cov_velocity = (poseOut.orientation.matrix().transpose() * cartesianVelCov.block<3,3>(0,0).inverse() * poseOut.orientation.matrix()).inverse();
    poseOut.angular_velocity = cartesianVelocities.block<3,1> (3,0);
    poseOut.cov_angular_velocity = cartesianVelCov.block<3,3>(3,3);
    _pose_samples_out.write(poseOut);


    /** The Delta pose of this step **/
    deltaPose.time = jointsSamples.time;
    deltaPose.sourceFrame = _delta_odometry_source_frame.value();
    deltaPose.targetFrame = _delta_odometry_target_frame.value();
    deltaPose.velocity =  cartesianVelocities.block<3,1>(0,0);
    deltaPose.cov_velocity = cartesianVelCov.block<3,3>(0,0);
    deltaPose.angular_velocity = cartesianVelocities.block<3,1>(3,0);
    deltaPose.cov_angular_velocity = cartesianVelCov.block<3,3>(3,3);
    _delta_pose_samples_out.write(deltaPose);

    /** Debug information **/
    if (_output_debug.value())
    {

        /** Forward kinematics information. Set of contact points. **/
        threed_odometry::RobotContactPointsRbs robotKineRbs;

        robotKineRbs.time = poseOut.time;
        robotKineRbs.rbsChain.resize(this->fkRobotTrans.size());

        /** For the movement of the points with respect to the body center **/
        for (register size_t i=0; i<this->fkRobotTrans.size(); ++i)
        {
            robotKineRbs.rbsChain[i].invalidate();
            robotKineRbs.rbsChain[i].time = robotKineRbs.time;
            robotKineRbs.rbsChain[i].setTransform(this->fkRobotTrans[i]);
            robotKineRbs.rbsChain[i].cov_position = this->fkRobotCov[i].topLeftCorner<3,3>();
            robotKineRbs.rbsChain[i].cov_orientation = this->fkRobotCov[i].bottomRightCorner<3,3>();
        }

        _fkchains_rbs_out.write(robotKineRbs);

        /** Output port **/
        base::VectorXd weightOut(centerOfMass.percentage);
        _weighting_vector.write(weightOut);
    }


    #ifdef DEBUG_PRINTS
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: poseOut.position\n"<<poseOut.position<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: poseOut.cov_position\n"<<poseOut.cov_position<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: poseOut.velocity\n"<<poseOut.velocity<<"\n";
    Eigen::Vector3d euler;
    euler[2] = poseOut.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
    euler[1] = poseOut.orientation.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
    euler[0] = poseOut.orientation.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: Pose Orientation\n";
    std::cout<<"Roll: "<<euler[0]*R2D<<" Pitch: "<<euler[1]*R2D<<" Yaw: "<<euler[2]*R2D<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: Pose cov_orientation\n"<<poseOut.cov_orientation<<"\n";
    #endif

    #ifdef DEBUG_PRINTS
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: deltaPose.position\n"<<deltaPose.position<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: deltaPose.cov_position\n"<<deltaPose.cov_position<<"\n";
    Eigen::Vector3d deltaEuler;
    deltaEuler[2] = deltaPose.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
    deltaEuler[1] = deltaPose.orientation.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
    deltaEuler[0] = deltaPose.orientation.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
    std::cout<< "[THREED_ODOMETRY OUTPUT_PORTS]: Delta Pose Orientation\n";
    std::cout<< "******** Delta Rotation *******"<<"\n";
    std::cout<< "Roll: "<<deltaEuler[0]*R2D<<" Pitch: "<<deltaEuler[1]*R2D<<" Yaw: "<<deltaEuler[2]*R2D<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: Delta pose cov_orientation\n"<<deltaPose.cov_orientation<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]\n ******************** END ******************** \n";
    #endif


    return;
}


