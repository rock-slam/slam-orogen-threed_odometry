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
    this->vector_cartesian_velocities = std::vector< Eigen::Matrix <double, 6, 1> , Eigen::aligned_allocator < Eigen::Matrix <double, 6, 1> > > (2);
    this->vector_cartesian_velocities[0].setZero(); this->vector_cartesian_velocities[1].setZero();

    /***************************/
    /** Input port variables  **/
    /***************************/
    this->orientation_samples.orientation = Eigen::Quaterniond(base::NaN<double>()*Eigen::Vector4d::Ones());
    this->orientation_samples.cov_orientation = base::NaN<double>() * Eigen::Matrix3d::Ones();

    /***************************/
    /** Output port variables **/
    /***************************/
    this->body_pose.invalidate();

}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

void Task::joints_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Joints &joints_samples_sample)
{
    /** Two different manners to get the delta time **/
    //double delta_t = static_cast<const double>(_joints_samples_period.value());
    //double delta_t = joints_samples_sample.time.toSeconds() - joints_samples.time.toSeconds();

    /** Get the Joints values  **/
    this->joints_samples = joints_samples_sample;
    this->joint_positions.setZero();
    this->joint_velocities.setZero();

    /** Mechanical joints ordered by jointsName **/
    this->joints_samplesUnpack(this->joints_samples, this->all_joint_names, this->joint_positions, this->joint_velocities);

    #ifdef DEBUG_PRINTS
    std::cout<<"[THREED_ODOMETRY JOINT_SAMPLES] Received time-stamp:\n"<<this->joints_samples.time.toMicroseconds()<<"\n";
    std::cout<<"[THREED_ODOMETRY JOINT_SAMPLES] Joint Positions:\n"<<this->joint_positions<<"\n";
    std::cout<<"[THREED_ODOMETRY JOINT_SAMPLES] Joint Velocities:\n"<<this->joint_velocities<<"\n";
    #endif

    #ifdef DEBUG_PRINTS
    /** Check the time difference between joint and orientation samples **/
    base::Time diffTime = this->joints_samples.time - this->orientation_samples.time;

    std::cout<<"[THREED_ODOMETRY JOINT_SAMPLES] [ON] ("<<joints_samples.time.toMicroseconds()<<")\n";
    std::cout<<"[THREED_ODOMETRY JOINT_SAMPLES] diffTime:\n"<<diffTime.toSeconds()<<"\n";
    #endif

    /** Perform the Motion Model and the Dead-reckoning **/
    if (orientation_samples.time.toSeconds() != 0.00)
    {
        #ifdef DEBUG_PRINTS
        struct timeval start, end;
        gettimeofday(&start, NULL);
        #endif


        this->motionVelocities();

        #ifdef DEBUG_PRINTS
        gettimeofday(&end, NULL);
        double execution_delta = ((end.tv_sec  - start.tv_sec) * 1000000u + end.tv_usec - start.tv_usec) / 1.e6;
        RTT::log(RTT::Warning)<<"[THREED_ODOMETRY] Execution delta: ";
        RTT::log(RTT::Warning)<<execution_delta<<RTT::endlog();
        #endif

    }

}

void Task::orientation_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample)
{
    Eigen::Affine3d tf; /** Transformer transformation **/
    Eigen::Quaternion <double> qtf; /** Rotation part of the transformation in quaternion form **/

    /** Two different manners to get the delta time **/
    //double delta_t = static_cast<const double>(_orientation_samples_period.value());
    double delta_t = orientation_samples_sample.time.toSeconds() - orientation_samples.time.toSeconds();

    /** Get the transformation (transformation) Tbody_imu **/
    if (!_imu2body.get(ts, tf, false))
    {
        throw std::runtime_error("[THREED_ODOMETRY] Transformation from imu to body is not provided.");
        return;
    }

    qtf = Eigen::Quaternion <double> (tf.rotation());//!Quaternion from Body to imu (transforming samples from imu to body)

    /** Transform the orientation world_imu to world_body **/
    Eigen::Quaterniond orientation_in_body  = orientation_samples_sample.orientation * qtf.inverse(); // Tworld_body = Tworld_imu * (Tbody_imu)^-1
    Eigen::Matrix3d cov_orientation_in_body = tf.rotation() * orientation_samples_sample.cov_orientation * tf.rotation().transpose(); // Tworld_body = Tworld_imu * (Tbody_imu)^-1

    /* Pose and Twist to zero **/
    this->delta_pose.initUnknown();
    this->delta_pose.cov_pose().setZero();
    this->delta_pose.cov_velocity().setZero();

    /** Delta quaternion: (rotation k-1 - rotation k) **/
    this->delta_pose.orientation() = this->orientation_samples.orientation.inverse() * orientation_in_body; /** previous * ((T0_k-1)^-1 * T0_k) **/
    Eigen::Matrix3d delta_cov_orientation; delta_cov_orientation = cov_orientation_in_body - this->orientation_samples.cov_orientation; /** delta_cov in body frame **/
    Task::guaranteeSPD(delta_cov_orientation);
    Eigen::Matrix3d orientation_matrix; orientation_matrix = this->delta_pose.orientation().toRotationMatrix();
    this->delta_pose.cov_orientation(orientation_matrix * delta_cov_orientation * orientation_matrix.transpose());

    /** Angular velocity **/
    Eigen::Quaterniond delta_orientation(this->delta_pose.orientation());
    Eigen::Vector3d angular_velocity =  Task::boxminus(delta_orientation.w(), delta_orientation.vec(), double(2), true);
    angular_velocity = angular_velocity/delta_t;

    /** Fill the Cartesian Velocities **/
    this->delta_pose.velocity.vel = Eigen::Matrix<double, 3, 1>::Identity() * base::NaN<double>();
    this->delta_pose.velocity.rot = angular_velocity;//!Angular velocities come from orientation derivation

    /** Fill the Cartesian velocity covariance **/
    this->delta_pose.cov_velocity() = this->delta_pose.cov_pose()/(delta_t * delta_t);

    /** Get the orientation readings  **/
    this->orientation_samples.time = orientation_samples_sample.time;
    this->orientation_samples.orientation = orientation_in_body;
    this->orientation_samples.cov_orientation = cov_orientation_in_body;

    #ifdef DEBUG_PRINTS
    std::cout<<"[THREED_ODOMETRY INERTIAL_SAMPLES] Received time-stamp:\n"<<orientation_samples.time.toMicroseconds()<<"\n";
    std::cout<<"[THREED_ODOMETRY INERTIAL_SAMPLES] Delta_t:\n"<<delta_t<<"\n";
    std::cout<<"[THREED_ODOMETRY INERTIAL_SAMPLES] Cartesian Velocities:\n"<<this->delta_pose.velocity<<"\n";
    #endif

    #ifdef DEBUG_PRINTS
    /** Check the time difference between orientation and joint samples **/
    base::Time diffTime = orientation_samples.time - joints_samples.time;
    std::cout<<"[THREED_ODOMETRY INERTIAL_SAMPLES] diffTime:\n"<<diffTime.toSeconds()<<"\n";
    #endif

    /** Compute Odometry only if there were samples from the joints **/
    if (this->joints_samples.time.toSeconds() != 0.00)
    {
        /** Perform the Dead-Reckoning **/
        this->deadReckoning(delta_t);

        /** Out port the information **/
        this->outputPortSamples (joint_positions, joint_velocities);
    }
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    /************************/
    /** Read configuration **/
    /************************/
    this->urdfFile = _urdf_file.value();
    this->iirConfig = _iir_filter.value();
    this->contact_point_segments = _contact_point_segments.value();
    this->contact_angle_segments = _contact_angle_segments.value();
    this->all_joint_names = _all_joint_names.value();
    this->slip_joint_names = _slip_joint_names.value();
    this->contact_joint_names = _contact_joint_names.value();

    this->number_robot_joints =  this->all_joint_names.size()-this->slip_joint_names.size()-this->contact_joint_names.size();

    this->kinematic_model_type = _kinematic_model_type.value();

    /*********************************************/
    /** Configure the Motion Model of the Robot **/
    /*********************************************/
    if (kinematic_model_type == NUMERICAL)
    {
        /** Robot Kinematics Model **/
        robotKinematics.reset(new threed_odometry::KinematicKDL (urdfFile, this->contact_point_segments,
                            this->contact_angle_segments, this->number_robot_joints, this->slip_joint_names.size(), this->contact_joint_names.size()));
        RTT::log(RTT::Warning)<<"[THREED_ODOMETRY] Numerical Model selected"<<RTT::endlog();
    }
    else if (kinematic_model_type == ANALYTICAL)
    {
        /** HERE YOU NEED TO PROVIDE YOUR NUMERICAL "AD HOC" KINEMATICS MODEL **/
        RTT::log(RTT::Warning)<<"[THREED_ODOMETRY] Analytical KDL Model selected"<<RTT::endlog();
    }
    else
        throw std::runtime_error("[THREED_ODOMETRY] Invalid Odometry Model.");


    /** Create the Motion Model **/
    motionModel.reset(new threed_odometry::MotionModel<double> (this->contact_point_segments.size(), this->number_robot_joints,
                                                        this->slip_joint_names.size(), this->contact_joint_names.size()));

    /** Weighting Matrix Initialization. Default is equally distributed among all the contact_points_segments **/
    this->WeightMatrix.resize(6*this->contact_point_segments.size(), 6*this->contact_point_segments.size());
    this->WeightMatrix.setIdentity();
    this->WeightMatrix = (1.0/this->contact_point_segments.size()) * WeightMatrix;

    /*************************/
    /** Motion Model Joints **/
    /*************************/
    this->motion_model_joint_names.resize(this->all_joint_names.size());
    bool result = this->joints_samplesMotionModel(this->motion_model_joint_names, this->all_joint_names, this->slip_joint_names, this->contact_joint_names);
    if (!result)
    {
        throw std::runtime_error("[THREED_ODOMETRY] Error Ordering Motion Model joints");
    }

    /****************/
    /** IIR Filter **/
    /****************/
    if (this->iirConfig.iirOn)
    {
        RTT::log(RTT::Warning)<<"[THREED_ODOMETRY] Infinite Impulse Response Filter [ON]"<<RTT::endlog();
        Eigen::Matrix <double, NORDER_BESSEL_FILTER+1, 1> besselBCoeff, besselACoeff;
        besselBCoeff = iirConfig.feedForwardCoeff;
        besselACoeff = iirConfig.feedBackCoeff;

        /** Create the Bessel Low-pass filter with the right coefficients **/
        this->bessel.reset(new threed_odometry::IIR<NORDER_BESSEL_FILTER, 3> (besselBCoeff, besselACoeff));
    }
    else
        RTT::log(RTT::Warning)<<"[THREED_ODOMETRY] Infinite Impulse Response Filter [OFF]"<<RTT::endlog();

    /*******************************/
    /** Inertial Noise Covariance **/
    /*******************************/

    /** Resize the Velocity covariance **/
    this->modelVelCov.resize(robotKinematics->model_dof, robotKinematics->model_dof);
    this->modelVelCov.setZero();

    /***************************/
    /** Input port variables **/
    /***************************/
    this->orientation_samples.position.setZero();
    this->orientation_samples.cov_position.setZero();
    this->orientation_samples.orientation.setIdentity();
    this->orientation_samples.cov_orientation.setZero();

    /************************************/
    /** Resize variables for Odometry  **/
    /************************************/
    this->joint_positions.resize(robotKinematics->model_dof, 1);
    this->joint_velocities.resize(robotKinematics->model_dof, 1);

    /*************************************/
    /** Reset initial Starting position **/
    /*************************************/

    /** Get the Initial pose and uncertainty **/
    this->body_pose.initUnknown();
    this->body_pose.cov_pose().setZero();
    this->body_pose.cov_velocity().setZero();

    /** Delta pose initialization **/
    this->delta_pose.initUnknown();
    this->delta_pose.cov_pose().setZero();
    this->delta_pose.cov_velocity().setZero();

    #ifdef DEBUG_PRINTS
    std::cout<< "[THREED_ODOMETRY]\n";
    std::cout<< "******** Initial Pose *******"<<"\n";
    std::cout<< this->body_pose<<"\n";
    std::cout<< "******** Initial Delta Pose *******"<<"\n";
    std::cout<< this->delta_pose<<"\n";

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

void Task::motionVelocities ()
{

    bool known_contact_angles = false;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> J;
    std::vector<double> vectorPositions(robotKinematics->model_dof, 0);

    /** Copy Eigen to vector **/
    Eigen::Map <Eigen::Matrix <double, Eigen::Dynamic, 1> > (&(vectorPositions[0]), robotKinematics->model_dof) = joint_positions;

    /** Update the Motion Model (Forward Kinematics and to set Contact Points) **/
    this->robotKinematics->fkSolver(vectorPositions, this->contact_point_segments, this->fkRobotTrans, this->fkRobotCov);

    /** Compute Robot Jacobian matrix **/
    J = this->robotKinematics->jacobianSolver(this->all_joint_names, vectorPositions);

    /** In case of no noise in properties, set to zero to get the noise from the Motion Model **/
    if (!_orientation_samples_noise_on.value())
        this->delta_pose.velocity.invalidateCovariance();

    /** Reorganize the Jacobian matrix as required by the motion model **/
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> organized_J;
    organized_J.resize(6*this->contact_point_segments.size(), robotKinematics->model_dof);

    /** Get joints position and velocity ordered by Motion Model joint names **/
    this->joints_samplesUnpack(joints_samples, motion_model_joint_names, joint_positions, joint_velocities);

    /** In case the information about contact angles is not NaN **/
    if (base::isnotnan(joint_velocities.block(this->number_robot_joints+this->slip_joint_names.size(), 0, this->contact_joint_names.size(),1)) &&
       base::isnotnan(joint_positions.block(this->number_robot_joints+this->slip_joint_names.size(), 0, this->contact_joint_names.size(),1)))
    {
       known_contact_angles = true;
    }

    this->robotKinematics->organizeJacobian(0, motion_model_joint_names, this->all_joint_names, J, organized_J);

    #ifdef DEBUG_PRINTS
    std::cout<<"** [MOTION_VELOCITIES] Contact angle information available: "<<known_contact_angles<<"\n";
    std::cout<<"** [MOTION_VELOCITIES] JACOBIAN KDL is of size "<<organized_J.rows()<<" x "<<organized_J.cols()<<"\n"<< organized_J<<"\n\n";
    #endif

    /** Read new Weighting matrix from the Input port **/
    base::MatrixXd wMatrix;
    if (_weighting_samples.read(wMatrix, false) == RTT::NewData)
    {
        if (wMatrix.size() == WeightMatrix.size())
        {
            this->WeightMatrix = wMatrix;
        }
        else
        {
            throw std::runtime_error("[THREED_ODOMETRY WEIGHT MATRIX] Weight Matrix with different size than expected.");
        }
    }

    Eigen::Matrix<double, 6, 1> velocities;
    Eigen::Matrix<double, 6, 6> cov_velocities;

    /** 3D odometry library expects velocity vector and covariance in inversion order **/
    velocities.block<3,1>(0,0) = this->delta_pose.velocity.vel;
    velocities.block<3,1>(3,0) = this->delta_pose.velocity.rot;
    cov_velocities.topLeftCorner<3,3>() = this->delta_pose.cov_linear_velocity();
    cov_velocities.bottomRightCorner<3,3>() = this->delta_pose.cov_angular_velocity();

    /** Solve the navigation kinematics **/
    this->motionModel->navSolver(joint_positions, joint_velocities, organized_J,
                                velocities,
                                modelVelCov,
                                cov_velocities,
                                WeightMatrix,
                                known_contact_angles);

    this->delta_pose.velocity.vel = velocities.block<3,1>(0,0);
    this->delta_pose.velocity.rot = velocities.block<3,1>(3,0);
    this->delta_pose.velocity.setLinearVelocityCov(cov_velocities.topLeftCorner<3,3>());
    this->delta_pose.velocity.setAngularVelocityCov(cov_velocities.bottomRightCorner<3,3>());

    /** Bessel IIR Low-pass filter of the linear cartesian velocities from the Motion Model **/
    if (iirConfig.iirOn)
    {
        Eigen::Matrix<double, 3, 1> velocity = this->delta_pose.velocity.vel;
        Eigen::Matrix<double, 3, 3> velocity_cov = this->delta_pose.cov_linear_velocity();
        this->delta_pose.velocity.vel = this->bessel->perform(velocity, velocity_cov, false);

        /** Store the filtered velocity uncertainty (Uncertainty propagation is
         * time-correlated by the IIR) **/
        this->delta_pose.velocity.setLinearVelocityCov(velocity_cov);
    }

    /** Update the Cartesian velocities on the std_vector **/
    this->vector_cartesian_velocities[1] = this->vector_cartesian_velocities[0];
    this->vector_cartesian_velocities[0] = this->delta_pose.velocity.getVelocity();

    return;
}

void Task::deadReckoning (const double &delta_t)
{

    /** Complete the delta pose  (assuming constant acceleration) **/
    this->delta_pose.position() = delta_t * ((this->vector_cartesian_velocities[1].block<3,1>(3,0) + this->vector_cartesian_velocities[0].block<3,1>(3,0))/2.0);
    this->delta_pose.cov_position(this->delta_pose.cov_linear_velocity() * (delta_t * delta_t)); //covariance in body_frame

    /** Take uncertainty on delta orientation from the motion model **/
    if (!_orientation_samples_noise_on.value())
    {
        this->delta_pose.cov_orientation(this->delta_pose.cov_angular_velocity() * (delta_t * delta_t));
    }

    /** Guarantee SPD covariance **/
    Task::guaranteeSPD(this->delta_pose.cov_pose());
    Task::guaranteeSPD(this->delta_pose.cov_velocity());

    /** The uncertainty needs to be transformed to the navigation frame **/
    //this->delta_pose.cov_position((this->body_pose.orientation().toRotationMatrix() * this->delta_pose.cov_position().inverse() * this->body_pose.orientation().toRotationMatrix().transpose()).inverse());
    //this->delta_pose.cov_orientation((this->body_pose.orientation().toRotationMatrix() * this->delta_pose.cov_orientation().inverse() * this->body_pose.orientation().toRotationMatrix().transpose()).inverse());

    /** Dead Reckon: Propagate Pose and its associated uncertainty **/
    this->body_pose = this->body_pose * this->delta_pose;
    //this->body_pose.velocity = this->delta_pose.velocity;
    //this->body_pose.cov_velocity() = this->delta_pose.cov_velocity();

    /** Guarantee SPD covariance **/
    Task::guaranteeSPD(this->body_pose.cov_pose());
    Task::guaranteeSPD(this->body_pose.cov_velocity());

    return;
}

void Task::joints_samplesUnpack(const ::base::samples::Joints &original_joints,
                                const std::vector<std::string> &order_names,
                                Eigen::Matrix< double, Eigen::Dynamic, 1  > &joint_positions,
                                Eigen::Matrix< double, Eigen::Dynamic, 1  > &joint_velocities)
{
    assert (static_cast<unsigned int>(joint_positions.size()) == original_joints.size());
    assert (static_cast<unsigned int>(joint_velocities.size()) == original_joints.size());
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
    order_names.insert(order_names.end(), slip_names.begin(), slip_names.end());

    /** Concatenate the contact joints at the end **/
    order_names.insert(order_names.end(), contact_names.begin(), contact_names.end());

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

void Task::outputPortSamples(const Eigen::Matrix< double, Eigen::Dynamic, 1  > &joint_positions,
                                const Eigen::Matrix< double, Eigen::Dynamic, 1  > &joint_velocities)
{
    base::samples::RigidBodyState pose_out;

    /***************************************/
    /** Port out the OutPorts information **/
    /***************************************/

    /** The Motion Model Estimated pose **/
    /** NOTE: Position and orientation values are wrt the local navigation frame (frame where the dead-reckoning process "3D-Odometry" started) **/
    pose_out.setTransform(this->body_pose.pose.getTransform());
    pose_out.sourceFrame = _odometry_source_frame.value();
    pose_out.targetFrame = _odometry_target_frame.value();
    pose_out.time = joints_samples.time; //!timestamp;
    pose_out.cov_position = this->body_pose.cov_position();
    pose_out.cov_orientation = this->body_pose.cov_orientation();

    /** NOTE: Linear and angular velocities are wrt the local navigation frame (frame where the dead-reckoning process "3D-Odometry" started) **/
    pose_out.velocity = this->body_pose.linear_velocity();
    pose_out.cov_velocity = this->body_pose.cov_linear_velocity();
    pose_out.angular_velocity = this->body_pose.angular_velocity();
    pose_out.cov_angular_velocity =  this->body_pose.cov_angular_velocity();

    _pose_samples_out.write(pose_out);

    /** The Motion Model Estimated pose in body state **/
    this->body_pose.time = joints_samples.time; //!timestamp;
    _body_pose_samples_out.write(this->body_pose);

    /** The Delta pose of this step. Delta pose transformation with instantaneous velocity **/
    /** NOTE: Linear and Angular velocities are wrt the local robot body frame **/
    this->delta_pose.time = joints_samples.time;
    _delta_pose_samples_out.write(this->delta_pose);

    /** Debug information **/
    if (_output_debug.value())
    {
        /** Forward kinematics information. Set of contact points. **/
        threed_odometry::RobotContactPointsRbs robotKineRbs;

        robotKineRbs.time = pose_out.time;
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
    }


    #ifdef DEBUG_PRINTS
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: BODY POSE\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: body_pose\n"<<this->body_pose<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: POSE OUT\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: pose_out.sourceFrame\n"<<pose_out.sourceFrame<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: pose_out.targetFrame\n"<<pose_out.targetFrame<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: pose_out.position\n"<<pose_out.position<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: pose_out.cov_position\n"<<pose_out.cov_position<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: pose_out.velocity\n"<<pose_out.velocity<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: pose_out.cov_velocity\n"<<pose_out.cov_velocity<<"\n";
    Eigen::Vector3d euler;
    euler[2] = pose_out.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
    euler[1] = pose_out.orientation.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
    euler[0] = pose_out.orientation.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: Pose Orientation\n";
    std::cout<< "******** Pose Rotation *******"<<"\n";
    std::cout<<"Roll: "<<euler[0]*R2D<<" Pitch: "<<euler[1]*R2D<<" Yaw: "<<euler[2]*R2D<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: Pose cov_orientation\n"<<pose_out.cov_orientation<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: pose_out.angular_velocity\n"<<pose_out.angular_velocity<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: pose_out.cov_angular_velocity\n"<<pose_out.cov_angular_velocity<<"\n";
    #endif

    #ifdef DEBUG_PRINTS
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: DELTA POSE OUT\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: delta_pose\n"<<this->delta_pose<<"\n";
    Eigen::Vector3d deltaEuler;
    deltaEuler[2] = this->delta_pose.orientation().toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
    deltaEuler[1] = this->delta_pose.orientation().toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
    deltaEuler[0] = this->delta_pose.orientation().toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
    std::cout<< "[THREED_ODOMETRY OUTPUT_PORTS]: Delta Pose Orientation\n";
    std::cout<< "******** Delta Rotation *******"<<"\n";
    std::cout<< "Roll: "<<deltaEuler[0]*R2D<<" Pitch: "<<deltaEuler[1]*R2D<<" Yaw: "<<deltaEuler[2]*R2D<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]\n ******************** END ******************** \n";
    #endif


    return;
}


