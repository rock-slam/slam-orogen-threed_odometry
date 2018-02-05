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
    vector_cartesian_velocities = std::vector< Eigen::Matrix <double, 6, 1> , Eigen::aligned_allocator < Eigen::Matrix <double, 6, 1> > > (2);
    vector_cartesian_velocities[0].setZero(); vector_cartesian_velocities[1].setZero();

    /***************************/
    /** Input port variables  **/
    /***************************/
    orientation_samples.orientation = Eigen::Quaterniond(base::NaN<double>()*Eigen::Vector4d::Ones());
    orientation_samples.cov_orientation = base::NaN<double>() * Eigen::Matrix3d::Ones();

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

void Task::joints_samplesTransformerCallback(const base::Time &ts,
        const ::base::samples::Joints &joints_samples_sample)
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

    /** Perform the Motion Model and get the velocities **/
    if (this->orientation_samples.time.toSeconds() != 0.00)
    {
        this->motionVelocities();

        this->outputPortContactPoints();
    }
}

void Task::orientation_samplesTransformerCallback(const base::Time &ts,
        const ::base::samples::RigidBodyState &orientation_samples_sample)
{
    Eigen::Affine3d tf; /** Transformer transformation **/
    Eigen::Quaternion <double> qtf; /** Rotation part of the transformation in quaternion form **/

    /** Compute Odometry only if there were samples from the joints **/
    if (this->joints_samples.time.toSeconds() == 0.00)
    {
        std::cout << "[THREED_ODOMETRY] No valid joint samples. Skipping orientation handling" << std::endl;
        return;
    }

    /** Two different manners to get the delta time **/
    double delta_t = static_cast<const double>(_orientation_samples_period.value());
    if (this->orientation_samples.time.toSeconds() != 0.00)
    {
        delta_t = orientation_samples_sample.time.toSeconds() - orientation_samples.time.toSeconds();
    }
  
    /** Get the transformation (transformation) Tbody_imu **/
    if (_body_frame.value().compare(_imu_frame.value()) == 0)
    {
        tf.setIdentity();
    }
    else if (!_imu2body.get(ts, tf, false))
    {
        throw std::runtime_error("[THREED_ODOMETRY] Transformation from imu to body is not provided.");
        return;
    }

    qtf = Eigen::Quaternion <double> (tf.rotation());//!Quaternion from Body to imu (transforming samples from imu to body)

    /** Transform the orientation world_imu to world_body **/
    Eigen::Quaterniond orientation_in_body  = orientation_samples_sample.orientation * qtf.inverse(); // Tworld_body = Tworld_imu * (Tbody_imu)^-1
    Eigen::Matrix3d cov_orientation_in_body = tf.inverse().rotation() * orientation_samples_sample.cov_orientation * tf.inverse().rotation().transpose(); // Tworld_body = Tworld_imu * (Tbody_imu)^-1

    /** Reset delta pose **/
    this->delta_pose.position.setZero();
    this->delta_pose.cov_position.setZero();
    this->delta_pose.orientation = Eigen::Quaterniond::Identity();
    this->delta_pose.cov_orientation.setZero();

    /** Delta quaternion: (rotation k-1 - rotation k) **/
    this->delta_pose.orientation = this->orientation_samples.orientation.inverse() * orientation_in_body; /** (T0_k-1)^-1 * T0_k **/
    this->delta_pose.cov_orientation = cov_orientation_in_body - this->orientation_samples.cov_orientation;
    Task::guaranteeSPD< Eigen::Matrix<double, 3, 3> > (this->delta_pose.cov_orientation);

    /** Angular velocity **/
    Eigen::Vector3d angular_velocity =  Task::boxminus(this->delta_pose.orientation.w(), this->delta_pose.orientation.vec(), double(2), true);
    angular_velocity = angular_velocity/delta_t;

    /** Fill the Cartesian Velocities **/
    this->cartesian_velocities.block<3,1> (3,0) = angular_velocity;//!Angular velocities come from gyros

    /** Fill the Cartesian velocity covariance **/
    this->cartesianVelCov.block<3,3>(3,3) = this->delta_pose.cov_orientation / delta_t;

    /** Get the orientation readings  **/
    this->orientation_samples.time = orientation_samples_sample.time;
    this->orientation_samples.orientation = orientation_in_body;
    this->orientation_samples.cov_orientation = cov_orientation_in_body;

    #ifdef DEBUG_PRINTS
    std::cout<<"[THREED_ODOMETRY INERTIAL_SAMPLES] Received time-stamp:\n"<<this->orientation_samples.time.toMicroseconds()<<"\n";
    std::cout<<"[THREED_ODOMETRY INERTIAL_SAMPLES] Delta_t:\n"<<delta_t<<"\n";
    std::cout<<"[THREED_ODOMETRY INERTIAL_SAMPLES] Cartesian Velocities:\n"<<this->cartesian_velocities<<"\n";
    #endif

    #ifdef DEBUG_PRINTS
    /** Check the time difference between orientation and joint samples **/
    base::Time diffTime = this->orientation_samples.time - this->joints_samples.time;
    std::cout<<"[THREED_ODOMETRY INERTIAL_SAMPLES] diffTime:\n"<<diffTime.toSeconds()<<"\n";
    #endif

    /** Perform the Dead-Reckoning **/
    this->deadReckoning(delta_t);

    /** Out port the information **/
    this->outputPortPose (this->cartesian_velocities);

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
    this->all_joint_names = _all_joint_names.value();
    this->slip_joint_names = _slip_joint_names.value();
    this->contact_joint_names = _contact_joint_names.value();

    this->number_robot_joints =  this->all_joint_names.size()-this->slip_joint_names.size()-this->contact_joint_names.size();

    this->kinematic_model_type = _kinematic_model_type.value();

        //RTT::log(RTT::Warning)<<"[THREED_ODOMETRY] Dynamic Weight Matrix [ON]"<<RTT::endlog();

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
    this->weight_matrix.resize(6*this->contact_point_segments.size(), 6*this->contact_point_segments.size());
    this->weight_matrix.setIdentity();
    this->weight_matrix = (1.0/this->contact_point_segments.size()) * weight_matrix;

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

    /** Angular velocity coming from gyros **/
    this->cartesianVelCov.setZero(); modelVelCov.setZero();

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
    this->pose = poseInit.getTransform();
    this->poseCov << poseInit.cov_position, Eigen::Matrix3d::Zero(),
            Eigen::Matrix3d::Zero(), poseInit.cov_orientation;

    /** Delta pose initialization **/
    this->delta_pose.invalidate();
    this->delta_pose.sourceFrame = _delta_odometry_source_frame.value();
    this->delta_pose.targetFrame = _delta_odometry_target_frame.value();
    this->delta_pose.position.setZero();
    this->delta_pose.cov_position.setZero();
    this->delta_pose.orientation = Eigen::Quaterniond::Identity();
    this->delta_pose.cov_orientation.setZero();

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
        cartesianVelCov.setZero();

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
    std::cout<<"** [UPDATE_ODOMETRY] Contact angle information available: "<<known_contact_angles<<"\n";
    std::cout<<"** [UPDATE_ODOMETRY] JACOBIAN KDL is of size "<<organized_J.rows()<<" x "<<organized_J.cols()<<"\n"<< organized_J<<"\n\n";
    #endif

    /** Read new Weighting matrix diagonal from the Input port **/
    base::VectorXd w_diagonal;
    if (_weighting_samples.read(w_diagonal, false) == RTT::NewData)
    {
        /** The vector size should be equal to the matrix number of columns **/
        if (w_diagonal.size() == weight_matrix.cols())
        {
            this->weight_matrix.diagonal() = w_diagonal;
        }
        else
        {
            throw std::runtime_error("[THREED_ODOMETRY WEIGHT MATRIX] Weight Matrix with different size than expected.");
        }
    }

    /** NaN linear velocities, these are the quatities to compute by the motion model **/
    this->cartesian_velocities.block<3,1> (0,0) = Eigen::Matrix<double, 3, 1>::Identity() * base::NaN<double>();

    /** Solve the navigation kinematics **/
    this->motionModel->navSolver(joint_positions, joint_velocities, organized_J, cartesian_velocities,
                                modelVelCov, cartesianVelCov, weight_matrix, known_contact_angles);

    /** Bessel IIR Low-pass filter of the linear cartesian_velocities from the Motion Model **/
    if (iirConfig.iirOn)
    {
        Eigen::Matrix<double, 3, 1> velocity = cartesian_velocities.block<3, 1>(0,0);
        Eigen::Matrix<double, 3, 3> velocityCov = cartesianVelCov.block<3, 3>(0,0);
        cartesian_velocities.block<3, 1>(0,0) = this->bessel->perform(velocity, velocityCov, false);

        /** Store the filtered velocity uncertainty (Uncertainty propagation is
         * time-correlated by the IIR) **/
        cartesianVelCov.block<3, 3>(0,0) = velocityCov;
    }

    /** Update the Cartesian velocities on the std_vector **/
    this->vector_cartesian_velocities[1] = this->vector_cartesian_velocities[0];
    this->vector_cartesian_velocities[0] = cartesian_velocities;

    return;
}

void Task::deadReckoning(const double &delta_t)
{

    /** Complete the delta pose  (assuming constant acceleration) **/
    this->delta_pose.position = delta_t * ((vector_cartesian_velocities[1].block<3,1>(0,0) + vector_cartesian_velocities[0].block<3,1>(0,0))/2.0);
    this->delta_pose.cov_position = cartesianVelCov.block<3, 3>(0,0) * delta_t;

    /** Take uncertainty on delta orientation from the motion model **/
    if (!_orientation_samples_noise_on.value())
    {
        this->delta_pose.cov_orientation = cartesianVelCov.block<3, 3>(3,3) * delta_t;
    }

    /** Perform the velocities integration to get the pose (Dead Reckoning) **/
    Eigen::Matrix<double, 6, 6> delta_poseCov;

    /** The uncertainty needs to be transformed to the navigation frame **/
    delta_poseCov<< this->pose.rotation() * this->delta_pose.cov_position * this->pose.rotation().transpose() , Eigen::Matrix3d::Zero(),
            Eigen::Matrix3d::Zero(), this->pose.rotation() * this->delta_pose.cov_orientation * this->pose.rotation().transpose();

    /** Dead Reckon: Propagate Pose **/
    this->pose = this->pose * this->delta_pose.getTransform();

    /** Adding method of propagating uncertainty **/
    this->poseCov = this->poseCov + delta_poseCov;

    /** Guarantee SPD covariance **/
    Task::guaranteeSPD(poseCov);

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
        try
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
        } catch(const std::exception& e){
            RTT::log(RTT::Error)<<e.what()<<RTT::endlog();
            throw std::runtime_error("[THREED_ODOMETRY JOINT_SAMPLES] Joint name not found");
        }

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
    std::cout<<"[THREED_ODOMETRY JOINTS_SAMPLES] Number of Ordered Joints: "<<order_names.size()<<"\n";
    std::cout<<"[THREED_ODOMETRY JOINTS_SAMPLES] Name for Ordered Joints:\n";
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

void Task::outputPortPose(const Eigen::Matrix< double, 6, 1  > &cartesian_velocities)
{
    base::samples::RigidBodyState pose_out;

    /***************************************/
    /** Port out the OutPorts information **/
    /***************************************/

    /** The Motion Model Estimated pose **/
    /** NOTE: Position and orientation values are wrt the local navigation frame (frame where the dead-reckoning process "3D-Odometry" started) **/
    pose_out.setTransform(pose);
    pose_out.sourceFrame = _odometry_source_frame.value();
    pose_out.targetFrame = _odometry_target_frame.value();
    pose_out.time = this->orientation_samples.time; //!timestamp;
    pose_out.cov_position = poseCov.block<3,3>(0,0);
    pose_out.cov_orientation = poseCov.block<3,3>(3,3);

    /** NOTE: Linear and angular velocities are wrt the local robot body frame **/
    //pose_out.velocity = cartesian_velocities.block<3,1>(0,0);
    //pose_out.cov_velocity = cartesianVelCov.block<3,3>(0,0);
    //pose_out.angular_velocity = cartesian_velocities.block<3,1> (3,0);
    //pose_out.cov_angular_velocity = cartesianVelCov.block<3,3>(3,3);

    /** NOTE: Linear and angular velocities are wrt the local navigation frame (frame where the dead-reckoning process "3D-Odometry" started) **/
    pose_out.velocity = pose_out.orientation * cartesian_velocities.block<3,1>(0,0);//v_navigation = Tnavigation_body * v_body
    pose_out.cov_velocity = pose_out.orientation.matrix() * cartesianVelCov.block<3,3>(0,0) * pose_out.orientation.matrix().transpose();
    pose_out.angular_velocity = pose_out.orientation * cartesian_velocities.block<3,1> (3,0);
    pose_out.cov_angular_velocity = pose_out.orientation.matrix() * cartesianVelCov.block<3,3>(3,3) * pose_out.orientation.matrix().transpose();
    _pose_samples_out.write(pose_out);


    /** The Delta pose of this step. Delta pose transformation with instantaneous velocity **/
    /** NOTE: Linear and Angular velocities are wrt the local robot body frame **/
    this->delta_pose.time = this->orientation_samples.time;
    this->delta_pose.velocity =  cartesian_velocities.block<3,1>(0,0);
    this->delta_pose.cov_velocity = cartesianVelCov.block<3,3>(0,0);
    this->delta_pose.angular_velocity = cartesian_velocities.block<3,1>(3,0);
    this->delta_pose.cov_angular_velocity = cartesianVelCov.block<3,3>(3,3);

    _delta_pose_samples_out.write(this->delta_pose);

    #ifdef DEBUG_PRINTS
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: pose_out.position\n"<<pose_out.position<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: pose_out.cov_position\n"<<pose_out.cov_position<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: pose_out.velocity\n"<<pose_out.velocity<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: pose_out.cov_velocity\n"<<pose_out.cov_velocity<<"\n";
    Eigen::Vector3d euler;
    euler[2] = pose_out.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
    euler[1] = pose_out.orientation.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
    euler[0] = pose_out.orientation.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: Pose Orientation\n";
    std::cout<<"Roll: "<<euler[0]*R2D<<" Pitch: "<<euler[1]*R2D<<" Yaw: "<<euler[2]*R2D<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: Pose cov_orientation\n"<<pose_out.cov_orientation<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: pose_out.angular_velocity\n"<<pose_out.angular_velocity<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: pose_out.cov_angular_velocity\n"<<pose_out.cov_angular_velocity<<"\n";
    #endif

    #ifdef DEBUG_PRINTS
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: delta_pose.position\n"<<delta_pose.position<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: delta_pose.cov_position\n"<<delta_pose.cov_position<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: delta_pose.velocity\n"<<delta_pose.velocity<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: delta_pose.cov_velocity\n"<<delta_pose.cov_velocity<<"\n";
    Eigen::Vector3d deltaEuler;
    deltaEuler[2] = delta_pose.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
    deltaEuler[1] = delta_pose.orientation.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
    deltaEuler[0] = delta_pose.orientation.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
    std::cout<< "[THREED_ODOMETRY OUTPUT_PORTS]: Delta Pose Orientation\n";
    std::cout<< "******** Delta Rotation *******"<<"\n";
    std::cout<< "Roll: "<<deltaEuler[0]*R2D<<" Pitch: "<<deltaEuler[1]*R2D<<" Yaw: "<<deltaEuler[2]*R2D<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: Delta pose cov_orientation\n"<<delta_pose.cov_orientation<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: delta_pose.angular_velocity\n"<<delta_pose.angular_velocity<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]: delta_pose.cov_angular_velocity\n"<<delta_pose.cov_angular_velocity<<"\n";
    std::cout<<"[THREED_ODOMETRY OUTPUT_PORTS]\n ******************** END ******************** \n";
    #endif


    return;
}

void Task::outputPortContactPoints()
{
    /** Debug information **/
    if (_output_debug.value())
    {
        /** Forward kinematics information. Set of contact points. **/
        threed_odometry::RobotContactPointsRbs robotKineRbs;

        robotKineRbs.time = this->joints_samples.time;
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
    return;
}


