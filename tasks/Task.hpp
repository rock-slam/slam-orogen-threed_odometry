/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef THREED_ODOMETRY_TASK_TASK_HPP
#define THREED_ODOMETRY_TASK_TASK_HPP

#include "threed_odometry/TaskBase.hpp"

/** Boost **/
#include <boost/shared_ptr.hpp> /** Shared pointers **/

/** Eigen **/
#include <Eigen/Core>/** Eigen core library **/
#include <Eigen/Geometry>/** Eigen geometry **/
#include <Eigen/SVD> /** Eigen SVD decomposition**/
#include <Eigen/Dense> /** Algebra and transformation matrices **/
#include <Eigen/StdVector> /** STL container with Eigen types **/

/** 3D Odometry includes **/
#include <threed_odometry/KinematicKDL.hpp> /** KDL model **/
#include <threed_odometry/MotionModel.hpp> /** Motion model solver **/
#include <threed_odometry/IIR.hpp> /** Infinite Impulse Response filter **/



namespace threed_odometry {

    /** Data types definition **/
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> WeightingMatrix;

    template<class scalar> inline scalar tolerance();

    template<> inline float  tolerance<float >() { return 1e-5f; }
    template<> inline double tolerance<double>() { return 1e-11; }

    static const unsigned int NORDER_BESSEL_FILTER = 8; /** Order of the IIR Bessel filter **/

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Declare the Three Odometry class
    The component computes the robot pose based on
    a complete motion model.
    Robot joints positions are needed to compute
    the forward kinematics of robot chains.
    Angular and robot joints
    rates are needed to compute the movement.


    The corresponding C++ class can be edited in tasks/Task.hpp and
    tasks/Task.cpp, and will be put in the threed_odometry namespace.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','threed_odometry::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:

        virtual void joints_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Joints &joints_samples_sample);

        virtual void orientation_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample);

    protected:

        /**************************/
        /*** Property Variables ***/
        /**************************/

        std::string urdfFile;

        std::vector<std::string> contact_point_segments;

        std::vector<std::string> contact_angle_segments;

        /** Order of Joints by Name **/
        std::vector<std::string> all_joint_names;

        std::vector<std::string> slip_joint_names;

        std::vector<std::string> contact_joint_names;

        ModelType kinematic_model_type;

        /** IIR filter configuration structure **/
        IIRCoefficients iirConfig;

        /******************************************/
        /*** General Internal Storage Variables ***/
        /******************************************/

        /** Number of physical joints according to the model and task properties  **/
        int number_robot_joints;

        /** Name of all joints model names **/
        std::vector<std::string> motion_model_joint_names;

        /** Joint, Slip and Contact Angle positions NOTE: The order of the storage needs to be coincident if used as input for the motionModel **/
        Eigen::Matrix< double, Eigen::Dynamic, 1  > joint_positions;

        /** Joint, Slip and Contact Angle velocities NOTE: The order of the storage needs to be coincident if used as input for the motionModel **/
        Eigen::Matrix< double, Eigen::Dynamic, 1  > joint_velocities;

        /** Buffer for the storage of cartesian velocities variables  (for integration assuming constant acceleration) **/
        std::vector< Eigen::Matrix <double, 6, 1> , Eigen::aligned_allocator < Eigen::Matrix <double, 6, 1> > > vector_cartesian_velocities;

        /** Robot Kinematic Model **/
        boost::shared_ptr< threed_odometry::KinematicKDL > robotKinematics;

        /** Robot Motion Model **/
        boost::shared_ptr< threed_odometry::MotionModel<double> > motionModel;

        /**< Forward kinematics of the robot chains */
        std::vector<Eigen::Affine3d> fkRobotTrans;

        /**< Uncertainty of the forward kinematics (if any) */
        std::vector<base::Matrix6d> fkRobotCov;

        /** Covariance Joints, Slip and Contact Angle velocities NOTE: The order of the storage needs to be coincident if used as input for the motionModel **/
        Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > modelVelCov;

        /** Bessel Low-pass IIR filter for the Motion Model velocities
         * Specification of the Order and Data dimension is required */
        boost::shared_ptr< threed_odometry::IIR<NORDER_BESSEL_FILTER, 3> > bessel;

        /** Weighting Matrix for the Motion Model  **/
        WeightingMatrix WeightMatrix;

        /** Delta pose step **/
        ::base::samples::BodyState delta_pose;

        /***************************/
        /** Input port variables **/
        /***************************/

        ::base::samples::Joints joints_samples;

        ::base::samples::RigidBodyState orientation_samples;


        /***************************/
        /** Output port variables **/
        /***************************/

        /** Body Center w.r.t the World Coordinate system (using statistical Motion Model and IMU orientation) */
        ::base::samples::BodyState body_pose;


    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "exoter_odometry::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

        /** @brief Computes the velocities using
        *  the motion model.
        */
        void motionVelocities();

        /** @brief Performs the odometry update
         */
        void deadReckoning  (const double &delta_t);

        /** @brief
         */
        void joints_samplesUnpack(const ::base::samples::Joints &original_joints,
                                const std::vector<std::string> &order_names,
                                Eigen::Matrix< double, Eigen::Dynamic, 1  > &joint_positions,
                                Eigen::Matrix< double, Eigen::Dynamic, 1  > &joint_velocities);

        bool joints_samplesMotionModel(std::vector<std::string> &order_names,
                                const std::vector<std::string> &joint_names,
                                const std::vector<std::string> &slip_names,
                                const std::vector<std::string> &contact_names);


        /** \brief Store the variables in the Output ports
         */
        void outputPortPose();

        void outputPortContactPoints();

    public:

        template <typename _MatrixType>
        static _MatrixType guaranteeSPD (const _MatrixType &A)
        {
            _MatrixType spdA;
            Eigen::VectorXd s;
            s.resize(A.rows(), 1);

            /**
            * Single Value Decomposition
            */
            Eigen::JacobiSVD <Eigen::MatrixXd > svdOfA (A, Eigen::ComputeThinU | Eigen::ComputeThinV);

            s = svdOfA.singularValues(); //!eigenvalues

            #ifdef DEBUG_PRINTS
            std::cout<<"[SPD-SVD] s: \n"<<s<<"\n";
            std::cout<<"[SPD-SVD] svdOfA.matrixU():\n"<<svdOfA.matrixU()<<"\n";
            std::cout<<"[SPD-SVD] svdOfA.matrixV():\n"<<svdOfA.matrixV()<<"\n";

            Eigen::EigenSolver<_MatrixType> eig(A);
            std::cout << "[SPD-SVD] BEFORE: eigen values: " << eig.eigenvalues().transpose() << std::endl;
            #endif

            for (register int i=0; i<s.size(); ++i)
            {
                #ifdef DEBUG_PRINTS
                std::cout<<"[SPD-SVD] i["<<i<<"]\n";
                #endif

                if (s(i) < 0.00)
                    s(i) = 0.00;
            }
            spdA = svdOfA.matrixU() * s.matrix().asDiagonal() * svdOfA.matrixV().transpose();

            #ifdef DEBUG_PRINTS
            Eigen::EigenSolver<_MatrixType> eigSPD(spdA);
            if (eig.eigenvalues() == eigSPD.eigenvalues())
            std::cout<<"[SPD-SVD] EQUAL!!\n";

            std::cout << "[SPD-SVD] AFTER: eigen values: " << eigSPD.eigenvalues().transpose() << std::endl;
            #endif

            return spdA;
        };

        /**
         * boxminus operation on a manifold
         *
         * @param result @c vectview to the result
         * @param w      scalar part of input
         * @param vec    vector part of input
         * @param scale  scale result by this value
         * @param plus_minus_periodicity if true values @f$[w, vec]@f$ and @f$[-w, -vec]@f$ give the same result 
         */
        Eigen::Vector3d boxminus(const double &w, const Eigen::Vector3d& vec, const double &scale, bool plus_minus_periodicity)
        {
            Eigen::Vector3d result;

            double nv = vec.norm();
            if(nv < threed_odometry::tolerance<double>())
            {
                if(!plus_minus_periodicity)
                {
                    // find the maximal entry:
                    int i;
                    vec.maxCoeff(&i);
                    result = scale * std::atan2(0, w) * Eigen::Vector3d::Unit(i);
                    return result;
                }
                nv = threed_odometry::tolerance<double>();
            }
            double s = scale / nv * (plus_minus_periodicity ? std::atan(nv / w) : std::atan2(nv, w) );

            result = s * vec;

            return result;
        };

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    };
}

#endif

