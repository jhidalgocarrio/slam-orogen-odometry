/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Generic.hpp"

using namespace odometry;

Generic::Generic(std::string const& name)
    : GenericBase(name)
{
}

Generic::Generic(std::string const& name, RTT::ExecutionEngine* engine)
    : GenericBase(name, engine)
{
}

Generic::~Generic()
{
}

void Generic::pushState(base::Time const& ts,
	envire::TransformWithUncertainty& body2PrevBody,
	base::Quaterniond const& R_body2World,
        base::Vector3d const& velocity,
        base::Vector3d const& angular_velocity)
{
    // Generate the output for the pose delta
    base::samples::RigidBodyState state;
    state.invalidate();
    state.time = ts;
    body2PrevBody.copyToRigidBodyState( state );
    state.angular_velocity = angular_velocity;
    state.velocity = velocity;

    // write to port if connected
    if( _odometry_delta_samples.connected() )
        _odometry_delta_samples.write( state );

    // also sum up the relative changes in an absolute odometry frame
    // and output to a separate port. 
    lastBody2Odometry = lastBody2Odometry * body2PrevBody;
    // need to set the absolute body to world orientation from the imu
    // because of potential drift, and a missing proper initial value
    Eigen::Affine3d body2Odometry = lastBody2Odometry.getTransform();
    body2Odometry.linear() = R_body2World.toRotationMatrix();
    lastBody2Odometry.setTransform( body2Odometry );

    // this will update the global rotation covariance to that of the IMU,
    // which is providing the error estimate itself
    // TODO remove hardcoded rotation covariance
    Eigen::Matrix<double,6,6> cov = lastBody2Odometry.getCovariance();
    cov.topLeftCorner<3,3>() = (Eigen::Vector3d( 2, 2, 10 ) * M_PI/180.0 ).array().square().matrix().asDiagonal();
    cov.bottomLeftCorner<3,3>().setZero();
    cov.topRightCorner<3,3>().setZero();
    lastBody2Odometry.setCovariance( cov );

    state.sourceFrame = _body_frame.get();
    state.targetFrame = _odometry_frame.get();
    lastBody2Odometry.copyToRigidBodyState( state );

    // write to port for summed odometry readings
    _odometry_samples.write( state );
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Generic.hpp for more detailed
// documentation about them.

// bool Generic::configureHook()
// {
//     if (! GenericBase::configureHook())
//         return false;
//     return true;
// }
bool Generic::startHook()
{
    if (! GenericBase::startHook())
        return false;

     // reset absolute odometry integration
     //lastBody2Odometry = envire::TransformWithUncertainty::Identity();

    //printf("[Odoemtry] starting with %g/%g/%g\n", start_rbs.position[0], start_rbs.position[1], start_rbs.position[2]);
    lastBody2Odometry = envire::TransformWithUncertainty(start_rbs);
    return true;
}
// void Generic::updateHook()
// {
//     GenericBase::updateHook();
// }
// void Generic::errorHook()
// {
//     GenericBase::errorHook();
// }
// void Generic::stopHook()
// {
//     GenericBase::stopHook();
// }
// void Generic::cleanupHook()
// {
//     GenericBase::cleanupHook();
// }

bool Generic::setStart_pose(::base::samples::RigidBodyState const & value)
{
    start_rbs = value;
    //printf("[Odometry] changed start pos to %g/%g/%g\n", start_rbs.position[0], start_rbs.position[1], start_rbs.position[2]);
    return(odometry::GenericBase::setStart_pose(value));
}
