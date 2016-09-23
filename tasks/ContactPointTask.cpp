/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ContactPointTask.hpp"
#include <odometry/ContactOdometry.hpp>
#include <base/Eigen.hpp>

using namespace odometry;

ContactPointTask::ContactPointTask(std::string const& name)
    : ContactPointTaskBase(name), contactOdometry(NULL)
{
}

ContactPointTask::ContactPointTask(std::string const& name, RTT::ExecutionEngine* engine)
    : ContactPointTaskBase(name, engine), contactOdometry(NULL)
{
}

ContactPointTask::~ContactPointTask()
{
}

void ContactPointTask::contact_samplesTransformerCallback(const base::Time &ts, const ::odometry::BodyContactState &contact_samples_sample)
{
    contactState = contact_samples_sample;
    gotContactState = true;
}

void ContactPointTask::orientation_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample)
{
    if(!gotContactState)
        return;

    Eigen::Affine3d tf; /** Transformer transformation **/

    /** Get the transformation (transformation) Tbody_imu **/
    if (_body_frame.value().compare(_imu_frame.value()) == 0)
    {
        tf.setIdentity();
    }
    else if (!_imu2body.get(ts, tf, false))
    {
        throw std::runtime_error("[CONTACT_POINT_ODOMETRY] Transformation from imu to body is not provided.");
        return;
    }

    Eigen::Quaternion <double> qtf = Eigen::Quaternion <double> (tf.rotation());//!Quaternion from Body to imu (transforming samples from imu to body)

    // get only the rotation component
    Eigen::Quaterniond R_body2World(qtf * orientation_samples_sample.orientation);

    contactOdometry->update(contactState, R_body2World);

    // create a transform with uncertainty based on the odometry 
    base::TransformWithCovariance body2PrevBody( 
            contactOdometry->getPoseDelta().toTransform(),
            contactOdometry->getPoseError() );

    pushState(ts, body2PrevBody, R_body2World);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ContactPointTask.hpp for more detailed
// documentation about them.

// bool ContactPointTask::configureHook()
// {
//     if (! ContactPointTaskBase::configureHook())
//         return false;
//     return true;
// }
bool ContactPointTask::startHook()
{
    if (! ContactPointTaskBase::startHook())
        return false;

    delete contactOdometry;
    contactOdometry = new odometry::FootContact(odometryConfiguration);

    gotContactState = false;

    return true;
}
// void ContactPointTask::updateHook()
// {
//     ContactPointTaskBase::updateHook();
// }
// void ContactPointTask::errorHook()
// {
//     ContactPointTaskBase::errorHook();
// }
// void ContactPointTask::stopHook()
// {
//     ContactPointTaskBase::stopHook();
// }
// void ContactPointTask::cleanupHook()
// {
//     ContactPointTaskBase::cleanupHook();
// }

