/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "LatOdom.hpp"
#include <sstream>

using namespace odometry;

LatOdom::LatOdom(std::string const& name)
    : LatOdomBase(name)
{
}

LatOdom::LatOdom(std::string const& name, RTT::ExecutionEngine* engine)
    : LatOdomBase(name, engine)
{
}

LatOdom::~LatOdom()
{
}

void LatOdom::printInvalidSample()
{
    std::cerr << "Invalid actuator sample:" << std::endl;
    std::cerr << "  Expected the following joint names:" << std::endl;
    std::stringstream s;
    std::copy(leftWheelNames.begin(),leftWheelNames.end(), std::ostream_iterator<std::string>(s,", "));
    std::copy(rightWheelNames.begin(),rightWheelNames.end(), std::ostream_iterator<std::string>(s,", "));
    std::copy(leftSteeringNames.begin(),leftSteeringNames.end(), std::ostream_iterator<std::string>(s,", "));
    std::copy(rightSteeringNames.begin(),rightSteeringNames.end(), std::ostream_iterator<std::string>(s,", "));
    std::cerr << "    " << s.str() << std::endl;
    std::cerr << "  But got the following joint names:" << std::endl;
    std::stringstream s2;
    std::copy(currentActuatorSample.names.begin(),currentActuatorSample.names.end(), std::ostream_iterator<std::string>(s2,", "));
    std::cerr << "    " << s2.str() << std::endl;
}

void LatOdom::actuator_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Joints &actuator_samples)
{
    currentActuatorSample = actuator_samples;
    actuatorUpdated = true;

    // use the transformer to get the body2world transformation 
    // this should include the imu reading
    base::Transform3d body2IMUWorld;
    if( !_body2imu_world.get( ts, body2IMUWorld ) )
        return;

    // calculates the rotation from body to world base on the orientation measurment 
    Eigen::Quaterniond R_body2World(body2IMUWorld.rotation()); 

    try
    {
        odometry->update(currentActuatorSample, currentActuatorSample, R_body2World);
    }
    catch(const base::NamedVector<base::JointState>::InvalidName &e)
    {
        printInvalidSample();
        error(EXCEPTION);
    }

    // create a transform with uncertainty based on the odometry 
    envire::TransformWithUncertainty body2PrevBody( 
            odometry->getPoseDelta().toTransform(),
            odometry->getPoseError() );

    // push the transformations
    pushState( ts, body2PrevBody, R_body2World );
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See LatOdom.hpp for more detailed
// documentation about them.

bool LatOdom::configureHook()
{
    if (! LatOdomBase::configureHook())
        return false;

    rightWheelNames = _rightWheelNames.get();
    leftWheelNames = _leftWheelNames.get();
    
    rightSteeringNames = _rightSteeringNames.get();
    leftSteeringNames = _leftSteeringNames.get();

    wheelRadius = _wheelRadiusAvg.value();

    odometry = boost::shared_ptr<odometry::SkidOdometry>(new odometry::SkidOdometry(
            _odometry_config.get(),
            _wheelRadiusAvg.get(),
            _trackWidth.get(),
            _wheelBase.get(),
            leftWheelNames, rightWheelNames, leftSteeringNames, rightSteeringNames));

    return true;
}
bool LatOdom::startHook()
{
    if (! LatOdomBase::startHook())
        return false;
    
    prev_ts = base::Time();
    actuatorUpdated = false;
    
    return true;
}
void LatOdom::updateHook()
{
    LatOdomBase::updateHook();
}
void LatOdom::errorHook()
{
    LatOdomBase::errorHook();
}
void LatOdom::stopHook()
{
    LatOdomBase::stopHook();
}
void LatOdom::cleanupHook()
{
    LatOdomBase::cleanupHook();
}
