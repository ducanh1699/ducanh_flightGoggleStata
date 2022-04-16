/**
 * @file carDynamicsSim.cpp
 * @author Ezra Tal
 * @brief Car dynamics simulator class implementation
 * 
 */
#include "carDynamicsSim.hpp"
#include <iostream>
#include <chrono>

template<class T>
const T& clamp( const T& v, const T& lo, const T& hi )
{
    assert( !(hi < lo) );
    return (v < lo) ? lo : (hi < v) ? hi : v;
}

/**
 * @brief Construct a new Car Dynamics Sim object
 *
 * @param maxSteeringAngle Maximum steering angle, which equal maximum rotation rate.
 * @param minSpeed Minimum forward speed.
 * @param maxSpeed Maximum forward speed.
 */
CarDynamicsSim::CarDynamicsSim(
    const double maxSteeringAngle,
    const double minSpeed,
    const double maxSpeed,
    const Eigen::Vector2d & velProcNoiseAutoCorr)
{
    setVehicleProperties(maxSteeringAngle, minSpeed, maxSpeed, velProcNoiseAutoCorr);
    randomNumberGenerator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
}

/**
 * @brief Set RNG seed
 *
 * @param seed Seed value.
 */
void CarDynamicsSim::setNoiseSeed(
    const unsigned seed)
{
    randomNumberGenerator_.seed(seed);
}


/**
 * @brief Set vehicle properties of Car Dynamics Sim object
 *
 * @param maxSteeringAngle Maximum steering angle, which equal maximum rotation rate.
 * @param minSpeed Minimum forward speed.
 * @param maxSpeed Maximum forward speed.
 */
void CarDynamicsSim::setVehicleProperties(
    const double maxSteeringAngle,
    const double minSpeed,
    const double maxSpeed,
    const Eigen::Vector2d & velProcNoiseAutoCorr)
{
    maxSteeringAngle_ = maxSteeringAngle;
    minSpeed_ = minSpeed;
    maxSpeed_ = maxSpeed;
    velocityProcessNoiseAutoCorrelation_ = velProcNoiseAutoCorr;
}

/**
 * @brief Set vehicle state
 *
 * @param position Position in 2D world-fixed reference frame.
 * @param heading Heading angle wrt world-fixed reference frame.
 */
void CarDynamicsSim::setVehicleState(
    const Eigen::Vector2d & position,
    const Eigen::Vector2d & velocity,
    const Eigen::Rotation2Dd & heading)
{
    position_ = position;
    velocity_ = velocity;
    heading_ = heading;
}

/**
 * @brief Get vehicle state
 *
 * @param position Position in 2D world-fixed reference frame output.
 * @param velocity Velocity in 2D vehicle-fixed reference frame output.
 * @param heading Heading wrt world-fixed reference frame output.
 */
void CarDynamicsSim::getVehicleState(
    Eigen::Vector2d & position,
    Eigen::Vector2d & velocity,
    Eigen::Rotation2Dd & heading)
{
    position = position_;
    velocity = velocity_;
    heading = heading_;
}

/**
 * @brief Set vehicle position
 * 
 * @param Eigen::Vector2d Position in 2D world-fixed reference frame.
 */
void CarDynamicsSim::setVehiclePosition(const Eigen::Vector2d & position)
{
    position_ = position;
}

/**
 * @brief Set vehicle velocity
 * 
 * @param Eigen::Vector2d Velocity in body-fixed reference frame.
 */
void CarDynamicsSim::setVehicleVelocity(const Eigen::Vector2d & velocity)
{
    velocity_ = velocity;
}

/**
 * @brief Set vehicle heading
 * 
 * @param Eigen::Rotation2Dd Heading wrt world-fixed reference frame output.
 */
void CarDynamicsSim::setVehicleHeading(const Eigen::Rotation2Dd & heading)
{
    heading_ = heading;
}

/**
 * @brief Get vehicle position
 * 
 * @return Eigen::Vector2d Position in 2D world-fixed reference frame.
 */
Eigen::Vector2d CarDynamicsSim::getVehiclePosition(void)
{
    return position_;
}

/**
 * @brief Get vehicle velocity
 * 
 * @return Eigen::Vector2d Velocity in body-fixed reference frame.
 */
Eigen::Vector2d CarDynamicsSim::getVehicleVelocity(void)
{
    return velocity_;
}

/**
 * @brief Get vehicle heading
 * 
 * @return Eigen::Rotation2Dd Heading wrt world-fixed reference frame output.
 */
Eigen::Rotation2Dd CarDynamicsSim::getVehicleHeading(void)
{
    return heading_;
}

/**
 * @brief Get position derivative
 * 
 * @return Eigen::Vector2d Position derivative in 2D world-fixed reference frame.
 */
Eigen::Vector2d CarDynamicsSim::getPositionDerivative(void)
{
    return heading_*velocity_;
}

/**
 * @brief Get heading derivative
 * 
 * @return double Heading derivative wrt world-fixed reference frame.
 */
double CarDynamicsSim::getHeadingDerivative(const double steeringAngle)
{
    return clamp(steeringAngle, -maxSteeringAngle_, maxSteeringAngle_);
}

/**
 * @brief Proceed vehicle dynamics using Explicit Euler integration
 * 
 * @param dt_secs Time step
 * @param motorSpeedCommand Motor speed commands 
 */
void CarDynamicsSim::proceedState_ExplicitEuler(
    const double dt_secs,
    const double speed,
    const double steeringAngle)
{
    if(dt_secs <= 0.) return;

    Eigen::Vector2d saturatedSpeed;
    saturatedSpeed << clamp(speed, minSpeed_, maxSpeed_), 0.;

    Eigen::Vector2d stochasticVelocity;
    stochasticVelocity << sqrt(velocityProcessNoiseAutoCorrelation_(0)/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
                          sqrt(velocityProcessNoiseAutoCorrelation_(1)/dt_secs)*standardNormalDistribution_(randomNumberGenerator_);

    setVehicleVelocity(saturatedSpeed + stochasticVelocity);

    position_ += dt_secs*getPositionDerivative();
    heading_ *= Eigen::Rotation2Dd(dt_secs*getHeadingDerivative(steeringAngle));
}