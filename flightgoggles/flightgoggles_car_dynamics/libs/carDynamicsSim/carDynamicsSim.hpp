/**
 * @file carDynamicsSim.hpp
 * @author Ezra Tal
 * @brief Car dynamics simulator class header file
 * 
 */

#ifndef CARDYNAMICSSIM_H
#define CARDYNAMICSSIM_H

#include <random>
#include <Eigen/Dense>
#include <Eigen/Geometry>

/**
 * @brief Car dynamics simulator class
 * 
 */
class CarDynamicsSim{
    public:
        CarDynamicsSim(
            const double maxSteeringAngle,
            const double minSpeed,
            const double maxSpeed,
            const Eigen::Vector2d & velProcNoiseAutoCorr);

        void setNoiseSeed(
            const unsigned seed);

        void setVehicleProperties(
            const double maxSteeringAngle,
            const double minSpeed,
            const double maxSpeed,
            const Eigen::Vector2d & velProcNoiseAutoCorr);

        void setVehicleState(
            const Eigen::Vector2d & position,
            const Eigen::Vector2d & velocity,
            const Eigen::Rotation2Dd & heading);

        void getVehicleState(
            Eigen::Vector2d & position,
            Eigen::Vector2d & velocity,
            Eigen::Rotation2Dd & heading);

        void setVehiclePosition(const Eigen::Vector2d & position);
        void setVehicleVelocity(const Eigen::Vector2d & velocity);
        void setVehicleHeading(const Eigen::Rotation2Dd & heading);
        Eigen::Vector2d getVehiclePosition(void);
        Eigen::Vector2d getVehicleVelocity(void);
        Eigen::Rotation2Dd getVehicleHeading(void);

        void proceedState_ExplicitEuler(
            const double dt_secs,
            const double speed,
            const double steeringAngle);

    private:
        /// @name Vehicle properties
        //@{
        double maxSteeringAngle_;
        double minSpeed_;
        double maxSpeed_;
        Eigen::Vector2d velocityProcessNoiseAutoCorrelation_;
        //@}

        /// @name Vehicle state
        //@{
        Eigen::Vector2d position_;
        Eigen::Vector2d velocity_;
        Eigen::Rotation2Dd heading_;
        //@}

        /// @name Std normal RNG
        //@{
        std::default_random_engine randomNumberGenerator_;
        std::normal_distribution<double> standardNormalDistribution_ = std::normal_distribution<double>(0.0,1.0);
        //@}

        Eigen::Vector2d getPositionDerivative(void);
        double getHeadingDerivative(const double steeringAngle);
};

#endif // CARDYNAMICSSIM_H