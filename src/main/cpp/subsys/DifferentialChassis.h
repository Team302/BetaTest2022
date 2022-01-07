#pragma once
#include <string>
#include <units/velocity.h>
#include <units/angular_velocity.h>

#include <subsys/interfaces/IChassis.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/drive/DifferentialDrive.h>

class DifferentialChassis : public IChassis 
{

    public:
        DifferentialChassis() = delete;
        virtual ~DifferentialChassis() = default;

        DifferentialChassis
        (
            std::shared_ptr<IDragonMotorController>         leftMotor, 
            std::shared_ptr<IDragonMotorController>         rightMotor,
            units::meter_t                                  trackWidth,
            units::velocity::meters_per_second_t            maxSpeed,
            units::angular_velocity::degrees_per_second_t   maxAngSpeed,
            units::length::inch_t                           wheelDiameter,
            std::string				                        networkTableName,
			std::string                                     controlFileName
        );

        /// @brief      return the chassis type
        /// @returns    CHASSIS_TYPE
        IChassis::CHASSIS_TYPE GetType() const override {return IChassis::CHASSIS_TYPE::DIFFERENTIAL;}

        void Drive(frc::ChassisSpeeds chassisSpeeds) override;

        frc::Pose2d GetPose() const override;
        void ResetPose(const frc::Pose2d& pose) override;
        void UpdatePose() override;
        units::velocity::meters_per_second_t GetMaxSpeed() const override;
        units::angular_velocity::radians_per_second_t GetMaxAngularSpeed() const override;

        units::length::inch_t GetWheelDiameter() const override ;
        units::length::inch_t GetTrack() const override;

        bool IsMoving() const override;

    private:
        std::shared_ptr<IDragonMotorController> m_leftMotor;
        std::shared_ptr<IDragonMotorController> m_rightMotor;
        
        units::velocity::meters_per_second_t m_maxSpeed;
        units::angular_velocity::degrees_per_second_t m_maxAngSpeed;
        units::length::inch_t   m_wheelDiameter;
        units::length::inch_t   m_track;

        frc::DifferentialDriveKinematics*   m_kinematics;
        frc::DifferentialDriveOdometry*     m_differentialOdometry;
        std::string                         m_ntName;
        std::string                         m_controlFileName;

};