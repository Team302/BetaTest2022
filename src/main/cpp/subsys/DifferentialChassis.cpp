#include <subsys/DifferentialChassis.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/drive/DifferentialDrive.h>

using namespace std;

DifferentialChassis::DifferentialChassis
(
    shared_ptr<IDragonMotorController>              leftMotor, 
    shared_ptr<IDragonMotorController>              rightMotor,
    units::meter_t                                  trackWidth,
    units::velocity::meters_per_second_t            maxSpeed,
    units::angular_velocity::degrees_per_second_t   maxAngSpeed,
    units::length::inch_t                           wheelDiameter,
    string                                          networkTableName,
    string                                          controlFileName
) : m_leftMotor(leftMotor),
    m_rightMotor(rightMotor),
    m_maxSpeed(maxSpeed),
    m_maxAngSpeed(maxAngSpeed),
    m_wheelDiameter(wheelDiameter),
    m_track(trackWidth),
    m_kinematics(new frc::DifferentialDriveKinematics(trackWidth)),
    m_differentialOdometry(new frc::DifferentialDriveOdometry(frc::Rotation2d(), frc::Pose2d())),
    m_ntName(networkTableName),
    m_controlFileName(controlFileName)
{
    
}
//Moves the robot
void DifferentialChassis::Drive(frc::ChassisSpeeds chassisSpeeds)
{
    auto wheels = m_kinematics->ToWheelSpeeds(chassisSpeeds);
    wheels.Desaturate(m_maxSpeed);
    if (m_leftMotor.get() != nullptr)
    {
        m_leftMotor.get()->Set(wheels.left/m_maxSpeed);
    }
    if (m_rightMotor.get() != nullptr)
    {
        m_rightMotor.get()->Set(wheels.right/m_maxSpeed);
    }
}

frc::Pose2d DifferentialChassis::GetPose() const
{
    return frc::Pose2d();
}

void DifferentialChassis::ResetPose(const frc::Pose2d& pose)
{

}

void DifferentialChassis::UpdatePose()
{
    
}

units::velocity::meters_per_second_t DifferentialChassis::GetMaxSpeed() const
{
    return m_maxSpeed;
}

units::angular_velocity::radians_per_second_t DifferentialChassis::GetMaxAngularSpeed() const
{
    return m_maxAngSpeed;
}

units::length::inch_t DifferentialChassis::GetWheelDiameter() const
{
    return units::length::inch_t(4);
}    

units::length::inch_t DifferentialChassis::GetTrack() const
{
    return m_track;
}
bool DifferentialChassis::IsMoving() const
{
    return false;
}