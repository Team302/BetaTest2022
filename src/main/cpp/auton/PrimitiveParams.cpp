/*
 * PrimitiveParams.cpp
 *
 *  Created on: Jan 17, 2017
 *      Author: Jonah Shader
 */

#include <auton/PrimitiveEnums.h>
#include <auton/PrimitiveParams.h>
#include <states/arm/ArmStateMgr.h>
#include <states/ballrelease/BallReleaseStateMgr.h>
#include <states/balltransfer/BallTransferStateMgr.h>
#include <states/intake/IntakeStateMgr.h>

PrimitiveParams::PrimitiveParams
(
    PRIMITIVE_IDENTIFIER								id,
    float                       						time,
    float                       						distance,
    float                       						xLoc,
    float                       						yLoc,
    float                       						heading,
    float                       						startDriveSpeed,
    float                       						endDriveSpeed,
	std::string											pathName,
	IntakeStateMgr::INTAKE_STATE                        intakeState,
	BallTransferStateMgr::BALL_TRANSFER_STATE           transferState,
	ArmStateMgr::ARM_STATE                              armState,
	BallReleaseStateMgr::BALL_RELEASE_STATE             releaseState
):	//Pass over parameters to class variables
		m_id(id), //Primitive ID
		m_time(time),
		m_distance(distance),
		m_xLoc( xLoc ),
		m_yLoc( yLoc ),
		m_heading(heading),
		m_startDriveSpeed( startDriveSpeed ),
		m_endDriveSpeed( endDriveSpeed ),
		m_pathName ( pathName),
		m_intakeState(intakeState),
		m_transferState(transferState),
		m_armState(armState),
		m_releaseState(releaseState)
{
}

PRIMITIVE_IDENTIFIER PrimitiveParams::GetID() const
{
	return m_id;
}

float PrimitiveParams::GetTime() const
{
	return m_time;
}

float PrimitiveParams::GetDistance() const
{
	return m_distance;
}

float PrimitiveParams::GetXLocation() const
{
    return m_xLoc;
}

float PrimitiveParams::GetYLocation() const
{
    return m_yLoc;
}

float PrimitiveParams::GetHeading() const
{
	return m_heading;
}

float PrimitiveParams::GetDriveSpeed() const
{
    return m_startDriveSpeed;
}

float PrimitiveParams::GetEndDriveSpeed() const
{
    return m_endDriveSpeed;
}

std::string PrimitiveParams::GetPathName() const
{
	return m_pathName;
}

//Setters
void PrimitiveParams::SetDistance(float distance)
{
	m_distance = distance;
}

IntakeStateMgr::INTAKE_STATE PrimitiveParams::GetIntakeState() const
{
	return m_intakeState;
}
BallTransferStateMgr::BALL_TRANSFER_STATE PrimitiveParams::GetTransferState() const
{
	return m_transferState;
}
ArmStateMgr::ARM_STATE PrimitiveParams::GetArmState() const
{
	return m_armState;
}
BallReleaseStateMgr::BALL_RELEASE_STATE PrimitiveParams::GetReleaseState() const
{
	return m_releaseState;
}

