
//====================================================================================================================================================
// Copyright 2022 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

// C++ Includes
#include <string>
#include <memory>
#include <iostream>

// FRC includes
#include <frc/Timer.h>
#include <frc/DriverStation.h>

// Team 302 includes
#include <auton/primitives/IPrimitive.h>
#include <auton/CyclePrimitives.h>
#include <auton/PrimitiveFactory.h>
#include <auton/AutonSelector.h>
#include <auton/PrimitiveEnums.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <auton/PrimitiveParser.h>
#include <auton/PrimitiveParams.h>
#include <subsys/MechanismFactory.h>
#include <utils/Logger.h>
#include <states/arm/ArmStateMgr.h>
#include <states/ballrelease/BallReleaseStateMgr.h>
#include <states/balltransfer/BallTransferStateMgr.h>
#include <states/intake/IntakeStateMgr.h>

// Third Party Includes

using namespace frc;
using namespace std;

CyclePrimitives::CyclePrimitives() : m_primParams(), 
									 m_currentPrimSlot(0), 
								     m_currentPrim(nullptr), 
									 m_primFactory(
									 PrimitiveFactory::GetInstance()), 
									 m_doNothing(nullptr), 
									 m_autonSelector( new AutonSelector()) ,
									 m_timer( make_unique<Timer>()),
									 m_maxTime( 0.0 ),
									 m_isDone( false ),
									 m_intake(IntakeStateMgr::GetInstance()),
									 m_transfer(BallTransferStateMgr::GetInstance()),
									 m_arm(ArmStateMgr::GetInstance()),
									 m_release(BallReleaseStateMgr::GetInstance()),
									 m_currIntakeState(IntakeStateMgr::INTAKE_STATE::OFF),
									 m_currTransferState(BallTransferStateMgr::BALL_TRANSFER_STATE::OFF),
									 m_currArmState(ArmStateMgr::ARM_STATE::HOLD_POSITION),
									 m_currReleaseState(BallReleaseStateMgr::BALL_RELEASE_STATE::HOLD)
{
}

void CyclePrimitives::Init()
{
	m_currentPrimSlot = 0; //Reset current prim
	m_primParams.clear();

	m_primParams = PrimitiveParser::ParseXML( m_autonSelector->GetSelectedAutoFile() );
	if (!m_primParams.empty())
	{
		GetNextPrim();
	}
}

void CyclePrimitives::Run()
{
	if (DriverStation::IsAutonomousEnabled())
	{
		if (m_currentPrim != nullptr)
		{
			m_currentPrim->Run();
			if(m_intake != nullptr)
			{
				m_intake->SetCurrentState(m_currIntakeState, true);
			}
			if(m_transfer != nullptr)
			{
				m_transfer->SetCurrentState(m_currTransferState, true);
			}
			if(m_arm != nullptr)
			{
				m_arm->SetCurrentState(m_currArmState, true);
			}
			if(m_release != nullptr)
			{
				m_release->SetCurrentState(m_currReleaseState, true);
			}
		}
		else
		{
			Logger::GetLogger()->LogError(string("CyclePrimitive"), string("Completed"));
			m_isDone = true;
			m_primParams.clear();	// clear the primitive params vector
			m_currentPrimSlot = 0;  //Reset current prim slot
			RunDoNothing();
		}
	}
}

bool CyclePrimitives::AtTarget() const
{
	return m_isDone;
}

void CyclePrimitives::GetNextPrim()
{
	PrimitiveParams* currentPrimParam = (m_currentPrimSlot < (int) m_primParams.size()) ? m_primParams[m_currentPrimSlot] : nullptr;

	m_currentPrim = (currentPrimParam != nullptr) ? m_primFactory->GetIPrimitive(currentPrimParam) : nullptr;
	if (m_currentPrim != nullptr)
	{
		m_currentPrim->Init(currentPrimParam);
		m_maxTime = currentPrimParam->GetTime();
		m_timer->Reset();
		m_timer->Start();
		m_currIntakeState = currentPrimParam->GetIntakeState();
		m_currTransferState = currentPrimParam->GetTransferState();
		m_currArmState = currentPrimParam->GetArmState();
		m_currReleaseState = currentPrimParam->GetReleaseState();
	}

	m_currentPrimSlot++;
}

void CyclePrimitives::RunDoNothing()
{
	if (m_doNothing == nullptr)
	{	
		auto time = DriverStation::GetMatchTime();
		auto params = new PrimitiveParams( DO_NOTHING,          // identifier
		                                   time,              	// time
		                                   0.0,                 // distance
		                                   0.0,                 // target x location
		                                   0.0,                 // target y location
		                                   0.0,                 // heading
		                                   0.0,                 // start drive speed
		                                   0.0,					// end drive speed
										  std::string(),
										  IntakeStateMgr::INTAKE_STATE::OFF,
										  BallTransferStateMgr::BALL_TRANSFER_STATE::OFF,
										  ArmStateMgr::ARM_STATE::HOLD_POSITION,
										  BallReleaseStateMgr::BALL_RELEASE_STATE::HOLD );             
		m_doNothing = m_primFactory->GetIPrimitive(params);
		m_doNothing->Init(params);
	}
	m_doNothing->Run();
}
