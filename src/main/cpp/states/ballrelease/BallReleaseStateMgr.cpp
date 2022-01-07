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
#include <map>
#include <memory>
#include <vector>

// FRC includes
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

// Team 302 includes
#include <states/IState.h>
#include <states/ballrelease/BallReleaseStateMgr.h>
#include <xmlmechdata/StateDataDefn.h>
#include <controllers/MechanismTargetData.h>
#include <utils/Logger.h>
#include <gamepad/TeleopControl.h>
#include <states/BallRelease/BallReleaseState.h>
#include <subsys/MechanismFactory.h>
#include <subsys/MechanismTypes.h>
#include <states/StateMgr.h>
#include <states/StateStruc.h>


// Third Party Includes

using namespace std;


BallReleaseStateMgr* BallReleaseStateMgr::m_instance = nullptr;
BallReleaseStateMgr* BallReleaseStateMgr::GetInstance()
{
	if ( BallReleaseStateMgr::m_instance == nullptr )
	{
		BallReleaseStateMgr::m_instance = new BallReleaseStateMgr();
	}
	return BallReleaseStateMgr::m_instance;
}


/// @brief    initialize the state manager, parse the configuration file and create the states.
BallReleaseStateMgr::BallReleaseStateMgr() 
{
    map<string, StateStruc> stateMap;
    stateMap["BALLRELEASEHOLD"] = m_holdState;
    stateMap["BALLRELEASEOPEN"] = m_releaseState;

    Init(MechanismFactory::GetMechanismFactory()->GetBallRelease(), stateMap);
}

/// @brief  run the current state
/// @return void
void BallReleaseStateMgr::CheckForStateTransition()
{
    if ( MechanismFactory::GetMechanismFactory()->GetBallRelease() != nullptr )
    {
        // process teleop/manual interrupts
        auto currentState = static_cast<BALL_RELEASE_STATE>(GetCurrentState());
        
        auto controller = TeleopControl::GetInstance();
        if ( controller != nullptr )
        {
            auto releasePressed = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::RELEASE);
            if (releasePressed  &&  currentState != BALL_RELEASE_STATE::RELEASE )
            {
                SetCurrentState( BALL_RELEASE_STATE::RELEASE, false );
            }
            else if (!releasePressed && currentState != BALL_RELEASE_STATE::HOLD)
            {
                SetCurrentState(BALL_RELEASE_STATE::HOLD, false);
            }
        }
    }
}





