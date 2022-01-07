
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

#pragma once

// C++ Includes

// FRC includes

// Team 302 includes
#include <states/StateMgr.h>
#include <states/StateStruc.h>



// Third Party Includes

class ArmStateMgr : public StateMgr
{
    public:
        /// @enum the various states of the Arm
        enum ARM_STATE
        {
            GOING_UP,
            GOING_DOWN,
            UP_POS,
            DOWN_POS,
            HOLD_POSITION,
            MAX_ARM_STATES
        };
        
		/// @brief  Find or create the state manmanager
		/// @return ArmStateMgr* pointer to the state manager
		static ArmStateMgr* GetInstance();
        void CheckForStateTransition() override;

    private:

        ArmStateMgr();
        ~ArmStateMgr() = default;

		static ArmStateMgr*	m_instance;

        const StateStruc  m_goingUpState = {ARM_STATE::GOING_UP, StateType::ARM, false};
        const StateStruc  m_goingDownState = {ARM_STATE::GOING_DOWN, StateType::ARM, false};
        const StateStruc  m_upPosState = {ARM_STATE::UP_POS, StateType::ARM, false};
        const StateStruc  m_downPosState = {ARM_STATE::DOWN_POS, StateType::ARM, false};
        const StateStruc  m_holdState = {ARM_STATE::HOLD_POSITION, StateType::ARM, true};
};


