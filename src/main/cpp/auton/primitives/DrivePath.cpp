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

//C++
#include <string>
#include <iostream>

//FRC Includes
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/Filesystem.h>
#include <units/angular_velocity.h>

// 302 Includes
#include <auton/primitives/DrivePath.h>
#include <subsys/ChassisFactory.h>
#include <utils/Logger.h>

#include <wpi/fs.h>

using namespace std;
using namespace frc;

using namespace wpi::math;

DrivePath::DrivePath() : m_chassis(ChassisFactory::GetChassisFactory()->GetIChassis()),
                         m_timer(make_unique<Timer>()),
                         m_currentChassisPosition(m_chassis.get()->GetPose()),
                         m_trajectory(),
                         m_runHoloController(false),
                         m_ramseteController(),
                         m_holoController(frc2::PIDController{1, 0, 0},
                                          frc2::PIDController{1, 0, 0},
                                          frc::ProfiledPIDController<units::radian>{1, 0, 0,
                                                                                    frc::TrapezoidProfile<units::radian>::Constraints{6.28_rad_per_s, 3.14_rad_per_s / 1_s}}),
                         //max velocity of 1 rotation per second and a max acceleration of 180 degrees per second squared.
                         m_PrevPos(m_chassis.get()->GetPose()),
                         m_PosChgTimer(make_unique<Timer>()),
                         m_timesRun(0),
                         m_targetPose(),
                         m_deltaX(0.0),
                         m_deltaY(0.0),
                         m_trajectoryStates(),
                         m_desiredState()

{
    m_trajectoryStates.clear();
}
void DrivePath::Init(PrimitiveParams *params)
{
    auto m_pathname = params->GetPathName(); //Grabs path name from auton xml

    Logger::GetLogger()->ToNtTable("DrivePath" + m_pathname, "Initialized", "False");
    Logger::GetLogger()->ToNtTable("DrivePath" + m_pathname, "Running", "False");
    Logger::GetLogger()->ToNtTable("DrivePath" + m_pathname, "Done", "False");
    Logger::GetLogger()->ToNtTable("DrivePath" + m_pathname, "WhyDone", "Not done");
    Logger::GetLogger()->ToNtTable("DrivePath" + m_pathname, "Times Ran", 0);

    m_trajectoryStates.clear(); //Clears the primitive of previous path/trajectory

    m_wasMoving = false;

    Logger::GetLogger()->ToNtTable("DrivePath" + m_pathname, "Initialized", "True"); //Signals that drive path is initialized in the console

    GetTrajectory(params->GetPathName());  //Parses path from json file based on path name given in xml
    
    Logger::GetLogger()->ToNtTable(m_pathname + "Trajectory", "Time", m_trajectory.TotalTime().to<double>());// Debugging
    
    if (!m_trajectoryStates.empty()) // only go if path name found
    {
        m_desiredState = m_trajectoryStates.front(); //m_desiredState is the first state, or starting position

        m_timer.get()->Reset(); //Restarts and starts timer
        m_timer.get()->Start();

        Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentPosX", m_currentChassisPosition.X().to<double>());
        Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentPosY", m_currentChassisPosition.Y().to<double>());

        Logger::GetLogger()->ToNtTable("Deltas", "iDeltaX", "0");
        Logger::GetLogger()->ToNtTable("Deltas", "iDeltaX", "0");

        //A timer used for position change detection
        m_PosChgTimer.get()->Reset(); 
        m_PosChgTimer.get()->Start(); // start scan timer to detect motion

        //Is used to determine what controller/ "drive mode" pathweaver will run in
        //Holo / Holonomic = Swerve X, y, z movement   Ramsete = Differential / Tank x, y movement
        if (m_runHoloController)
        {
            m_holoController.SetEnabled(true);
        }
        else
        {
            m_ramseteController.SetEnabled(true);
        }


        //Sampling means to grab a state based on the time, if we want to know what state we should be running at 5 seconds,
        //we will sample the 5 second state.
        auto targetState = m_trajectory.Sample(m_trajectory.TotalTime());  //"Samples" or grabs the position we should be at based on time

        m_targetPose = targetState.pose;  //Target pose represents the pose that we want to be at, based on the target state from above

        auto currPose = m_chassis.get()->GetPose(); //Grabs the current pose of the robot to compare to the target pose
        auto trans = m_targetPose - currPose; //Translation / Delta of the target pose and current pose

        m_deltaX = trans.X().to<double>();  //Separates the delta "trans" from above into two variables for x and y
        m_deltaY = trans.Y().to<double>();

        //m_chassis.get()->RunWPIAlgorithm(true); //Determines what pose estimation method we will use, we have a few using different methods/equations
    }
    m_timesRun = 0;
}
void DrivePath::Run()
{
    Logger::GetLogger()->ToNtTable("DrivePath" + m_pathname, "Running", "True");

    if (!m_trajectoryStates.empty()) //If we have a path parsed / have states to run
    {
        // debugging
        m_timesRun++;
        
        Logger::GetLogger()->ToNtTable("DrivePath" + m_pathname, "Times Ran", m_timesRun);

        // calculate where we are and where we want to be
        CalcCurrentAndDesiredStates();

        // Use the controller to calculate the chassis speeds for getting there
        auto refChassisSpeeds = m_runHoloController ? m_holoController.Calculate(m_currentChassisPosition, m_desiredState, m_desiredState.pose.Rotation()) :
                                                      m_ramseteController.Calculate(m_currentChassisPosition, m_desiredState);

        // debugging
        Logger::GetLogger()->ToNtTable("DrivePathValues", "ChassisSpeedsX", refChassisSpeeds.vx());
        Logger::GetLogger()->ToNtTable("DrivePathValues", "ChassisSpeedsY", refChassisSpeeds.vy());
        Logger::GetLogger()->ToNtTable("DrivePathValues", "ChassisSpeedsZ", units::degrees_per_second_t(refChassisSpeeds.omega()).to<double>());

        // Run the chassis
        m_chassis->Drive(refChassisSpeeds);
    }
    else //If we don't have states to run, don't move the robot
    {
        ChassisSpeeds speeds;
        speeds.vx = 0_mps;
        speeds.vy = 0_mps;
        speeds.omega = units::angular_velocity::radians_per_second_t(0);
        m_chassis->Drive(speeds);
    }

}

bool DrivePath::IsDone() //Default primitive function to determine if the primitive is done running
{

    bool isDone = false;
    string whyDone = ""; //debugging variable that we used to determine why the path was stopping
    
    if (!m_trajectoryStates.empty()) //If we have states... 
    {
        // Check if the current pose and the trajectory's final pose are the same
        auto curPos = m_chassis.get()->GetPose();
        //isDone = IsSamePose(curPos, m_targetPose, 100.0);
        if (IsSamePose(curPos, m_targetPose, 100.0))
        {
            isDone = true;
            whyDone = "Current Pose = Trajectory final pose";
        }
        

        
        if ( !isDone )
        {
            // Now check if the current pose is getting closer or farther from the target pose 
            auto trans = m_targetPose - curPos;
            auto thisDeltaX = trans.X().to<double>();
            auto thisDeltaY = trans.Y().to<double>();
            if (abs(thisDeltaX) < m_deltaX && abs(thisDeltaY) < m_deltaY)
            {   // Getting closer so just update the deltas
                m_deltaX = thisDeltaX;
                m_deltaY = thisDeltaY;
            }
            else
            {   // Getting farther away:  determine if it is because of the path curvature (not straight line between start and the end)
                // or because we went past the target (in this case, we are done)
                // Assume that once we get within a tenth of a meter (just under 4 inches), if we get
                // farther away we are passing the target, so we should stop.  Otherwise, keep trying.
                isDone = ((abs(m_deltaX) < 0.1 && abs(m_deltaY) < 0.1));
                if ((abs(m_deltaX) < 0.1 && abs(m_deltaY) < 0.1))
                {
                    whyDone = "Within 4 inches of target or getting farther away from target";
                }
            }
        }       
        

        


        if (m_PosChgTimer.get()->Get() > 1_s)//This if statement makes sure that we aren't checking for position change right at the start
        {                                    //caused problems that would signal we are done when the path hasn't started
           //auto moving = !IsSamePose(curPos, m_PrevPos, 7.5);
            auto moving = m_chassis.get()->IsMoving();  //Is moving checks to make sure we are moving
            if (!moving && m_wasMoving)  //If we aren't moving and last state we were moving, then...
            {
                    isDone = true;
                    whyDone = "Stopped moving";                    
            }
            m_PrevPos = curPos;
            m_wasMoving = moving;
        }

        // finally, do it based on time (we have no more states);  if we want to keep 
        // going, we need to understand where in the trajectory we are, so we can generate
        // a new state.
        if (!isDone)
        {
            //return (units::second_t(m_timer.get()->Get()) >= m_trajectory.TotalTime()); 
        }
    }
    else
    {
        Logger::GetLogger()->ToNtTable("DrivePath" + m_pathname, "Done", "True");
        return true;
    }
    if (isDone)
    {   //debugging
        Logger::GetLogger()->ToNtTable("DrivePath" + m_pathname, "Done", "True");
        Logger::GetLogger()->ToNtTable("DrivePath" + m_pathname, "WhyDone", whyDone);
        Logger::GetLogger()->LogError(Logger::LOGGER_LEVEL::PRINT, "DrivePath" + m_pathname, "Is done because: " + whyDone);
    }
    return isDone;
    
}

bool DrivePath::IsSamePose(frc::Pose2d lCurPos, frc::Pose2d lPrevPos, double tolerance) //position checking functions
{
    // Detect if the two poses are the same within a tolerance
    double dCurPosX = lCurPos.X().to<double>() * 1000; //cm
    double dCurPosY = lCurPos.Y().to<double>() * 1000;
    double dPrevPosX = lPrevPos.X().to<double>() * 1000;
    double dPrevPosY = lPrevPos.Y().to<double>() * 1000;

    double dDeltaX = abs(dPrevPosX - dCurPosX);
    double dDeltaY = abs(dPrevPosY - dCurPosY);

    Logger::GetLogger()->ToNtTable("Deltas", "iDeltaX", to_string(dDeltaX));
    Logger::GetLogger()->ToNtTable("Deltas", "iDeltaY", to_string(dDeltaY));

    //  If Position of X or Y has moved since last scan..  Using Delta X/Y
    return (dDeltaX <= tolerance && dDeltaY <= tolerance);
}

void DrivePath::GetTrajectory //Parses pathweaver json to create a series of points that we can drive the robot to
(
    string  path
)
{
    if (!path.empty()) // only go if path name found
    {
        Logger::GetLogger()->LogError(string("DrivePath" + m_pathname), string("Finding Deploy Directory"));

        // Read path into trajectory for deploy directory.  JSON File ex. Bounce1.wpilid.json
        //wpi::SmallString<64> deployDir;  //creates a string variable
        //frc::filesystem::GetDeployDirectory(deployDir);  //grabs the deploy directory: "/lvuser/deploy" on roborio
        //wpi::sys::path::append(deployDir, "paths");  //goes into "/lvuser/deploy/paths" on roborio
        //wpi::sys::path::append(deployDir, path); // load path from deploy directory
    	auto deployDir = frc::filesystem::GetDeployDirectory();
        deployDir += "/paths/" + m_pathname;

        Logger::GetLogger()->LogError(string("Deploy path is "), deployDir.c_str()); //Debugging
        
        m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDir);  //Creates a trajectory or path that can be used in the code, parsed from pathweaver json
        m_trajectoryStates = m_trajectory.States();  //Creates a vector of all the states or "waypoints" the robot needs to get to
        
        Logger::GetLogger()->LogError(string("DrivePath - Loaded = "), path);
        Logger::GetLogger()->ToNtTable("DrivePathValues", "TrajectoryTotalTime", m_trajectory.TotalTime().to<double>());
    }
}

void DrivePath::CalcCurrentAndDesiredStates()
{
    m_currentChassisPosition = m_chassis.get()->GetPose(); //Grabs current pose / position
    auto sampleTime = units::time::second_t(m_timer.get()->Get()); //+ 0.02  //Grabs the time that we should sample a state from

    m_desiredState = m_trajectory.Sample(sampleTime); //Gets the target state based on the current time

    // May need to do our own sampling based on position and time     

    Logger::GetLogger()->ToNtTable("DrivePathValues", "DesiredPoseX", m_desiredState.pose.X().to<double>());
    Logger::GetLogger()->ToNtTable("DrivePathValues", "DesiredPoseY", m_desiredState.pose.Y().to<double>());
    Logger::GetLogger()->ToNtTable("DrivePathValues", "DesiredPoseOmega", m_desiredState.pose.Rotation().Degrees().to<double>());
    Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentPosX", m_currentChassisPosition.X().to<double>());
    Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentPosY", m_currentChassisPosition.Y().to<double>());
    Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentPosOmega", m_desiredState.pose.Rotation().Degrees().to<double>());
    Logger::GetLogger()->ToNtTable("DeltaValues", "DeltaX", m_desiredState.pose.X().to<double>() - m_currentChassisPosition.X().to<double>());
    Logger::GetLogger()->ToNtTable("DeltaValues", "DeltaY", m_desiredState.pose.Y().to<double>() - m_currentChassisPosition.Y().to<double>());

    Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentTime", m_timer.get()->Get().to<double>());
}