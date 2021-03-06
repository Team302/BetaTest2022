
#pragma once

#include <map>
#include <memory>

#include <hw/interfaces/IDragonMotorController.h>
#include <hw/usages/MotorControllerUsage.h>

typedef std::map <MotorControllerUsage::MOTOR_CONTROLLER_USAGE, 
                  std::shared_ptr<IDragonMotorController>> IDragonMotorControllerMap;
