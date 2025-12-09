// Copyright 2025
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/**
 * @file States.hpp
 * @brief Header file for State Interface class
 * @author Daniel Zinobile
 * @date 08-Dec-2025
 */
#pragma once

#include <memory>

// Forward declaration to avoid circular references
class SheepdogNode;

/**
 * @class States
 * @brief Abstract state interface class for robot states
 * Defines common interface used by all states in the state machine:
 *  - update() issues commands based on current state
 *  - transition() determines the next state based on if goals have been reached
 */
class States {
public:
  /**
   * @brief Constructor for States class
   */
  States();
  /**
   * @brief Virtual destructor for States class
   */
  virtual ~States();
  /**
   * @brief Virtual method to execute behavior corresponding to a current state
   * @param context Reference to the SheepdogNode
   * @return Pointer to the next state
   */
  virtual void update(SheepdogNode &context) = 0;

  /**
   * @brief Change or keep state based on current conditions
   * @param context Reference to the SheepdogNode
   * @return Pointer to the next state
   */
  virtual States *transition(SheepdogNode &context) = 0;
};
