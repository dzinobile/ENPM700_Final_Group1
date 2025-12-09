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
 * @file DoneState.hpp
 * @brief Dummy State node for once all goals have been achieved
 * @author Daniel Zinobile
 * @date 08-Dec-2025
 */
#pragma once

#include "States.hpp"

// Forward declaration to avoid circular references
class SheepdogNode;

/**
 * @class DoneState
 * @brief Dummy state for when all other states have achieved their goals
 */
class DoneState : public States {
public:
  /**
   * @brief Constructor for Done state
   */
  DoneState();

  /**
   * @brief Inherited destructor for Done state
   */
  ~DoneState() override;

  /**
   * @brief Inherited update function to enact behavior per state
   * @param context Reference to SheepdogNode
   */
  void update(SheepdogNode &context) override;

  /**
   * @brief Inherited transition function to change the state
   * @param context Reference to SheepdogNode
   * @return Reference to state
   */
  States *transition(SheepdogNode &context) override;
};