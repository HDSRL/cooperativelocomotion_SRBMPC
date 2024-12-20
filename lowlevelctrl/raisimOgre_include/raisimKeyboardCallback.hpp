//
// Created by Jemin Hwangbo on 4/11/19.
// MIT License
//
// Copyright (c) 2019-2019 Robotic Systems Lab, ETH Zurich
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef RAISIMOGREVISUALIZER_RAISIMKEYBOARDCALLBACK_HPP
#define RAISIMOGREVISUALIZER_RAISIMKEYBOARDCALLBACK_HPP

#include "raisim/OgreVis.hpp"
#include "raisimKeyboardCallback.hpp"
#include "guiState.hpp"
#include "stdio.h"

struct RoboCmd{
  float vel[3] = {0};
  float pose[6] = {0};
  bool change_gait = 0;
};

RoboCmd cmd;

bool raisimKeyboardCallback(const OgreBites::KeyboardEvent &evt) {
  auto &key = evt.keysym.sym;
  // termination gets the highest priority
  switch (key) {
    case OgreBites::SDLK_F1:
      raisim::gui::showBodies = !raisim::gui::showBodies;
      break;
    case OgreBites::SDLK_F2:
      raisim::gui::showCollision = !raisim::gui::showCollision;
      break;
    case OgreBites::SDLK_F3:
      raisim::gui::showContacts = !raisim::gui::showContacts;
      break;
    case OgreBites::SDLK_F4:
      raisim::gui::showForces = !raisim::gui::showForces;
      break;
    case OgreBites::SDLK_UP:
      cmd.vel[0] += 0.1;
      break;
    case OgreBites::SDLK_DOWN:
      cmd.vel[0] -= 0.1;
      break;
    case OgreBites::SDLK_LEFT:
      cmd.vel[2] += 0.02;
      break;
    case OgreBites::SDLK_RIGHT:
      cmd.vel[2] -= 0.02;
      break;
    case OgreBites::SDLK_PAGEUP:
      cmd.pose[4] += 0.01;
      break;
    case OgreBites::SDLK_PAGEDOWN:
      cmd.pose[4] -= 0.01;
      break;
    case OgreBites::SDLK_RETURN:
      cmd.change_gait = 1;
      break;
    default:
      break;
  }
  // printf("Fwd Vel: %0.3f || Lat Vel: %0.3f || Yaw Vel: %0.3f || Pitch: %0.3f\n",cmd.vel[0],cmd.vel[1],cmd.vel[2],cmd.pose[1]);
  return false;
}

#endif //RAISIMOGREVISUALIZER_RAISIMKEYBOARDCALLBACK_HPP
