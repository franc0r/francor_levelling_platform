/*
 * LevellingPlatformNode.h
 *
 *  Created on: 17.04.2018
 *      Author: feesmrt
 *
 * BSD 3-Clause License
 * Copyright (c) 2018, FRANC0R - Franconian Open Robotics
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <francor_levelling_platform/LevellingPlatformNode.h>

namespace francor {

LevellingPlatformNode::LevellingPlatformNode(const std::string& ns, const std::string& serialPortPath) :
_nh(),
_serialPortPath(serialPortPath),
_serialInterface()
{

}

LevellingPlatformNode::~LevellingPlatformNode(void) 
{

}

void LevellingPlatformNode::run(void) 
{
  ROS_INFO("Starting up mode...");

  if(!_serialInterface.connect(_serialPortPath, 256000)) {
    ROS_ERROR("Failed to connect to virtual com port %s", _serialPortPath.c_str());
  }

  sleep(1);

  const unsigned char flushData[3] = {0x00, '\r', '\n'};
  _serialInterface.transmitBuffer(flushData, 3);




}

}

int main(int argc, char** argv) 
{
  std::string serialPortPath;

  ros::init(argc, argv, "francor_levelling_platform");
  ros::NodeHandle nh("~");

  nh.param<std::string>("serialPortPath", serialPortPath, "/dev/ttyACM0");

  francor::LevellingPlatformNode node(nh.getNamespace(), serialPortPath);

  node.run();

  return 0;

}
