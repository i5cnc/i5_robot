/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, i5cnc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the i5cnc nor the names
 *    of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * Author: Guo Wei, Liu Fang - i5cnc
 */

#include <simple_message/messages/joint_message.h>
#include <simple_message/messages/joint_feedback_message.h>
#include <industrial_robot_client/robot_state_interface.h>
#include <industrial_utils/param_utils.h>
#include <stdexcept>

#include "i5_driver/joint_feedback_handler.h"

using industrial_robot_client::robot_state_interface::RobotStateInterface;
using industrial_robot_client::joint_feedback_handler::JointFeedbackHandler;
using industrial_utils::param::getJointNames;

using namespace industrial::message_handler;
using namespace industrial::simple_message;
using namespace industrial::joint_message;
using namespace industrial::joint_feedback_message;

int main(int argc, char** argv)
{
    // initialize node
    ros::init(argc, argv, "state_interface");

    // launch the default Robot State Interface connection/handlers
    RobotStateInterface rsi;
    rsi.init();

    JointFeedbackHandler jointFeedbackHandler;
    std::vector < std::string > joint_names = rsi.get_joint_names();
    jointFeedbackHandler.init(rsi.get_connection(), joint_names);
    rsi.add_handler(&jointFeedbackHandler);


//    JointRelayHandler jointHandler;
//    jointHandler.init(rsi.get_connection(), joint_names);
//    rsi.add_handler(&jointHandler);

    // run the node
    rsi.run();

    return 0;
}

