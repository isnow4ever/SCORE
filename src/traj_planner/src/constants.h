/*
* License:The MIT License (MIT)
*
* Copyright (c) 2013,2014 Yanyu Su
* State Key Laboratory of Robotics and System, Harbin Institute of Technology
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/
#pragma once

// important topic names
#define COORDINATOR_STATES            "/coordinator_states"
#define COORDINATOR_COMMAND           "/coordinator_command"
#define COORDINATOR_USER_FRAME        "/coordinator_user_frame"

#define INTERFACE_JOINT_PATH_COMMAND  "/joint_path_command"
#define INTERFACE_FEEDBACK_STATES     "/feedback_states"
#define INTERFACE_JOINT_STATES        "/joint_states"
#define INTERFACE_COMMAND             "/interface_command"
#define INTERFACE_STATES              "/interface_states"

#define INTERFACE_CMD_DIRECT   "DIRECT"
#define INTERFACE_CMD_SPLINE   "SPLINE"
#define INTERFACE_CMD_CLEAR    "CLEAR"
#define INTERFACE_CMD_STOP     "STOP"

#define INTERFACE_STATE_SPARATER "\t"
#define INTERFACE_STATE_DIRECT   "DIRECT"
#define INTERFACE_STATE_SPLINE   "SPLINE"
#define INTERFACE_STATE_MOVING   "MOVING"
#define INTERFACE_STATE_STABLE   "STABLE"

// control loop durations in seconds
#define PMAC_LOOP         0.050
#define COORDINATOR_LOOP  0.025
#define STATE_LOOP        0.0375
#define INTERFACE_LOOP    0.0125

// coordinator defines in regex expression
#define REGEX_WORD     "(\\w+)"
#define REGEX_NUMBER   "([+-]?\\d+(?:\\.\\d+)?)"
#define REGEX_SPACE    "\\s+"
#define REGEX_ID       "<\\d+>"
#define REGEX_CMD      REGEX_WORD
#define REGEX_TIME     "\\[" REGEX_NUMBER "\\]"
#define REGEX_TARGET   "\\{" REGEX_WORD   "\\}"
#define REGEX_FRAME    "\\{" REGEX_WORD   "\\}"
#define REGEX_UNIT     "\\(" "mm" "," "deg" "," "s" "\\)"
#define REGEX_ANY     ".*"

/*
#define COORDINATOR_V6_CMD (REGEX_ID     REGEX_SPACE \
                        REGEX_CMD    REGEX_SPACE \
                        REGEX_TARGET REGEX_SPACE \
                        "(?:" REGEX_NUMBER REGEX_SPACE "){6}" \
                        REGEX_UNIT   REGEX_SPACE \
                        REGEX_FRAME)
*/

#define COORDINATOR_CMD_NUMBER(CMD,NUM)    (REGEX_ID     REGEX_SPACE \
                                            #CMD    REGEX_SPACE \
                                            REGEX_TARGET REGEX_SPACE \
                                            "(?:" REGEX_NUMBER REGEX_SPACE "){" #NUM "}" \
                                            REGEX_UNIT   REGEX_SPACE \
                                            REGEX_FRAME)

#define COORDINATOR_CMD_NUMBER_TIME(CMD,NUM)    (REGEX_ID     REGEX_SPACE \
                                                 #CMD         REGEX_SPACE \
                                                 REGEX_TARGET REGEX_SPACE \
                                                 "(?:" REGEX_NUMBER REGEX_SPACE "){" #NUM "}" \
                                                 REGEX_TIME   REGEX_SPACE \
                                                 REGEX_UNIT   REGEX_SPACE \
                                                 REGEX_FRAME)

#define COORDINATOR_CMD_MULTI_NUMBER_TIME(CMD)   (REGEX_ID           REGEX_SPACE \
                                             #CMD               REGEX_SPACE \
                                             REGEX_TARGET       REGEX_SPACE \
                                             "(?:" REGEX_NUMBER REGEX_SPACE ")+" \
                                             REGEX_TIME         REGEX_SPACE \
                                             REGEX_UNIT         REGEX_SPACE \
                                             REGEX_FRAME)

#define COORDINATOR_CMD_DIRACTE(CMD)        (REGEX_ID     REGEX_SPACE \
                                             #CMD)

#define COORDINATOR_CMD                     (REGEX_ID     REGEX_SPACE \
                                             REGEX_ANY )
#define COORDINATOR_FRAME                   (REGEX_WORD REGEX_SPACE REGEX_WORD \
                                            REGEX_SPACE REGEX_NUMBER \
                                            REGEX_SPACE REGEX_NUMBER \
                                            REGEX_SPACE REGEX_NUMBER \
                                            REGEX_SPACE REGEX_NUMBER \
                                            REGEX_SPACE REGEX_NUMBER \
                                            REGEX_SPACE REGEX_NUMBER)

#define COORDINATOR_CMD_IO(CMD)        (REGEX_ID     REGEX_SPACE \
                                             #CMD REGEX_SPACE REGEX_NUMBER)

#define COORDINATOR_CMD_WAIT(CMD)        (REGEX_ID     REGEX_SPACE \
                                             #CMD REGEX_SPACE REGEX_NUMBER \
                                             REGEX_SPACE REGEX_NUMBER)

// default values
#define JOINT1_ROS_AXIS    "joint_1"
#define JOINT2_ROS_AXIS    "joint_2"
#define JOINT3_ROS_AXIS    "joint_3"
#define JOINT4_ROS_AXIS    "joint_4"
#define JOINT5_ROS_AXIS    "joint_5"
#define JOINT6_ROS_AXIS    "joint_6"

#define JOINT1_PMAC_AXIS    "X"
#define JOINT2_PMAC_AXIS    "Y"
#define JOINT3_PMAC_AXIS    "Z"
#define JOINT4_PMAC_AXIS    "A"
#define JOINT5_PMAC_AXIS    "B"
#define JOINT6_PMAC_AXIS    "C"

#define JOINT1_PMAC_POS_FEEDBACK    "P1"
#define JOINT2_PMAC_POS_FEEDBACK    "P2"
#define JOINT3_PMAC_POS_FEEDBACK    "P3"
#define JOINT4_PMAC_POS_FEEDBACK    "P4"
#define JOINT5_PMAC_POS_FEEDBACK    "P5"
#define JOINT6_PMAC_POS_FEEDBACK    "P6"
