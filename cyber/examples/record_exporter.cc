/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <assert.h>
#include <fstream>
#include <iostream>
#include <string>

#include "cyber/cyber.h"
#include "cyber/common/file.h"
#include "cyber/message/raw_message.h"
#include "cyber/proto/record.pb.h"
#include "cyber/record/record_message.h"
#include "cyber/record/record_reader.h"
#include "cyber/record/record_writer.h"

#include "modules/control/proto/control_cmd.pb.h"
#include "modules/localization/proto/localization.pb.h"

using ::apollo::cyber::record::RecordReader;
using ::apollo::cyber::record::RecordMessage;
// using apollo::cyber::message::RawMessage;

using apollo::cyber::common::GetFileName;

void DisplayUsage(const std::string& binary) {
  std::cerr << "usage: " << binary << " <record_fiename>\n"
            << std::endl;
}

void DumpTextMessage(const google::protobuf::Message& msg) {
  std::string msg_text;

  google::protobuf::TextFormat::PrintToString(msg, &msg_text);

  std::cerr << "[" << msg.GetTypeName() << "]"
            << std::endl << msg_text << std::endl;
}

void ExportRecords(const std::string &readfile) {
  RecordReader reader(readfile);
  RecordMessage message;
  
  auto pose_filename = "out/" + GetFileName(readfile) + "-pose.csv";
  
  std::ofstream pose_file(pose_filename, std::ios::out);
  
  if (!pose_file.is_open()) { 
    AERROR << "Can't open output file '" << pose_filename << "'";
    AERROR << "Make sure target folder exists and is writable.";
    return;
  }
  
  pose_file << "timestamp_sec;"
    << "position_x;" << "position_y;" << "position_z;"
    << "heading;"
    << "qx;" << "qy;" << "qz;" << "qw;"
    << "euler_x;" << "euler_y;" << "euler_z;"
    << std::endl;
    
  std::cerr << "Write GNSS Pos to " << pose_filename << std::endl;

  // read all messages
  uint64_t i = 0;

  for (i = 0; reader.ReadMessage(&message); ++i) {
    /* 
    std::cerr << "msg[" << i << "]-> "
        << "channel name: " << message.channel_name
        << "; content len: " << message.content.size()
        << "; msg time: " << message.time
        << std::endl;
        */
    
    /*
    // Dump Control Commands as text protobuf messages
    if (!message.channel_name.compare("/apollo/control")) {
        auto msg = apollo::control::ControlCommand();

        if (msg.ParseFromString(message.content)) {

            std::string msg_text;

            google::protobuf::TextFormat::PrintToString(msg, &msg_text);

            std::cerr << "[apollo/control]" << msg_text << std::endl;
        } else {
            std::cerr << "Failed to parse protobuf" << std::endl;
        }
    }
    */
    
    if (message.channel_name.compare("/apollo/localization/pose") == 0) {
      // Create message object of appropriate type.
      // Check the output of 'cyber_recorder info' for the exact message types.
      
      auto msg = apollo::localization::LocalizationEstimate();
      
      // Parse message content
      if (msg.ParseFromString(message.content)) {
        // dump message to check fields
        // DumpTextMessage(msg);
        
        // process message data: save some fields to CSV
        const auto& pose = msg.pose();
        const auto& position = pose.position();
        const auto& orientation = pose.orientation();
        const auto& euler_angles = pose.euler_angles();
        
        // Make sure fields set and order match to the header written above
        
        pose_file << std::setprecision(6) << std::fixed << msg.header().timestamp_sec() << ";"
          << position.x() << ";" << position.y() << ";" << position.z() << ";"
          << pose.heading() << ";"
          << orientation.qx() << ";" << orientation.qy() << ";"<< orientation.qz() << ";"<< orientation.qw() << ";"
          << euler_angles.x() << ";" << euler_angles.y() << ";"<< euler_angles.z() << ";"
          << std::endl;

      } else {
        AERROR << "Failed to parse protobuf" << std::endl;
      }
    }
  }
  
  std::cerr << "MSG totalcount: " << i << std::endl;
}

int main(int argc, char *argv[]) {
  std::string binary = GetFileName(std::string(argv[0]));
  
  if (argc < 2) {
    DisplayUsage(binary);
    return -1;
  }
  
  apollo::cyber::Init(argv[0]);
  
  sleep(1);
  std::string file_name(argv[1]);
  
  // TODO setup logger to use ADEBUG/AINFO instead of std::cerr

  std::cerr << "Reading " << file_name << std::endl;
  ExportRecords(file_name);
  return 0;
}
