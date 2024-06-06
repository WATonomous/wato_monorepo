// Licensed to the Apache Software Foundation (ASF) under one
// or more contributor license agreements.  See the NOTICE file
// distributed with this work for additional information
// regarding copyright ownership.  The ASF licenses this file
// to you under the Apache License, Version 2.0 (the
// "License"); you may not use this file except in compliance
// with the License.  You may obtain a copy of the License at
// 
//   http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing,
// software distributed under the License is distributed on an
// "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied.  See the License for the
// specific language governing permissions and limitations
// under the License.

/** @file ros2socketcan.h
 *
 *  @ingroup ROS2CAN_bridge
 *  @author Philipp Wuestenberg and changed by Daniel Peter (peter@fh-aachen.de)
 *  @brief  bidirectional ROS2 to CAN interface with topics and service. (change: created a lib from the ROS2 node -> reduced overhead)
 */

#ifndef __ros2_socketcan_H__
#define __ros2_socketcan_H__
#include <linux/can/raw.h>

#include <boost/asio.hpp>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include <sstream>
#include <fstream>

class ros2socketcan
{

public:

    ros2socketcan() : stream(ios)
    {

    }

    ~ros2socketcan(){
        printf("\nEnd of Publisher Thread. \n");
    }

    void Init(const char* can_socket, std::function<void(const can_msgs::msg::Frame msg)> callback, std::string whitelist_path = "")
    {
        signals = std::make_shared<boost::asio::signal_set>(ios, SIGINT, SIGTERM);
        this->canCallback = callback;
	    std::cout << "Using can socket " << can_socket << std::endl;

        //generate can-whitelist from config_file:
        parseWhiteListFile(whitelist_path);

        const char* canname = can_socket;

        topicname_receive 	<< "CAN/" << canname << "/" << "receive";
        topicname_transmit 	<< "CAN/" << canname << "/" << "transmit";

        strcpy(ifr.ifr_name, can_socket);
        ioctl(natsock, SIOCGIFINDEX, &ifr);

        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if(bind(natsock,(struct sockaddr *)&addr,sizeof(addr))<0)
        {
            perror("Error in socket bind");
        }

        stream.assign(natsock);


        stream.async_read_some(boost::asio::buffer(&rec_frame, sizeof(rec_frame)),std::bind(&ros2socketcan::CanListener, this,std::ref(rec_frame),std::ref(stream)));

        signals->async_wait(std::bind(&ros2socketcan::stop, this));

        boost::system::error_code ec;

        std::size_t (boost::asio::io_service::*run)() = &boost::asio::io_service::run;
        std::thread bt(std::bind(run, &ios));
        bt.detach();
    }

    void CanSend(const can_msgs::msg::Frame msg)
    {
        
        if((use_tx_whitelist && std::find(can_tx_whitelist.begin(), can_tx_whitelist.end(), (unsigned int) msg.id) != can_tx_whitelist.end()) || !use_tx_whitelist) {

            struct can_frame frame1;

            frame1.can_id = msg.id;

            //if (msg.is_extended == 1) {
             if (msg.eff == 1) {
                frame1.can_id = frame1.can_id + CAN_EFF_FLAG;
            }

            //if (msg.is_error == 1) {
             if (msg.err == 1) {
                frame1.can_id = frame1.can_id + CAN_ERR_FLAG;
            }

            //if (msg.is_rtr == 1) {
             if (msg.rtr == 1) {
                frame1.can_id = frame1.can_id + CAN_RTR_FLAG;
            }

            frame1.can_dlc = msg.dlc;

            for (int i = 0; i < (int) frame1.can_dlc; i++) {
                frame1.data[i] = msg.data[i];
            }

            stream.async_write_some(boost::asio::buffer(&frame1, sizeof(frame1)), std::bind(&ros2socketcan::CanSendConfirm, this));
        }

    }

private:

    std::function<void(const can_msgs::msg::Frame msg)> canCallback;
    can_msgs::msg::Frame current_frame;
    boost::asio::io_service ios;
    boost::asio::posix::basic_stream_descriptor<> stream;
    std::shared_ptr<boost::asio::signal_set> signals;

    void CanSendConfirm()
    {
        std::cout << "Message sent" << std::endl;
    }

    void CanListener(struct can_frame& rec_frame, boost::asio::posix::basic_stream_descriptor<>& stream)
    {
	    //std::cout << "rec_frame id: " << rec_frame.can_id << std::endl;
        if((use_rx_whitelist && std::find(can_rx_whitelist.begin(), can_rx_whitelist.end(), (unsigned int) rec_frame.can_id) != can_rx_whitelist.end()) || !use_rx_whitelist) {
            can_msgs::msg::Frame frame;

            std::stringstream s;

            frame.id = rec_frame.can_id;
	    //std::cout << "Frame id: " << frame.id << std::endl;
            frame.dlc = int(rec_frame.can_dlc);

            for (int i = 0; i < rec_frame.can_dlc; i++) {
                frame.data[i] = rec_frame.data[i];
                s << rec_frame.data[i];
            }
            current_frame = frame;
            this->canCallback(current_frame);
        } else {
	        std::cout << "Did not pass" << std::endl;
	    }
        stream.async_read_some(boost::asio::buffer(&rec_frame, sizeof(rec_frame)),std::bind(&ros2socketcan::CanListener,this, std::ref(rec_frame),std::ref(stream)));

    }

    void stop()
    {
        stream.cancel();
        ios.stop();
        signals->clear();
        printf("\nEnd of Listener Thread. Please press strg+c again to stop the whole program.\n");
    }

    void parseWhiteListFile(std::string path)
    {
        bool readingRx = true;
        bool firstRxOrTxRead = false;
        std::ifstream infile(path);
        std::string line;
        while (std::getline(infile, line)){
            if(line == "RX"){
                readingRx = true;
                use_rx_whitelist = true;
                firstRxOrTxRead = true;
                continue;
            }else if (line == "TX"){
                readingRx = false;
                use_tx_whitelist = true;
                firstRxOrTxRead = true;
                continue;
            }
            //it was not specified whether the read line is a rx or a tx. Skip until rx or tx is specified.
            if(!firstRxOrTxRead){
                continue;
            }
            std::istringstream iss(line);
            unsigned int id;
            if (!(iss >> id)) { break; }
            if(readingRx){
                can_rx_whitelist.push_back(id);
                std::cout << "Added RX-Message with id=" << id << " to whitelist" << std::endl;
            }else{
                can_tx_whitelist.push_back(id);
                std::cout << "Added TX-Message with id=" << id << " to whitelist" << std::endl;
            }
        }
    }

    struct sockaddr_can addr;
    struct can_frame frame;
    struct can_frame rec_frame;
    struct ifreq ifr;

    int natsock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    std::stringstream topicname_receive;
    std::stringstream topicname_transmit;
    std::stringstream servername;

    bool use_rx_whitelist = false;
    bool use_tx_whitelist = false;
    std::vector<unsigned int> can_rx_whitelist;
    std::vector<unsigned int> can_tx_whitelist;
};
#endif
