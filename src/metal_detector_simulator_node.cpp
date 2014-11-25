/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Gonçalo Cabrita on 21/11/2014
*********************************************************************/

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <metal_detector_msgs/Coil.h>
#include <metal_detector_msgs/SetCoilsZero.h>

#include "MetalDetectorSimulator.h"

class Coil
{
public:
    Coil(){}

    std::vector<int> channel;
    std::vector<int> zero;
};

std::map<std::string, Coil> coils;

bool setCoilsZero(metal_detector_msgs::SetCoilsZero::Request &req, metal_detector_msgs::SetCoilsZero::Response &res)
{
    if(req.coil.size() == 0)
    {
        std::map<std::string, Coil>::iterator coil;
        for(coil=coils.begin() ; coil!=coils.end() ; ++coil)
        {
            for(int j=0 ; j<coil->second.channel.size() ; j++)
            {
                coil->second.zero[j] = coil->second.channel[j];
            }
        }

        res.status_message = "Set coils zero was successful using current coil values.";
        return true;
    }
    else if(req.coil.size() == coils.size())
    {
        int i;
        std::map<std::string, Coil>::iterator coil;
        for(i=0, coil=coils.begin() ; coil!=coils.end() ; ++coil, i++)
        {
            if(req.coil[i].zero.size() != coil->second.channel.size())
            {
                res.status_message = "Set coils zero failed, number of coil channels is invalid.";
                return false;
            }
            for(int j=0 ; j<coil->second.channel.size() ; j++)
            {
                coil->second.zero[j] = req.coil[i].zero[j];
            }
        }

        res.status_message = "Set coils zero was successful using the provided values.";
        return true;
    }
    else
    {
        res.status_message = "Set coils zero failed, number of coils is invalid.";
        return false;
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "metal_detector_simulator_node");

    // ROS stuff...
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    ROS_INFO("Metal Detector Simulator for ROS V0.1");

    tf::TransformListener listener;

    ros::Publisher coil_pub = n.advertise<metal_detector_msgs::Coil>("coils", 50);
    ros::ServiceServer service = n.advertiseService("set_coils_zero", setCoilsZero);

    std::string global_frame_id;
    pn.param<std::string>("global_frame_id", global_frame_id, "map");

    std::vector<std::string> coils_frame_ids;
    pn.getParam("coils_frame_ids", coils_frame_ids);

    // The simulator
    MetalDetectorSimulator simulator;

    // Lets start by loading the SVM models into the simulator
    std::vector<std::string> types_of_samples;
    pn.getParam("types_of_samples", types_of_samples);

    std::map< std::string, std::vector<std::string> > paths;
    for(int i=0 ; i<types_of_samples.size() ; i++)
    {
        std::vector<std::string> model_files;
        pn.getParam(types_of_samples[i].c_str(), model_files);
        paths.insert( std::pair< std::string, std::vector<std::string> >(types_of_samples[i], model_files) );
    }
    try{ simulator.loadModels(&paths); }
    catch(Exception& e)
    {
        ROS_FATAL("Metal Detector Simulator -- %s", e.what());
        ROS_BREAK();
    }

    // Now lets load the metal samples
    std::string samples_file;
    pn.param<std::string>("samples_file", samples_file, "samples.csv");

    int loaded;
    try{ loaded = simulator.loadMetalSamples(&samples_file); }
    catch(Exception& e)
    {
        ROS_FATAL("Metal Detector Simulator -- %s", e.what());
        ROS_BREAK();
    }
    ROS_INFO("Metal Detector Simulator -- Loaded %d samples sucessfuly.", loaded);

    // Populate the coils map
    for(int i=0 ; i<coils_frame_ids.size() ; i++)
    {
        Coil coil;
        coil.channel.resize(simulator.getNumberOfChannels());
        coil.zero.resize(simulator.getNumberOfChannels());
        coils.insert(std::pair<std::string, Coil>(coils_frame_ids[i], coil));
    }

    // Main loop...
    double rate;
    pn.param("rate", rate, 10.0);

    ros::Rate r(rate);
    while(ros::ok())
    {
        // For each coil
        for(int i=0 ; i<coils_frame_ids.size() ; i++)
        {
            // Lets check where the coil is on our global frame
            tf::StampedTransform transform;
            try
            {
              listener.lookupTransform(global_frame_id, coils_frame_ids[i], ros::Time(0), transform);
            }
            catch(tf::TransformException ex)
            {
                ROS_ERROR("Metal Detector Simulator -- %s", ex.what());
            }

            metal_detector_msgs::Coil coil_msg;

            coil_msg.header.stamp = ros::Time::now();
            coil_msg.header.frame_id = coils_frame_ids[i];

            // And ask the simulator what the channel readings are
            simulator.getChannelReadings(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z(), &coil_msg.channel);

            std::map<std::string, Coil>::iterator coil = coils.find(coils_frame_ids[i]);
            coil_msg.zero = coil->second.zero;

            coil_pub.publish(coil_msg);

            // We might need this later to set the zero
            coil->second.channel = coil_msg.channel;
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

// EOF
