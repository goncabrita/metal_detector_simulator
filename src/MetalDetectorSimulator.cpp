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

#include "MetalDetectorSimulator.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <fcntl.h>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <cstdlib>

#define Malloc(type,n) (type *)malloc((n)*sizeof(type))

#define METALDETECTOR_EXCEPT(except, msg, ...) \
{ \
    char buf[1000]; \
    snprintf(buf, 1000, msg " (in MetalDetectorSimulator::%s)" , ##__VA_ARGS__, __FUNCTION__); \
    throw except(buf); \
}

MetalDetectorSimulator::MetalDetectorSimulator()
{

}

void MetalDetectorSimulator::loadModels(std::map< std::string, std::vector<std::string> > * paths)
{
    number_of_channels_ = paths->begin()->second.size();

    std::map< std::string, std::vector<std::string> >::iterator type;
    for(type=paths->begin() ; type!=paths->end() ; ++type)
    {
        if(type->second.size() != number_of_channels_)
            METALDETECTOR_EXCEPT(Exception, "All samples must have the same number of channel models.");

        std::vector<struct svm_model *> channel_models;

        std::vector<std::string>::iterator model;
        for(model=type->second.begin() ; model!=type->second.end() ; ++type)
        {
            struct svm_model * new_model = svm_load_model(model->c_str());
            channel_models.push_back(new_model);
        }

        models_.insert(std::pair< std::string, std::vector<struct svm_model *> >(type->first, channel_models));
    }
}

int MetalDetectorSimulator::loadMetalSamples(std::string * path)
{
    if(models_.size() == 0)
        METALDETECTOR_EXCEPT(Exception, "Models must be loaded prior to samples.");

    FILE * samples_file;
    samples_file = fopen (path->c_str(), "r");
    if(samples_file == NULL)
        METALDETECTOR_EXCEPT(Exception, "Error opening file %s", path->c_str());

    double x;
    double y;
    double depth;
    char type[128];

    while(!feof(samples_file))
    {
        int ret = fscanf(samples_file, "%lf,%lf,%lf,%s\n", &x, &y, &depth, type);
        if(ret != 4)
        {
            fclose(samples_file);
            METALDETECTOR_EXCEPT(Exception, "Error reading samples file.");
        }
        samples_.push_back(MetalSample(x, y, depth, type));
    }
    fclose(samples_file);

    return samples_.size();
}

void MetalDetectorSimulator::getChannelReadings(double coil_x, double coil_y, double coil_height, std::vector<int> * channel_readings)
{
    // Step 1. Find the closest sample
    std::vector<MetalSample>::iterator it = samples_.begin();
    double shortest_distance = it->distance(coil_x, coil_y);
    std::vector<MetalSample>::iterator closest_sample = it;
    for(it=samples_.begin()+1 ; it!=samples_.end() ; ++it)
    {
        double d = it->distance(coil_x, coil_y);
        if(d < shortest_distance)
        {
            shortest_distance = d;
            closest_sample = it;
        }
    }

    // Step 2. Estimate the reading based on the model of each channel
    struct svm_node * x_space;
    x_space = Malloc(struct svm_node, 3);
    x_space[0].index = 0;
    x_space[0].value = shortest_distance;
    x_space[1].index = 1;
    x_space[1].value = coil_height;
    x_space[2].index = 2;
    x_space[2].value = closest_sample->depth;

    std::map< std::string, std::vector<struct svm_model *> >::iterator model = models_.find(closest_sample->type);
    for(int i=0 ; i<number_of_channels_ ; i++)
    {
        double reading = svm_predict(model->second[i], x_space);

        // Todo: Add noise
        //reading += noise;

        channel_readings->push_back(reading);
    }

    free(x_space);
}
