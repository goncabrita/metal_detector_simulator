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

#ifndef METALDETECTORSIMULATOR_H
#define METALDETECTORSIMULATOR_H

#include <stdexcept>
#include <termios.h>
#include <string>
#include <vector>
#include <map>
#include <stdint.h>
#include <math.h>

#include "libsvm/svm.h"

#define DEF_EXCEPTION(name, parent) \
class name : public parent \
{ \
    public: \
    name(const char* msg) : parent(msg) {} \
}

DEF_EXCEPTION(Exception, std::runtime_error);

#undef DEF_EXCEPTION

class MetalSample
{
public:
    MetalSample();
    MetalSample(double new_x, double new_y, double new_depth, char * new_type)
    {
        x = new_x;
        y = new_y;
        depth = new_depth;
        type = new_type;
    }

    double x;
    double y;
    double depth;
    std::string type;

    double distance(double pos_x, double pos_y)
    {
        return sqrt(pow((pos_x-x), 2) + pow((pos_y-y), 2));
    }
};

class MetalDetectorSimulator
{
public:
    MetalDetectorSimulator();

    void loadModels(std::map< std::string, std::vector<std::string> > * paths);
    int loadMetalSamples(std::string * path);

    void getChannelReadings(double coil_x, double coil_y, double coil_height, std::vector<int> * channel_readings);

    unsigned int getNumberOfChannels()
    {
        return number_of_channels_;
    }

private:
    unsigned int number_of_channels_;
    std::map< std::string, std::vector<struct svm_model *> > models_;
    std::vector<MetalSample> samples_;
};

#endif

// EOF

