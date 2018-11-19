//=========================================================================
// Bachelor - Autonomous Intelligent Driving GmbH competition solution
//
// Copyright 2018 Pierre Guilbert
// Author: Pierre Guilbert (spguilbert@gmail.com)
// Data: 14-11-2018
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//=========================================================================

#ifndef ELEVATION_MAP_H
#define ELEVATION_MAP_H

// LOCAL
#include "GeometricTools.h"
// STD
#include <iostream>
#include <vector>

/**
* \class ElevationMap
* \brief provides an easy API to manipulate the
*        elevation maps
*
* \author $Author: Pierre Guilbert $
* \version $Revision: 1.0 $
* \date $Date: 02-11-2018 $
* Contact: spguilbert@gmail.com
*/
class ElevationMap
{
public:
    /// constructor
    ElevationMap();

    /// constructor
    ElevationMap(const std::vector<uint8_t>& argElevation, const std::vector<uint8_t> argOverrides,
                 int argH, int argW);

    /// Get the elevation for the intput point
    double Z(int x, int y);

    /// Get the elevation differentiation for the input point
    Vector<double, 2> dZ(int x, int y);

    /// Get / Set if a position is available
    uint8_t GetAvailable(int x, int y);
    void SetAvailable(int x, int y, uint8_t argAvailable);

    /// indicate if the ground is water at the input point
    bool IsWater(int x, int y);

private:
    /// dimension of the elevation map support
    /// in number of pixels
    int H;
    int W;

    /// data of the elevation map and
    /// about the nature of the ground
    /// water, forest, ...
    std::vector<uint8_t> elevation;
    std::vector<uint8_t> overrides;
    std::vector<uint8_t> available;

    /// check if the coordinates are correct
    bool CheckCoord(int x, int y);
};

#endif // ELEVATION_MAP_H