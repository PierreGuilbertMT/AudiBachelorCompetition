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

// LOCAL
#include "ElevationMap.h"

//-------------------------------------------------------------------------
ElevationMap::ElevationMap()
{
    this->H = 0;
    this->W = 0;
}

//-------------------------------------------------------------------------
ElevationMap::ElevationMap(const std::vector<uint8_t>& argElevation, const std::vector<uint8_t> argOverrides,
                           int argH, int argW)
{
    this->H = argH;
    this->W = argW;
    this->elevation = argElevation;
    this->overrides = argOverrides;
    this->available.resize(this->overrides.size(), 0);
}

//-------------------------------------------------------------------------
bool ElevationMap::CheckCoord(int x, int y)
{
    bool ok = true;
    ok &= (x >= 0) && (x < this->W);
    ok &= (y >= 0) && (y < this->H);
    return ok;
}

//-------------------------------------------------------------------------
double ElevationMap::Z(int x, int y)
{
    // default Z return
    double Z = 10000.0;

    // Check that the coordinates are in the boundaries
    if (!CheckCoord(x, y))
        return Z;

    /*// Check that the ground does not caontain water
    if ((this->overrides[y * this->W + x] == 16) ||
        (this->overrides[y * this->W + x] == 64))
        return Z;*/

    // everything is fine, return elevation
    return static_cast<double>(this->elevation[y * this->W + x]);
}

//-------------------------------------------------------------------------
bool ElevationMap::IsRiver(int x, int y)
{
    // Check that the coordinates are in the boundaries
    if (!CheckCoord(x, y))
        return false;

    bool isWater = (this->overrides[y * this->W + x] & (16));
    return isWater;
}

//-------------------------------------------------------------------------
bool ElevationMap::CanBeCrossed(int x, int y)
{
    // Check that the coordinates are in the boundaries
    if (!CheckCoord(x, y))
        return false;

    bool canBeCrossed = (this->overrides[y * this->W + x] & (64)) ||
                   this->elevation[y * this->W + x] == 0;

    return canBeCrossed;
}

//-------------------------------------------------------------------------
Vector<double, 2> ElevationMap::dZ(int x, int y)
{
    Vector<double, 2> jacobian;

    // use Newton quotient finite difference
    // as an approximation of the first order
    // of the Z function
    jacobian[0] = (this->Z(x + 1, y) - this->Z(x - 1, y)) / 2.0;
    jacobian[1] = (this->Z(x, y + 1) - this->Z(x, y - 1)) / 2.0;

    return jacobian;
}

//-------------------------------------------------------------------------
uint8_t ElevationMap::GetAvailable(int x, int y)
{
    // Check that the coordinates are in the boundaries
    if (!CheckCoord(x, y))
        return 1;

    return this->available[y * this->W + x];
}

//-------------------------------------------------------------------------
void ElevationMap::SetAvailable(int x, int y, uint8_t argAvailable)
{
    // Check that the coordinates are in the boundaries
    if (!CheckCoord(x, y))
        return;

    this->available[y * this->W + x] = argAvailable;
}