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
#include "ShortestPathSolver.h"

int clockwiseIncrIndex(int prevIndex);
int counterClockwiseIncrIndex(int prevIndex);

//-------------------------------------------------------------------------
SmartVehicle::SmartVehicle(Point start, Point mid, Point end, ElevationMap argElevation)
{
    // initialization
    this->VehicleStart = start;
    this->VehicleGoal = mid;
    this->VehicleEnd = end;
    this->CurrentPoint = start;
    this->Elevation = argElevation;

    // direction in 8-neighborhood
    directions.push_back(std::pair<int, int>(-1, -1)); // dir-0
    directions.push_back(std::pair<int, int>(-1, 0)); // dir-1
    directions.push_back(std::pair<int, int>(-1, 1)); // dir-2
    directions.push_back(std::pair<int, int>(0, 1)); // dir-3
    directions.push_back(std::pair<int, int>(1, 1)); // dir-4
    directions.push_back(std::pair<int, int>(1, 0)); // dir-5
    directions.push_back(std::pair<int, int>(1, -1)); // dir-6
    directions.push_back(std::pair<int, int>(0, -1)); // dir-7
}

//-------------------------------------------------------------------------
std::vector<Point> SmartVehicle::GetVehiclePath()
{
    return this->VehiclePath;
}

//-------------------------------------------------------------------------
void SmartVehicle::FindGeodesic()
{
    bool shouldContinue = true;
    unsigned int count = 0;
    unsigned int maxCount = 2100;

    // loop to find the geodesic
    while (count < maxCount)
    {
        int dirIndex = this->ComputeBestDirection(this->CurrentPoint);

        // all directions has been rejected
        // the rover is blocked, stop the algorithm
        if (dirIndex == -1)
        {
            std::cout << "Algorithm stopped, vehicle blocked" << std::endl;
            break;
        }

        // Add the previous point to the vehicle path
        this->VehiclePath.push_back(this->CurrentPoint);

        // check if the selected point is not in the water
        Point nextPoint;
        nextPoint[0] = this->CurrentPoint[0] + this->directions[dirIndex].first;
        nextPoint[1] = this->CurrentPoint[1] + this->directions[dirIndex].second;

        // unblock the rover
        if (this->Elevation.IsWater(nextPoint[0], nextPoint[1]))
        {
            std::vector<Point> unblockPath = this->UnblockVehicleFromWater(dirIndex);
            std::cout << "UNblocking path length: " << unblockPath.size() << std::endl;
            for (unsigned int i = 0; i < unblockPath.size(); ++i)
            {
                this->VehiclePath.push_back(unblockPath[i]);
            }
            this->CurrentPoint = unblockPath[unblockPath.size() - 1];
        }
        else
        {
            // Update the current point
            this->CurrentPoint = nextPoint;
        }

        // Disable the current position for future condidate
        this->Elevation.SetAvailable(this->CurrentPoint[0], this->CurrentPoint[1], 1);

        // check if the algorithm should continue
        shouldContinue = !((this->CurrentPoint[0] == this->VehicleGoal[0]) && (this->CurrentPoint[1] == this->VehicleEnd[1]));

        // increase count
        count++;
    }
}

//-------------------------------------------------------------------------
int SmartVehicle::ComputeBestDirection(Point currPoint)
{
    // compute the elevation gradient for this point
    Vector<double, 2> dZ = this->Elevation.dZ(currPoint[0], currPoint[1]);

    double minDist = 10000000000;
    int dirIndex = -1;
    for (unsigned int neigh = 0; neigh < this->directions.size(); ++neigh)
    {
        // get the current direction
        Vector<double, 2> dX;
        dX[0] = this->directions[neigh].first;
        dX[1] = this->directions[neigh].second;

        // check that the candidate position is available
        if (this->Elevation.GetAvailable(this->CurrentPoint[0] + dX[0], this->CurrentPoint[1] + dX[1]) != 0)
        {
            continue;
        }

        // Compute the time-based manifold metric
        double dt = this->ComputeManifoldMetric(this->CurrentPoint, dX, dZ);

        if (dt < minDist)
        {
            minDist = dt;
            dirIndex = neigh;
        }
    }

    return dirIndex;
}

//-------------------------------------------------------------------------
double SmartVehicle::ComputeManifoldMetric(Point currPoint, Vector<double, 2> dX, Vector<double, 2> dZ)
{
    // moving direction of the car. It is defined as
    // [dx, dy, dX' * dZ], with dX = [dx, dy]. It is actually
    // the directional derivation of the parametrization function
    // of the S manifold along the direction dX
    Vector<double, 3> ex;
    ex[0] = dX[0]; ex[1] = dX[1]; ex[2] = dX.dot(dZ);

    // angle between the car direction and the ground plane
    double sinAngle = ex[2] / ex.norm();

    // vehicle velocity
    double v = this->vmax - this->slopeFactor * sinAngle;

    // time-based metric of the Riemannnian manifold
    // on which the car positions are lying.
    double dt = (1 + dZ[0] * dZ[0]) * dX[0] * dX[0];
    dt += 2 * dZ[0] * dZ[1] * dX[0] * dX[1];
    dt += (1 + dZ[1] * dZ[1]) * dX[1] * dX[1];
    dt = std::sqrt(dt / (v * v));

    // add a term to pull the vehicle toward its goal
    Vector<double, 2> goalDir = this->VehicleGoal - currPoint;
    dt -= dX.dot(goalDir) / (goalDir.norm() * dX.norm());

    return dt;
}

//-------------------------------------------------------------------------
std::vector<Point> SmartVehicle::UnblockVehicleFromWater(int dirIndex)
{
    std::cout << "Start Unblocking" << std::endl;
    // We will go along the both side of the water area
    // and see what is the correct path to unblock the vehicle
    std::vector<Point> path1, path2;
    Point currPoint1 = this->CurrentPoint;
    Point currPoint2 = this->CurrentPoint;
    int previousDirIndex1 = dirIndex;
    int previousDirIndex2 = dirIndex;

    bool shouldContinue = true;
    int count = 0;
    while (shouldContinue)
    {
        // Path 1 continuation
        bool isUnblock1 = this->ClockWiseUpdate(currPoint1);
        path1.push_back(currPoint1);

        // Path 2
        // change direction in counter-closkwise rotation
        bool isUnblock2 = this->CounterClockWiseUpdate(currPoint2);
        path2.push_back(currPoint2);

        if (isUnblock1)
            return path1;

        if (isUnblock2)
            return path2;

        count++;
    }
}

//-------------------------------------------------------------------------
bool SmartVehicle::ClockWiseUpdate(Point& currPoint)
{
    // First, compute the direction we would like to take
    int wantedDir = this->ComputeBestDirection(currPoint);

    // No direction can be taken
    if (wantedDir == -1)
        return false;

    // then, check if that direction can be taken
    Point candidatePoint;
    candidatePoint[0] = currPoint[0] + this->directions[wantedDir].first;
    candidatePoint[1] = currPoint[1] + this->directions[wantedDir].second;
    if (!this->Elevation.IsWater(candidatePoint[0], candidatePoint[1]))
    {
        // Stop here, its ok
        currPoint = candidatePoint;
        return true;
    }

    // If the direction can not be taken,
    // change direction in clockwise rotation
    bool shouldContinue = true;
    int prevDir = wantedDir;
    int nextDir;
    int count = 0;
    while (shouldContinue)
    {
        nextDir = clockwiseIncrIndex(prevDir);
        prevDir = nextDir;
        candidatePoint[0] = currPoint[0] + this->directions[nextDir].first;
        candidatePoint[1] = currPoint[1] + this->directions[nextDir].second;
        if (!this->Elevation.IsWater(candidatePoint[0], candidatePoint[1]))
        {
            break;
        }

        count++;
        if (count >= 8)
            break;
    }

    currPoint = candidatePoint;
    return false;
}

//-------------------------------------------------------------------------
bool SmartVehicle::CounterClockWiseUpdate(Point& currPoint)
{
    // First, compute the direction we would like to take
    int wantedDir = this->ComputeBestDirection(currPoint);

    // No direction can be taken
    if (wantedDir == -1)
        return false;

    // then, check if that direction can be taken
    Point candidatePoint;
    candidatePoint[0] = currPoint[0] + this->directions[wantedDir].first;
    candidatePoint[1] = currPoint[1] + this->directions[wantedDir].second;
    if (!this->Elevation.IsWater(candidatePoint[0], candidatePoint[1]))
    {
        // Stop here, its ok
        currPoint = candidatePoint;
        return true;
    }

    // If the direction can not be taken,
    // change direction in clockwise rotation
    bool shouldContinue = true;
    int prevDir = wantedDir;
    int nextDir;
    int count = 0;
    while (shouldContinue)
    {
        nextDir = counterClockwiseIncrIndex(prevDir);
        prevDir = nextDir;
        candidatePoint[0] = currPoint[0] + this->directions[nextDir].first;
        candidatePoint[1] = currPoint[1] + this->directions[nextDir].second;
        if (!this->Elevation.IsWater(candidatePoint[0], candidatePoint[1]))
        {
            break;
        }

        count++;
        if (count >= 8)
            break;
    }

    currPoint = candidatePoint;
    return false;
}

//-------------------------------------------------------------------------
int clockwiseIncrIndex(int prevIndex)
{
    int newIndex = prevIndex - 1;
    if (newIndex < 0)
        newIndex = 7;

    return newIndex;
}

//-------------------------------------------------------------------------
int counterClockwiseIncrIndex(int prevIndex)
{
    int newIndex = prevIndex + 1;
    if (newIndex > 7)
        newIndex = 0;

    return newIndex;
}