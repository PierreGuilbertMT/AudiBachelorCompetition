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

enum UnblockingStatus
{
    ContitnueUnblocking = 0,
    Unblocked = 1,
    DefinitivelyBlocked = 2
};

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
    unsigned int maxCount = 3000;

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
        if (this->Elevation.CanBeCrossed(nextPoint[0], nextPoint[1]))
        {
            std::vector<Point> unblockPath = this->UnblockVehicleFromWater(dirIndex);
            if (unblockPath.size() == 0)
            {
                std::cout << "Vehicle blocked by water, end of the algorithm" << std::endl;
                break;
            }

            for (unsigned int i = 0; i < unblockPath.size(); ++i)
            {
                // unavailable the taken path
                this->Elevation.SetAvailable(unblockPath[i][0], unblockPath[i][1], 1);

                // add it to the path
                this->VehiclePath.push_back(unblockPath[i]);
            }
            this->CurrentPoint = unblockPath[unblockPath.size() - 1];
            std::cout << "Length of the path: " << unblockPath.size() << std::endl;
            //break;
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
        if (!(count < maxCount))
        {
            std::cout << "Algorithm stopped because too much iteration" << std::endl;
        }
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
        double dt = this->ComputeManifoldMetric(currPoint, dX, dZ);

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

    // impact on crossing the river on the velocity
    if (this->Elevation.IsRiver(currPoint[0] + dX[0], currPoint[1] + dX[1]))
        v = this->crossingRiverFactor * v;

    // time-based metric of the Riemannnian manifold
    // on which the car positions are lying.
    double dt = (1 + dZ[0] * dZ[0]) * dX[0] * dX[0];
    dt += 2 * dZ[0] * dZ[1] * dX[0] * dX[1];
    dt += (1 + dZ[1] * dZ[1]) * dX[1] * dX[1];
    dt = std::sqrt(dt / (v * v));

    // add a term to pull the vehicle toward its goal
    Vector<double, 2> goalDir = this->VehicleGoal - currPoint;
    dt -= this->directionFactor * dX.dot(goalDir) / (goalDir.norm() * dX.norm());

    return dt;
}

//-------------------------------------------------------------------------
std::vector<Point> SmartVehicle::UnblockVehicleFromWater(int dirIndex)
{
    // We will go along both side of the water area
    // and see what is the correct path to unblock the vehicle
    std::vector<Point> path1, path2;
    Point currPoint1 = this->CurrentPoint;
    Point currPoint2 = this->CurrentPoint;
    int previousDirIndex1 = dirIndex;
    int previousDirIndex2 = dirIndex;

    bool shouldContinue = true;
    int count = 0;

    unsigned int unblockCount1 = 0;
    unsigned int unblockCount2 = 0;
    unsigned int maxUnblockCount = 50;

    while (shouldContinue)
    {
        std::cout << "============= " << count << " =========" << std::endl;
        // Path 1 continuation
        std::cout << "currPoint1 Before: " << currPoint1.toString() << std::endl;
        int unblockStatus1 = this->ClockWiseUpdate(currPoint1);
        std::cout << "currPoint1 After: " << currPoint1.toString() << std::endl;
        this->Elevation.SetAvailable(currPoint1[0], currPoint1[1], 1);
        path1.push_back(currPoint1);

        // Path 2
        // change direction in counter-closkwise rotation
        std::cout << "currPoint2 Before: " << currPoint2.toString() << std::endl;
        int unblockStatus2 = UnblockingStatus::DefinitivelyBlocked;// = this->CounterClockWiseUpdate(currPoint2);
        std::cout << "currPoint2 Before: " << currPoint2.toString() << std::endl;
        //this->Elevation.SetAvailable(currPoint2[0], currPoint2[1], 1);
        path2.push_back(currPoint2);

        // We want the vehicle to be definitively unblocked
        // before considering as being unblocked
        if (unblockStatus1 == UnblockingStatus::Unblocked)
        {
            unblockCount1++;
        }
        if (unblockStatus2 == UnblockingStatus::Unblocked)
        {
            unblockCount2++;
        }

        if (unblockCount1 > maxUnblockCount)
        {
            std::cout << "Unblocked from water !" << std::endl;
            return path1;
        }

        if (unblockCount2 > maxUnblockCount)
        {
            std::cout << "Unblocked from water !" << std::endl;
            return path2;
        }

        if ((unblockStatus1 == UnblockingStatus::DefinitivelyBlocked) &&
            (unblockStatus2 == UnblockingStatus::DefinitivelyBlocked))
        {
            std::cout << "Unblocking stopped because vehicle definitively blocked" << std::endl;
            return std::vector<Point>();
        }

        count++;
        std::cout << std::endl << std::endl;
    }
}

//-------------------------------------------------------------------------
int SmartVehicle::ClockWiseUpdate(Point& currPoint)
{
    // First, compute the direction we would like to take
    int wantedDir = this->ComputeBestDirection(currPoint);

    // No direction can be taken
    if (wantedDir == -1)
    {
        std::cout << "Clockwise blocked because no wanted dir" << std::endl;
        return UnblockingStatus::DefinitivelyBlocked;
    }

    // then, check if that direction can be taken
    Point candidatePoint;
    candidatePoint[0] = currPoint[0] + this->directions[wantedDir].first;
    candidatePoint[1] = currPoint[1] + this->directions[wantedDir].second;
    if ((!this->Elevation.CanBeCrossed(candidatePoint[0], candidatePoint[1])) &&
        (this->Elevation.GetAvailable(candidatePoint[0], candidatePoint[1]) == 0))
    {
        // Stop here, its ok
        std::cout << "Clockwise wanted dir possible, stop" << std::endl;
        currPoint = candidatePoint;
        return UnblockingStatus::Unblocked;
    }

    // If the direction can not be taken,
    // change direction in clockwise rotation
    bool shouldContinue = true;
    int prevDir = wantedDir;
    int nextDir;
    int count = 0;

    while (shouldContinue)
    {
        // Check that there is direction left
        if (count >= 8)
        {
            std::cout << "Clockwise, all direction are not available or in water..." << std::endl;
            return UnblockingStatus::DefinitivelyBlocked;
        }

        nextDir = clockwiseIncrIndex(prevDir);
        prevDir = nextDir;
        candidatePoint[0] = currPoint[0] + this->directions[nextDir].first;
        candidatePoint[1] = currPoint[1] + this->directions[nextDir].second;

        // check that the direction is available
        if (this->Elevation.GetAvailable(candidatePoint[0], candidatePoint[1]) != 0)
        {
            count++;
            continue;
        }

        // If the direction can be taken, take it
        if (!this->Elevation.CanBeCrossed(candidatePoint[0], candidatePoint[1]))
        {
            break;
        }

        count++;
    }

    currPoint = candidatePoint;
    return UnblockingStatus::ContitnueUnblocking;
}

//-------------------------------------------------------------------------
int SmartVehicle::CounterClockWiseUpdate(Point& currPoint)
{
    // First, compute the direction we would like to take
    int wantedDir = this->ComputeBestDirection(currPoint);

    // No direction can be taken
    if (wantedDir == -1)
    {
        std::cout << "Counter Clockwise blocked because no wanted dir" << std::endl;
        return UnblockingStatus::DefinitivelyBlocked;
    }

    // then, check if that direction can be taken
    Point candidatePoint;
    candidatePoint[0] = currPoint[0] + this->directions[wantedDir].first;
    candidatePoint[1] = currPoint[1] + this->directions[wantedDir].second;
    if ((!this->Elevation.CanBeCrossed(candidatePoint[0], candidatePoint[1])) &&
        (this->Elevation.GetAvailable(candidatePoint[0], candidatePoint[1]) == 0))
    {
        // Stop here, its ok
        std::cout << "Counter Clockwise wanted dir possible, stop" << std::endl;
        currPoint = candidatePoint;
        return UnblockingStatus::Unblocked;
    }

    // If the direction can not be taken,
    // change direction in clockwise rotation
    bool shouldContinue = true;
    int prevDir = wantedDir;
    int nextDir;
    int count = 0;

    while (shouldContinue)
    {
        // Check that there is direction left
        if (count >= 8)
        {
            std::cout << "Counter Clockwise, all direction are not available or in water..." << std::endl;
            return UnblockingStatus::DefinitivelyBlocked;
        }

        nextDir = counterClockwiseIncrIndex(prevDir);
        prevDir = nextDir;
        candidatePoint[0] = currPoint[0] + this->directions[nextDir].first;
        candidatePoint[1] = currPoint[1] + this->directions[nextDir].second;

        // check that the direction is available
        if (this->Elevation.GetAvailable(candidatePoint[0], candidatePoint[1]) != 0)
        {
            count++;
            continue;
        }

        // If the direction can be taken, take it
        if (!this->Elevation.CanBeCrossed(candidatePoint[0], candidatePoint[1]))
        {
            break;
        }

        count++;
    }

    currPoint = candidatePoint;
    return UnblockingStatus::ContitnueUnblocking;
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