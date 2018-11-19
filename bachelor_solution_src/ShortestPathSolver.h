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

#ifndef SHORTEST_PATH_SOLVER_H
#define SHORTEST_PATH_SOLVER_H

// LOCAL
#include "GeometricTools.h"
#include "ElevationMap.h"

// We make a difference between Point and Vector
// since we are working within an affine space
// algebraic structure
typedef Vector<double, 2> Point;
typedef Vector<double, 2> Vector2D;

/**
* \class SmartVehicle
* \brief SmartVehicle contains all information about the vehicle
*        position and goals. Moreover, the smartvehicle contains
*        the solver to heuristicly solved the geodesic relatively
*        to the manifold defined by the elevation map Z:
*
*        S(x, y) = [x, y, Z(x, y)]
*
*        this manifold can be upgraded to a Riemannian manifold
*        by crafting a time-based metric. To know more about that,
*        please find all the information in the document: 
*
* \author $Author: Pierre Guilbert $
* \version $Revision: 1.0 $
* \date $Date: 02-11-2018 $
* Contact: spguilbert@gmail.com
*/
class SmartVehicle
{
public:
    /// Constructor defining the keypoints and elevation map
    SmartVehicle(Point start, Point mid, Point end, ElevationMap argElevation);

    /// Find the geodesic of the time-based Riemannian
    /// manifold {[x, y, Z(x, y)]}
    void FindGeodesic();

    /// return the path of the vehicle
    std::vector<Point> GetVehiclePath();

private:
    /// Keypoints that must be reached
    Point VehicleStart;
    Point VehicleEnd;
    Point VehicleGoal;

    /// Path that has been taken
    std::vector<Vector<double, 2> > VehiclePath;

    /// Current point
    Vector<double, 2> CurrentPoint;

    /// maximum velocity of the vehicle in pixel/s
    double vmax = 1.0;

    /// impact of the slope on the vehicle velocity
    /// as described in the provided document, the
    /// slopeFactor depend on the friction factor, the
    /// mass of the vehicle and the power of the engine
    double slopeFactor = 0.1;

    /// impact of taking a direction that is not
    /// the correct one on the metric
    double directionFactor = 2.0;

    /// impact of crossing a river on the velocity
    /// of the car. This is a multiplicative factor
    double crossingRiverFactor = 0.3;

    /// Elevationn map
    ElevationMap Elevation;

    /// possible direction to move
    std::vector<std::pair<int, int> > directions;

    /// If the vehicle has reached the water, we will
    /// look from both side of the water where to go
    /// to unblock the vehicle and so that we can go toward
    /// the goal without crossing the water
    std::vector<Point> UnblockVehicleFromWater(int dirIndex);

    /// When unblocking the vehicle from the water
    /// move toward the water area banks either by
    /// updating the direction with clokwise or
    /// counter clock wise rotation
    int ClockWiseUpdate(Point& currPoint);
    int CounterClockWiseUpdate(Point& currPoint);

    /// Compute the time-based metric of the manidfold
    /// at the input point
    double ComputeManifoldMetric(Point currPoint, Vector<double, 2> dX, Vector<double, 2> dZ);

    /// Compute the best direction to take in order to
    /// locally minimize the manifold metric
    int ComputeBestDirection(Point currPoint);
};

#endif // SHORTEST_PATH_SOLVER_H