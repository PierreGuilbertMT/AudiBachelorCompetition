// LOCAL
#include "visualizer.h"
#include "GeometricTools.h"
#include "ShortestPathSolver.h"
#include "ElevationMap.h"

// STD
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <exception>

#ifdef _MSC_VER
static const char* PATH_SEP = "\\";
#else
static const char* PATH_SEP = "/";
#endif

// Bits used in the overrides image bytes
enum OverrideFlags
{
    OF_RIVER_MARSH = 0x10,
    OF_INLAND = 0x20,
    OF_WATER_BASIN = 0x40,
    OF_TAKEN_PATH = 255
};

// Some constants
enum {
    IMAGE_DIM = 2048, // Width and height of the elevation and overrides image
    
    ROVER_X = 159,
    ROVER_Y = 1520,
    BACHELOR_X = 1303,
    BACHELOR_Y = 85,
    WEDDING_X = 1577,
    WEDDING_Y = 1294
};

/**
* \function fileSize
* \brief Return the size of the file filename, throw
*        an exception if the file can't be open
*
* \author $Author: Autonomous Intelligent Driving GmbH $
* \version $Revision: 1.0 $
* \date $Date: ? $
* Contact: http://aid-driving.eu
*/
//-------------------------------------------------------------------------
std::ifstream::pos_type fileSize(const std::string& filename)
{
    // open the file at the end and return the position of
    // the current character as it is the last one
    std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
    if (!in.good())
    {
        throw std::exception();
    }
    return in.tellg(); 
}

/**
* \function loadFile
* \brief load the file filename and throw an exception if the
*        size of the file does not match the expected size
*
* \author $Author: Autonomous Intelligent Driving GmbH $
* \version $Revision: 1.0 $
* \date $Date: ? $
* Contact: http://aid-driving.eu
*/
//-------------------------------------------------------------------------
std::vector<uint8_t> loadFile(const std::string& filename, size_t expectedFileSize)
{
    size_t fsize = fileSize(filename);
    if (fsize != expectedFileSize)
    {
        throw std::exception();
    }
    std::vector<uint8_t> data(fsize);
    std::ifstream ifile(filename, std::ifstream::binary);
    if (!ifile.good())
    {
        throw std::exception();
    }
    ifile.read(reinterpret_cast<char*>(&data[0]), fsize);
    return data;
}

/**
* \function donut
* \brief Return true if the euclidean distance derived from the canonical
*        dot product of R^2 between (x, y) and (x1, y1) is comprised between
*        150 and 400 dist units.
*
* \author $Author: Autonomous Intelligent Driving GmbH $
* \version $Revision: 1.0 $
* \date $Date: ? $
* Contact: http://aid-driving.eu
*/
//-------------------------------------------------------------------------
bool donut(int x, int y, int x1, int y1)
{
    int dx = x - x1;
    int dy = y - y1;
    int r2 = dx * dx + dy * dy;
    return r2 >= 150 && r2 <= 400;
}

//-------------------------------------------------------------------------
int main(int argc, char** argv)
{
    // expected size of the input file
    const size_t expectedFileSize = IMAGE_DIM * IMAGE_DIM;
    
    // build the working dir using the absolute path
    // to the application launched
    std::string anchor = std::string(".") + PATH_SEP;
    std::string pname = argv[0];
    size_t lastpos = pname.find_last_of("/\\");
    if (lastpos != std::string::npos)
    {
        // remove the application name to just keep the curr folder
        anchor = pname.substr(0, lastpos) + PATH_SEP;
    }

    // load geographical data of the island
    std::vector<uint8_t> elevation = loadFile(anchor + "assets" + PATH_SEP + "elevation.data", expectedFileSize);
    std::vector<uint8_t> overrides = loadFile(anchor + "assets" + PATH_SEP + "overrides.data", expectedFileSize);

    // Keypoints to reach
    Vector<double, 2> startPoint, goalPoint, endPoint;
    startPoint[0] = ROVER_X; startPoint[1] = ROVER_Y;
    goalPoint[0] = BACHELOR_X; goalPoint[1] = BACHELOR_Y;
    endPoint[0] = WEDDING_X; endPoint[1] = WEDDING_Y;

    // Elevation map
    ElevationMap Elevation(elevation, overrides, IMAGE_DIM, IMAGE_DIM);

    // Smart Vehicle
    SmartVehicle vehicle(Elevation);
    vehicle.FindGeodesic(startPoint, goalPoint);
    vehicle.FindGeodesic(goalPoint, endPoint);

    // Update elevation map
    std::vector<Vector<double, 2> > vehiclePath = vehicle.GetVehiclePath();
    for (unsigned int k = 0; k < vehiclePath.size(); ++k)
    {
        int x = static_cast<int>(vehiclePath[k][0]);
        int y = static_cast<int>(vehiclePath[k][1]);
        
        overrides[y * IMAGE_DIM + x] = OF_TAKEN_PATH;
    }

    // output file (the overrrided image of the island)
    std::ofstream of("pic.bmp", std::ofstream::binary);

    // Write the image
    visualizer::writeBMP(
        of,
        &elevation[0],
        IMAGE_DIM,
        IMAGE_DIM,
        [&] (size_t x, size_t y, uint8_t elevation) {
        
            // Marks interesting positions on the map
            if (donut(x, y, ROVER_X, ROVER_Y) ||
                donut(x, y, BACHELOR_X, BACHELOR_Y) ||
                donut(x, y, WEDDING_X, WEDDING_Y))
            {
                return uint8_t(visualizer::IPV_PATH);
            }
            
            // signifies vehicle path
            if (overrides[y * IMAGE_DIM + x] == OF_TAKEN_PATH)
            {
                return uint8_t(visualizer::IPV_PATH);
            }

            // Signifies water
            if ((overrides[y * IMAGE_DIM + x] & (OF_WATER_BASIN | OF_RIVER_MARSH)) ||
                elevation == 0)
            {
                return uint8_t(visualizer::IPV_WATER);
            }

            // Signifies normal ground color
            if (elevation < visualizer::IPV_ELEVATION_BEGIN)
            {
                elevation = visualizer::IPV_ELEVATION_BEGIN;
            }

            return elevation;
    });

    // Flush and close the file
    of.flush();
    of.close();

    // display the generated output image
#if __APPLE__
    auto res = system("open pic.bmp");
    (void)res;
#elif _MSC_VER
    system("start pic.bmp");
#endif

    return EXIT_SUCCESS;
}

