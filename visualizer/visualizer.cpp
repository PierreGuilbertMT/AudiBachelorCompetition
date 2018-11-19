// LOCAL
#include "visualizer.h"

// STD
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <array>
#include <string.h>


namespace visualizer {

// prevent the compiler to applies default
// memory alignement
#pragma pack(push,1)
struct FileHeader {
    uint8_t signature[2];
    uint32_t filesize;
    uint32_t reserved;
    uint32_t fileoffset_to_pixelarray;
};
struct BitmapInfoHeader {
    uint32_t dibheadersize;
    uint32_t width;
    uint32_t height;
    uint16_t planes;
    uint16_t bitsperpixel;
    uint32_t compression;
    uint32_t imagesize;
    uint32_t ypixelpermeter;
    uint32_t xpixelpermeter;
    uint32_t numcolorspallette;
    uint32_t mostimpcolor;
};
struct Bitmap {
    FileHeader fileheader;
    BitmapInfoHeader bitmapinfoheader;
};
#pragma pack(pop)

void writeBMP(
    std::ostream& out,
    size_t width,
    size_t height,
    const uint8_t* pixels,
    const uint8_t* colormap,
    size_t colormapSize,
    std::function<uint8_t(size_t, size_t, uint8_t)> pixelFilter)
{
    // 4: R, G, B, Depth
    size_t colormapSizeBytes = 4 * colormapSize;
    size_t offsetPixels = ((sizeof(Bitmap) + colormapSizeBytes + 3) / 4) * 4;
    size_t bitsPerPixel = 8;
    size_t rowBytes = ((bitsPerPixel * width + 31) / 32) * 4;
    size_t pixelBytes = rowBytes * height;
    size_t fileSize = offsetPixels + pixelBytes;
    
    // file header and bitmap info header
    Bitmap bm;
    
    // Set bitmap header information and
    // write the header into the file
    memset(&bm, 0, sizeof(bm));
    memcpy(bm.fileheader.signature, "BM", 2);
    bm.fileheader.filesize = fileSize;
    bm.fileheader.fileoffset_to_pixelarray = offsetPixels;
    bm.bitmapinfoheader.dibheadersize = sizeof(BitmapInfoHeader);
    bm.bitmapinfoheader.width = width;
    bm.bitmapinfoheader.height = height;
    bm.bitmapinfoheader.planes = 1;
    bm.bitmapinfoheader.bitsperpixel = bitsPerPixel;
    bm.bitmapinfoheader.compression = 0;
    bm.bitmapinfoheader.imagesize = pixelBytes;
    bm.bitmapinfoheader.ypixelpermeter = 0x130B; //2835 , 72 DPI
    bm.bitmapinfoheader.xpixelpermeter = 0x130B; //2835 , 72 DPI
    bm.bitmapinfoheader.numcolorspallette = colormapSize;
    out.write((const char*)&bm, sizeof(bm));
    
    // Write colormap, given as RGB values. BMP is BGR in this respect
    while (colormapSize -- != 0)
    {
        char entry[4];
        entry[0] = colormap[2]; // B
        entry[1] = colormap[1]; // G
        entry[2] = colormap[0]; // R
        entry[3] = 0;
        out.write((const char*)entry, 4);
        colormap += 3;
    }
    size_t rest = (sizeof(bm) + colormapSizeBytes) % 4;
    if (rest != 0)
    {
        const char* filler = "FIL";
        out.write(filler + rest - 1, 4 - rest);
    }
    
    // Write pixels, last row first
    char fill[32];
    memset(fill, ' ', sizeof(fill));
    fill[rowBytes - width] = 0;
    pixels = pixels + (height - 1) * width;
    while (height --)
    {
        size_t left = width;
        size_t x = 0;
        while (left --)
        {
            uint8_t pixelValue = pixelFilter(x, height, *pixels);
            out.write((const char*)&pixelValue, 1);
            ++ pixels;
            ++ x;
        }
        pixels -= 2 * width;
        out << fill;
    }
}

/**
* \function maxImage
* \brief Return the maximum pixel information (intensity, elevation, ...)
*        value over the image
*
* \author $Author: Autonomous Intelligent Driving GmbH $
* \version $Revision: 1.0 $
* \date $Date: ? $
* Contact: http://aid-driving.eu
*/
//-------------------------------------------------------------------------
uint8_t maxImage(
    const uint8_t* pixels,
    size_t width,
    size_t height)
{
    size_t count = width * height;
    unsigned max = 0;
    while (count --)
    {
        if (*pixels > max)
        {
            max = *pixels;
        }
        ++ pixels;
    }
    return max;
}

/**
* \function setColor
* \brief Set the r, g, b color of the i-th pixel
*        /!\ note that i indicates the pixel position
*        and not the pointer position
*
* \author $Author: Autonomous Intelligent Driving GmbH $
* \version $Revision: 1.0 $
* \date $Date: ? $
* Contact: http://aid-driving.eu
*/
//-------------------------------------------------------------------------
void setColor(std::vector<uint8_t>& vec, size_t i, uint8_t r, uint8_t g, uint8_t b)
{
    size_t baseIndex = i * 3;
    assert(baseIndex + 2 < vec.size());
    vec[baseIndex + 0] = r;
    vec[baseIndex + 1] = g;
    vec[baseIndex + 2] = b;
}

/**
* \function blendColor
* \brief linearly interpolate r, g, b color between startIndex
*        and endIndex from the startColor to the endColor
*
* \author $Author: Autonomous Intelligent Driving GmbH $
* \version $Revision: 1.0 $
* \date $Date: ? $
* Contact: http://aid-driving.eu
*/
//-------------------------------------------------------------------------
void blendColor(
    std::vector<uint8_t>& vec,
    size_t startIndex,
    size_t endIndex,
    uint8_t startRed,
    uint8_t startGreen,
    uint8_t startBlue,
    uint8_t endRed,
    uint8_t endGreen,
    uint8_t endBlue)
{
    size_t steps = endIndex - startIndex;
    for (size_t i = startIndex; i != endIndex; ++ i)
    {
        size_t indicesLeft = endIndex - i;
        size_t indicesPassed = i - startIndex;
        uint8_t r = (indicesLeft * startRed + indicesPassed * endRed) / steps;
        uint8_t g = (indicesLeft * startGreen + indicesPassed * endGreen) / steps;
        uint8_t b = (indicesLeft * startBlue + indicesPassed * endBlue) / steps;
        setColor(vec, i, r, g, b);
    }
}

/**
* \function generateElevationColormap
* \brief The generated colormap can be seen as a function
*        from [0, 255] to [0, 255]^3 that map an integer
*        representing the elevation to a color. The colormap
*        is built with 0 elevation being the path, 1 elevation
*        being the water. Then, linearly interpolates the colors
*        from 2 to 128 to goes from a sand color to a forest color
*        and finally, interpolates colors from 129 to 255 to
*        represents highland elevation color (from forest to snow)
*
* \author $Author: Autonomous Intelligent Driving GmbH $
* \version $Revision: 1.0 $
* \date $Date: ? $
* Contact: http://aid-driving.eu
*/
//-------------------------------------------------------------------------
std::vector<uint8_t> generateElevationColormap()
{
    std::vector<uint8_t> result(256 * 3);
    
    // Path value
    setColor(result, 0, 255, 0, 0);
    
    // Water value
    setColor(result, 1, 53, 160, 198);
    
    // Sand to forest, levels 2 to 127
    blendColor(
        result,
        2, 128,
        204, 201, 55,
        26, 130, 41);
    
    // Highlands, 128 to 255
    blendColor(
        result,
        128, 256,
        26, 130, 41,
        229, 226, 215);
    
    return result;
}

/**
* \function writeBMP
* \brief The generated colormap can be seen as a function
*        from [0, 255] to [0, 255]^3 that map an integer
*        representing the elevation to a color. The colormap
*        is built with 0 elevation being the path, 1 elevation
*        being the water. Then, linearly interpolates the colors
*        from 2 to 128 to goes from a sand color to a forest color
*        and finally, interpolates colors from 129 to 255 to
*        represents highland elevation color (from forest to snow)
*
* \author $Author: Autonomous Intelligent Driving GmbH $
* \version $Revision: 1.0 $
* \date $Date: ? $
* Contact: http://aid-driving.eu
*/
//-------------------------------------------------------------------------
void writeBMP(
    std::ostream& out,
    const uint8_t* elevationData,
    size_t width,
    size_t height,
    std::function<uint8_t(size_t, size_t, uint8_t)> pixelFilter)
{
    auto colormap(generateElevationColormap());
    writeBMP(out, width, height, elevationData, &colormap[0], colormap.size() / 3, pixelFilter);
}


} // namespace visualizer
