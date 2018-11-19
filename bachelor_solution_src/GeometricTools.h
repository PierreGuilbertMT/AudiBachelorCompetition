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

#ifndef GEOMETRIC_TOOLS_H
#define GEOMETRIC_TOOLS_H

// STD
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

/**
* \class Vector
* \brief Let's E being a K vector space of finite dimension N, with
*        K having a field algebraic structure
*        Using the incomplete basis theorem we can create easily create
*        a basis Be of E. Given this basis, it exists a canonical vectorial
*        space isomorphism from E to K^N:
*
*        IsoE : E -> K^N, x -> Mat_Be(x)
*
*        Here, we propose to represent a vector x of E, using
*        the K^N vector IsoE(x) = Mat_Be(x)
*        Moreover, we equipped E with an Euclidean structure by defining
*        a dot / scalar product. Here we only manage the canonical scalar
*        product. e.g when M_Be(<.>) is the identity. If one want to handle
*        another scalar product, it only required to change the basis of E.
*        Indeed, using the spectral theorem, it exists an orthonormal basis
*        of E for the required scalar product
*        
*
* \author $Author: Pierre Guilbert $
* \version $Revision: 1.0 $
* \date $Date: 02-11-2018 $
* Contact: spguilbert@gmail.com
*/
template <typename K, unsigned int N>
class Vector
{
public:
    /// default constructor
    Vector();

    /// Add two vectors
    Vector operator+(Vector u)
    {
        Vector<K, N> sum;
        for (unsigned int k = 0; k < N; ++k)
        {
            sum[k] = this->coordinates[k] + u[k];
        }
        return sum;
    }

    /// multiplication with a scalar
    Vector operator*(double a)
    {
        Vector<K, N> res;
        for (unsigned int k = 0; k < N; ++k)
        {
            res[k] = a * this->coordinates[k];
        }
        return res;
    }

    /// multiplication with a scalar
    Vector operator/(double a)
    {
        Vector<K, N> res;
        for (unsigned int k = 0; k < N; ++k)
        {
            res[k] = this->coordinates[k] / a;
        }
        return res;
    }

    /// substract two vectors
    Vector operator-(Vector u)
    {
        Vector<K, N> subs;
        for (unsigned int k = 0; k < N; ++k)
        {
            subs[k] = this->coordinates[k] - u[k];
        }
        return subs;
    }

    /// coordinate setter
    K& operator[](unsigned int k)
    {
        return this->coordinates[k];
    }

    /// coordinate getter
    K operator[](unsigned int k) const
    {
        return this->coordinates[k];
    }

    /// canonical scalar product of K^N. If the scalar product
    /// matrix is not identity, one can change the basis of E to
    /// have a identity scalar product within this basis. This is
    /// possible due to the spectral theorem
    K dot(Vector u);

    /// norm associated to the scalar product
    K norm();

    /// distance associated to the euclidean norm
    K distance(Vector u);

    /// Display the vector as a stream
    std::string toString();

protected:
    /// coordinates of the x vector according
    /// to the Be basis
    std::vector<K> coordinates;
};

// templated methods implementation
#include "GeometricTools.txx"

#endif // GEOMETRIC_TOOLS_H