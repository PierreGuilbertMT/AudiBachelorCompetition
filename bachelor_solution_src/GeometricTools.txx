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

#include "GeometricTools.h"

#ifndef GEOMETRIC_TOOLS_TXX
#define GEOMETRIC_TOOLS_TXX

// STD
#include <iostream>
#include <sstream>

//-------------------------------------------------------------------------
template <typename K, unsigned int N>
Vector<K, N>::Vector()
{
    this->coordinates.resize(N, 0);
}

//-------------------------------------------------------------------------
template <typename K, unsigned int N>
std::string Vector<K, N>::toString()
{
    std::stringstream ss;
    ss << "[";
    for (unsigned int k = 0; k < N; ++k)
    {
        ss << this->coordinates[k] << ",";
    }
    ss << "]";
    return ss.str();
}

//-------------------------------------------------------------------------
template <typename K, unsigned int N>
K Vector<K, N>::dot(Vector u)
{
    K res = 0;
    for (unsigned int k = 0; k < N; ++k)
    {
        res += this->coordinates[k] * u[k];
    }
    return res;
}

//-------------------------------------------------------------------------
template <typename K, unsigned int N>
K Vector<K, N>::norm()
{
    return std::sqrt(this->dot(*this));
}

//-------------------------------------------------------------------------
template <typename K, unsigned int N>
K Vector<K, N>::distance(Vector u)
{
    return (*this - u).norm();
}

#endif // GEOMETRIC_TOOLS_TXX