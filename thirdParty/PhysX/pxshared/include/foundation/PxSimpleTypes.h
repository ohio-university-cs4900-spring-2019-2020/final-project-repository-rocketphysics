//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef PXFOUNDATION_PXSIMPLETYPES_H
#define PXFOUNDATION_PXSIMPLETYPES_H

/** \addtogroup foundation
  @{
*/

// Platform specific types:
// Design note: Its OK to use int for general loop variables and temps.

#include "foundation/PxPreprocessor.h"
#if PX_VC
#pragma warning(push)
#pragma warning(disable : 4668) // suppressing warning generated by Microsoft Visual Studio when including this standard
// header
#endif

#if PX_LINUX
#define __STDC_LIMIT_MACROS
#endif

#include <stdint.h>
#if PX_VC
#pragma warning(pop)
#endif

#if PX_VC // we could use inttypes.h starting with VC12
#define PX_PRIu64 "I64u"
#else
#if !PX_PS4 && !PX_APPLE_FAMILY
#define __STDC_FORMAT_MACROS
#endif
#include <inttypes.h>
#define PX_PRIu64 PRIu64
#endif

namespace physx
{
typedef int64_t PxI64;
typedef uint64_t PxU64;
typedef int32_t PxI32;
typedef uint32_t PxU32;
typedef int16_t PxI16;
typedef uint16_t PxU16;
typedef int8_t PxI8;
typedef uint8_t PxU8;
typedef float PxF32;
typedef double PxF64;
typedef float PxReal;
}

// Type ranges

// These are here because we sometimes have non-IEEE compliant platforms to deal with.
// Removal is under consideration (issue GWSD-34)

#define PX_MAX_F32 3.4028234663852885981170418348452e+38F
// maximum possible float value
#define PX_MAX_F64 DBL_MAX // maximum possible double value

#define PX_EPS_F32 FLT_EPSILON // maximum relative error of float rounding
#define PX_EPS_F64 DBL_EPSILON // maximum relative error of double rounding

#define PX_MAX_REAL PX_MAX_F32
#define PX_EPS_REAL PX_EPS_F32
#define PX_NORMALIZATION_EPSILON float(1e-20f)

// Legacy type ranges used by PhysX
#define PX_MAX_I8 INT8_MAX
#define PX_MIN_I8 INT8_MIN
#define PX_MAX_U8 UINT8_MAX
#define PX_MIN_U8 UINT8_MIN
#define PX_MAX_I16 INT16_MAX
#define PX_MIN_I16 INT16_MIN
#define PX_MAX_U16 UINT16_MAX
#define PX_MIN_U16 UINT16_MIN
#define PX_MAX_I32 INT32_MAX
#define PX_MIN_I32 INT32_MIN
#define PX_MAX_U32 UINT32_MAX
#define PX_MIN_U32 UINT32_MIN

/** @} */
#endif // #ifndef PXFOUNDATION_PXSIMPLETYPES_H
