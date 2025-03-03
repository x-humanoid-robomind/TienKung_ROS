/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include <assert.h>
#include <stdint.h>
#include <string>

namespace broccoli {
namespace analysis {
    //! Specification of mesh types for taskspace visualization
    /*! \ingroup broccoli_analysis_taskspace */
    class TaskSpaceMesh {
    public:
        //! Type specification
        enum class Type : uint8_t {
            VOXEL_BOX_FLAT = 0, //!< Simple box for each voxel (flat shaded - constant color for each box)
            VOXEL_BOX_SMOOTH, //!< Simple box for each voxel (smooth shaded - corner points color interpolated from neighbor voxels)
            VOXEL_ICOSPHERE, //!< Icosphere for each voxel (flat shaded - constant color for each icosphere)
            VOLUME, //!< Outer shell of volume as surface mesh (smooth shaded - vertex color interpolated from voxel data)
            POINT_CLOUD, //!< Point cloud (points coincide with voxel centers)
            TYPE_COUNT //!< Total count of elements
        };

        //! Returns the string representation of the given mesh type
        static inline std::string toString(const Type& type)
        {
            // Check mesh type
            switch (type) {
            case Type::VOXEL_BOX_FLAT:
                return "VOXEL_BOX_FLAT";
            case Type::VOXEL_BOX_SMOOTH:
                return "VOXEL_BOX_SMOOTH";
            case Type::VOXEL_ICOSPHERE:
                return "VOXEL_ICOSPHERE";
            case Type::VOLUME:
                return "VOLUME";
            case Type::POINT_CLOUD:
                return "POINT_CLOUD";
            default:
                break;
            }

            // Unknown selection
            assert(false);
            return "UNKNOWN";
        }
    };
} // namespace analysis
} // namespace broccoli

#endif // HAVE_EIGEN3
