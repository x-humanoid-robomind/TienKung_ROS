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
    //! Specification of metric types
    /*! \ingroup broccoli_analysis_taskspace */
    class TaskSpaceMetric {
    public:
        //! Type specification
        enum class Type : uint8_t {
            REACHABILITY = 0, //!< Reachability (from count of collected samples)
            CONDITION_INDEX_MINIMUM, //!< Minimum value for \ref TaskSpaceSample::conditionIndex()
            CONDITION_INDEX_MEAN, //!< Mean value for \ref TaskSpaceSample::conditionIndex()
            CONDITION_INDEX_MAXIMUM, //!< Maximum value for \ref TaskSpaceSample::conditionIndex()
            MANIPULABILITY_MEASURE_MINIMUM, //!< Minimum value for \ref TaskSpaceSample::manipulabilityMeasure()
            MANIPULABILITY_MEASURE_MEAN, //!< Minimum value for \ref TaskSpaceSample::manipulabilityMeasure()
            MANIPULABILITY_MEASURE_MAXIMUM, //!< Maximum value for \ref TaskSpaceSample::manipulabilityMeasure()
            JOINT_RANGE_AVAILABILITY_MINIMUM, //!< Minimum value for \ref TaskSpaceSample::jointRangeAvailability()
            JOINT_RANGE_AVAILABILITY_MEAN, //!< Minimum value for \ref TaskSpaceSample::jointRangeAvailability()
            JOINT_RANGE_AVAILABILITY_MAXIMUM, //!< Maximum value for \ref TaskSpaceSample::jointRangeAvailability()
            TYPE_COUNT //!< Total count of elements
        };

        //! Returns the string representation of the given metric type
        static inline std::string toString(const Type& type)
        {
            // Check metric type
            switch (type) {
            case Type::REACHABILITY:
                return "REACHABILITY";
            case Type::CONDITION_INDEX_MINIMUM:
                return "CONDITION_INDEX_MINIMUM";
            case Type::CONDITION_INDEX_MEAN:
                return "CONDITION_INDEX_MEAN";
            case Type::CONDITION_INDEX_MAXIMUM:
                return "CONDITION_INDEX_MAXIMUM";
            case Type::MANIPULABILITY_MEASURE_MINIMUM:
                return "MANIPULABILITY_MEASURE_MINIMUM";
            case Type::MANIPULABILITY_MEASURE_MEAN:
                return "MANIPULABILITY_MEASURE_MEAN";
            case Type::MANIPULABILITY_MEASURE_MAXIMUM:
                return "MANIPULABILITY_MEASURE_MAXIMUM";
            case Type::JOINT_RANGE_AVAILABILITY_MINIMUM:
                return "JOINT_RANGE_AVAILABILITY_MINIMUM";
            case Type::JOINT_RANGE_AVAILABILITY_MEAN:
                return "JOINT_RANGE_AVAILABILITY_MEAN";
            case Type::JOINT_RANGE_AVAILABILITY_MAXIMUM:
                return "JOINT_RANGE_AVAILABILITY_MAXIMUM";
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
