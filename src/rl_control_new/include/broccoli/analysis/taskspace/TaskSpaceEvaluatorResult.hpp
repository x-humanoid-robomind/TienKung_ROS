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
    //! Specification of result types for task-space evaluation algorithms
    /*! \ingroup broccoli_analysis_taskspace */
    class TaskSpaceEvaluatorResult {
    public:
        //! Type specification
        enum class Type : uint8_t {
            UNKNOWN = 0, //!< Unknown result (this should **never** be returned by a function)
            SUCCESS, //!< Algorithm was successful
            ERROR_SEGMENT_INVALID_NAME, //!< **Error:** The name of the segment is invalid
            ERROR_SEGMENT_INVALID_PARENT, //!< **Error:** The parent of the segment is invalid
            ERROR_SEGMENT_INVALID_LIMITS, //!< **Error:** The limits (minimum/maximum) of the segment are invalid
            ERROR_SEGMENT_INVALID_SAMPLES, //!< **Error:** The count of samples of the segment is invalid
            ERROR_SEGMENT_INVALID_DOFTYPE, //!< **Error:** The DOF type of the segment is invalid
            ERROR_CHAIN_INVALID_SEGMENT_COUNT, //!< **Error:** The chain has an invalid segment count
            ERROR_CHAIN_DUPLICATE_SEGMENT_NAME, //!< **Error:** Two segments have the same name
            ERROR_CHAIN_ROOT_HAS_PARENT, //!< **Error:** The root segment has a parent specified
            ERROR_CHAIN_PARENT_CHILD_MISMATCH, //!< **Error:** The segments in the frame are not linked correctly (parent - child)
            ERROR_CHAIN_INVALID_ROOT_NAME, //!< **Error:** The name of the root segment is invalid
            ERROR_CHAIN_INVALID_TCP_NAME, //!< **Error:** The name of the tcp segment is invalid
            ERROR_CHAIN_ROOT_NOT_FOUND, //!< **Error:** The root frame could not be found
            ERROR_CHAIN_TCP_NOT_FOUND, //!< **Error:** The tcp frame could not be found
            ERROR_CHAIN_PARENT_NOT_FOUND, //!< **Error:** The parent of a frame could not be found
            ERROR_SERIALIZATION, //!< **Error**: Serialization failed
            ERROR_DESERIALIZATION, //!< **Error**: Deserialization failed
            ERROR_FILE_NOEXIST, //!< **Error**: File does not exist
            ERROR_FILE_OPEN, //!< **Error**: Opening file failed
            TYPE_COUNT //!< Total count of elements
        };

        //! Returns the string representation of the given result type
        static inline std::string toString(const Type& result)
        {
            // Check result
            switch (result) {
            case Type::UNKNOWN:
                return "UNKNOWN";
            case Type::SUCCESS:
                return "SUCCESS";
            case Type::ERROR_SEGMENT_INVALID_NAME:
                return "ERROR_SEGMENT_INVALID_NAME";
            case Type::ERROR_SEGMENT_INVALID_PARENT:
                return "ERROR_SEGMENT_INVALID_PARENT";
            case Type::ERROR_SEGMENT_INVALID_LIMITS:
                return "ERROR_SEGMENT_INVALID_LIMITS";
            case Type::ERROR_SEGMENT_INVALID_SAMPLES:
                return "ERROR_SEGMENT_INVALID_SAMPLES";
            case Type::ERROR_SEGMENT_INVALID_DOFTYPE:
                return "ERROR_SEGMENT_INVALID_DOFTYPE";
            case Type::ERROR_CHAIN_INVALID_SEGMENT_COUNT:
                return "ERROR_CHAIN_INVALID_SEGMENT_COUNT";
            case Type::ERROR_CHAIN_DUPLICATE_SEGMENT_NAME:
                return "ERROR_CHAIN_DUPLICATE_SEGMENT_NAME";
            case Type::ERROR_CHAIN_ROOT_HAS_PARENT:
                return "ERROR_CHAIN_ROOT_HAS_PARENT";
            case Type::ERROR_CHAIN_PARENT_CHILD_MISMATCH:
                return "ERROR_CHAIN_PARENT_CHILD_MISMATCH";
            case Type::ERROR_CHAIN_INVALID_ROOT_NAME:
                return "ERROR_CHAIN_INVALID_ROOT_NAME";
            case Type::ERROR_CHAIN_INVALID_TCP_NAME:
                return "ERROR_CHAIN_INVALID_TCP_NAME";
            case Type::ERROR_CHAIN_ROOT_NOT_FOUND:
                return "ERROR_CHAIN_ROOT_NOT_FOUND";
            case Type::ERROR_CHAIN_TCP_NOT_FOUND:
                return "ERROR_CHAIN_TCP_NOT_FOUND";
            case Type::ERROR_CHAIN_PARENT_NOT_FOUND:
                return "ERROR_CHAIN_PARENT_NOT_FOUND";
            case Type::ERROR_SERIALIZATION:
                return "ERROR_SERIALIZATION";
            case Type::ERROR_DESERIALIZATION:
                return "ERROR_DESERIALIZATION";
            case Type::ERROR_FILE_NOEXIST:
                return "ERROR_FILE_NOEXIST";
            case Type::ERROR_FILE_OPEN:
                return "ERROR_FILE_OPEN";
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
