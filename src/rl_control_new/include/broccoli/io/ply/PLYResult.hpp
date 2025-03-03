/*
 * This file is part of broccoli.
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include <assert.h>
#include <stdint.h>
#include <string>

namespace broccoli {
namespace io {
    /*!
     * \addtogroup broccoli_io_ply
     * \{
     */

    //! Specification of result types for PLY algorithms
    enum class PLYResult : uint8_t {
        UNKNOWN = 0, //!< Unknown result (this should **never** be returned by a function)
        SUCCESS, //!< Algorithm was successful
        ERROR_INVALID_FORMAT, //!< **Error:** The specified PLY format is invalid
        ERROR_INVALID_SCALARTYPE, //!< **Error:** The specified PLY scalar type is invalid
        ERROR_INVALID_HEADER_PLY, //!< **Error:** Header is invalid (does not start with 'ply')
        ERROR_INVALID_HEADER_FORMAT, //!< **Error:** Header is invalid (could not identify 'format')
        ERROR_INVALID_HEADER_ELEMENTTYPE, //!< **Error:** Header is invalid (invalid element type specification)
        ERROR_INVALID_HEADER_PROPERTYTYPE, //!< **Error:** Header is invalid (invalid property type specification)
        ERROR_INVALID_HEADER_END, //!< **Error:** Header is invalid (could not find end of header 'end_header')
        ERROR_INVALID_BUFFER_ELEMENTCOUNT, //!< **Error:** The element count in the buffer does not match the element count in the element type specification
        ERROR_INVALID_BUFFER_PROPERTYCOUNT, //!< **Error:** The property count in the buffer does not match the property count in the element type specification
        ERROR_INVALID_BUFFER_PROPERTYVALUE, //!< **Error:** The property value has an invalid format
        ERROR_DECODE_SCALAR, //!< **Error:** Decoding scalar property failed
        ERROR_DECODE_LIST, //!< **Error:** Decoding list property failed
        ERROR_FILE_NOEXIST, //!< **Error:** File does not exist
        ERROR_FILE_OPEN, //!< **Error:** Could not open file
        ERROR_MESH_INVALID, //!< **Error:** The given input mesh is invalid
        ERROR_INVALID_VERTEX_ELEMENT_TYPE, //!< **Error:** Specification of element type "vertex" is invalid
        ERROR_INVALID_FACE_ELEMENT_TYPE, //!< **Error:** Specification of element type "face" is invalid
        ERROR_INVALID_FACE_VERTEX_COUNT, //!< **Error:** Invalid count of vertices in face (currently only 3 are supported)
        ERROR_INVALID_MATERIAL_ELEMENT_TYPE, //!< **Error:** Specification of element type "material" is invalid
        PLYRESULT_COUNT //!< Total count of elements
    };

    //! Returns the string representation of the given result type
    static inline std::string PLYResultToString(const PLYResult& result)
    {
        // Check result
        switch (result) {
        case PLYResult::UNKNOWN:
            return "UNKNOWN";
        case PLYResult::SUCCESS:
            return "SUCCESS";
        case PLYResult::ERROR_INVALID_FORMAT:
            return "ERROR_INVALID_FORMAT";
        case PLYResult::ERROR_INVALID_SCALARTYPE:
            return "ERROR_INVALID_SCALARTYPE";
        case PLYResult::ERROR_INVALID_HEADER_PLY:
            return "ERROR_INVALID_HEADER_PLY";
        case PLYResult::ERROR_INVALID_HEADER_FORMAT:
            return "ERROR_INVALID_HEADER_FORMAT";
        case PLYResult::ERROR_INVALID_HEADER_ELEMENTTYPE:
            return "ERROR_INVALID_HEADER_ELEMENTTYPE";
        case PLYResult::ERROR_INVALID_HEADER_PROPERTYTYPE:
            return "ERROR_INVALID_HEADER_PROPERTYTYPE";
        case PLYResult::ERROR_INVALID_HEADER_END:
            return "ERROR_INVALID_HEADER_END";
        case PLYResult::ERROR_INVALID_BUFFER_ELEMENTCOUNT:
            return "ERROR_INVALID_BUFFER_ELEMENTCOUNT";
        case PLYResult::ERROR_INVALID_BUFFER_PROPERTYCOUNT:
            return "ERROR_INVALID_BUFFER_PROPERTYCOUNT";
        case PLYResult::ERROR_INVALID_BUFFER_PROPERTYVALUE:
            return "ERROR_INVALID_BUFFER_PROPERTYVALUE";
        case PLYResult::ERROR_DECODE_SCALAR:
            return "ERROR_DECODE_SCALAR";
        case PLYResult::ERROR_DECODE_LIST:
            return "ERROR_DECODE_LIST";
        case PLYResult::ERROR_FILE_NOEXIST:
            return "ERROR_FILE_NOEXIST";
        case PLYResult::ERROR_FILE_OPEN:
            return "ERROR_FILE_OPEN";
        case PLYResult::ERROR_MESH_INVALID:
            return "ERROR_MESH_INVALID";
        case PLYResult::ERROR_INVALID_VERTEX_ELEMENT_TYPE:
            return "ERROR_INVALID_VERTEX_ELEMENT_TYPE";
        case PLYResult::ERROR_INVALID_FACE_ELEMENT_TYPE:
            return "ERROR_INVALID_FACE_ELEMENT_TYPE";
        case PLYResult::ERROR_INVALID_FACE_VERTEX_COUNT:
            return "ERROR_INVALID_FACE_VERTEX_COUNT";
        case PLYResult::ERROR_INVALID_MATERIAL_ELEMENT_TYPE:
            return "ERROR_INVALID_MATERIAL_ELEMENT_TYPE";
        default:
            break;
        }

        // Unknown selection
        assert(false);
        return "UNKNOWN";
    }

    //! \}
} // namespace io
} // namespace broccoli
