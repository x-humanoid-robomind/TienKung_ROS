/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "../encoding.hpp"
#include "ColumnBasedLogFileData.hpp"
#include <array>

namespace broccoli {
namespace io {
    /*!
     * \brief Implementation of ColumnBasedLogFileData, which contains a single array
     * \ingroup broccoli_io_logging
     *
     * This is a convenience class for logging a single array to a column based logfile.
     * \par Configuration
     *
     * Template parameters are used to specialize the class:
     * - \tparam ElementType specifies the datatype for the elements in the array (e.g. `double`)
     * - \tparam Dimension specifies the count of elements in the array
     */
    template <class ElementType, unsigned int Dimension>
    class LogFileDataArray : public ColumnBasedLogFileData {
    public:
        //! Default constructor
        LogFileDataArray()
        {
        }

        //! Destructor
        virtual ~LogFileDataArray()
        {
        }

        // Members
        std::array<ElementType, Dimension> m_data; //!< Array of data elements of the specified \p ElementType

    protected:
        void encodeColumnDataToLogStream(ColumnBasedLogStream& stream) const override
        {
            stream.addData(m_data, "array");
        }

#ifdef HAVE_EIGEN3
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
#endif // HAVE_EIGEN3
    };
} // namespace io
} // namespace broccoli
