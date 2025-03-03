/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "../../memory/MultiVector.hpp"
#include "../encoding.hpp"
#include "../serialization/serialization.hpp"
#include "PNMFileEncoding.hpp"
#include <array>
#include <assert.h>
#include <ctype.h>
#include <limits>
#include <stdint.h>

namespace broccoli {
namespace io {
    /*!
     * \addtogroup broccoli_io_pnm
     * \{
     */

    //! Container for data stored in the Portable Anymap (PNM) format
    /*!
     * This container stores data for all possible data types:
     *   * Portable Bitmap (PBM)
     *   * Portable Graymap (PGM) with 8-bit depth
     *   * Portable Graymap (PGM) with 16-bit depth
     *   * Portable Pixmap (PPM) with 8-bit depth
     *   * Portable Pixmap (PPM) with 16-bit depth
     *
     * The data type is specified by \ref type(). The class provides functionality to convert from one type to another.
     *
     * Coordinate system
     * -----------------
     * The data is stored as a two-dimensional array where the first index specifies the row and the second index specifies the column.
     * The origin lies in the top-left corner of the image. This corresponds to following coordinate sytem:
     * \code
     *        top
     *     O-------o---->Y (second index)
     *     |       |
     * left|       |right
     *     |       |
     *     o-------o
     *     | bottom
     *     |
     *     v
     *     X (first index)
     * \endcode
     * Note that the **width** of the image is given by the **column** count (dimension of second index), while the **height** of the
     * image is given by the **row** count (dimension of first index).
     */
    class PNMFileData {
        // Type definitions
        // ----------------
    public:
        //! Specification of a tuple of samples
        template <typename SampleType, unsigned int TupleSize = 3>
        using Tuple = std::array<SampleType, TupleSize>;

        //! Specification of a two-dimensional raster
        template <typename SampleType>
        using Raster = memory::MultiVector<SampleType, 2, std::allocator<SampleType>>;

        // Portable Bitmap (PBM)
        //! Specification of sample type of a Portable Bitmap (PBM)
        /*!
         * \note We use a `uint8_t` as representation for a bit, since `std::vector<bool>` (`MultiVector` uses `std::vector`)
         * represents a special case which would make the interface problematic. Note that this leads to 8x memory consumption.
         */
        enum class PBMSample : uint8_t {
            WHITE = 0, //!< White is defined as "0" in a Portable Bitmap (PBM)
            BLACK = 1 //!< Black is defined as "1" in a Portable Bitmap (PBM)
        };
        using PBMCell = PBMSample; //!< Specification of cell type of a Portable Bitmap (PBM)
        using PBM = Raster<PBMCell>; //!< Specification of raster type of a Portable Bitmap (PBM)

        // Portable Graymap (PGM) with 8-bit depth
        using PGM8Sample = uint8_t; //!< Specification of sample type of a Portable Graymap (PGM) with 8-bit depth
        using PGM8Cell = PGM8Sample; //!< Specification of cell type of a Portable Graymap (PGM) with 8-bit depth
        using PGM8 = Raster<PGM8Cell>; //!< Specification of raster type of a Portable Graymap (PGM) with 8-bit depth

        // Portable Graymap (PGM) with 16-bit depth
        using PGM16Sample = uint16_t; //!< Specification of sample type of a Portable Graymap (PGM) with 16-bit depth
        using PGM16Cell = PGM16Sample; //!< Specification of cell type of a Portable Graymap (PGM) with 16-bit depth
        using PGM16 = Raster<PGM16Cell>; //!< Specification of raster type of a Portable Graymap (PGM) with 16-bit depth

        // Portable Pixmap (PPM) with 8-bit depth
        using PPM8Sample = uint8_t; //!< Specification of sample type of a Portable Pixmap (PPM) with 8-bit depth
        using PPM8Cell = Tuple<PPM8Sample>; //!< Specification of cell type of a Portable Pixmap (PPM) with 8-bit depth
        using PPM8 = Raster<PPM8Cell>; //!< Specification of data type of a Portable Pixmap (PPM) with 8-bit depth

        // Portable Pixmap (PPM) with 16-bit depth
        using PPM16Sample = uint16_t; //!< Specification of sample type of a Portable Pixmap (PPM) with 16-bit depth
        using PPM16Cell = Tuple<PPM16Sample>; //!< Specification of cell type of a Portable Pixmap (PPM) with 16-bit depth
        using PPM16 = Raster<PPM16Cell>; //!< Specification of raster type of a Portable Pixmap (PPM) with 16-bit depth

        //! Specification of data types
        enum class Type : uint8_t {
            UNKNOWN = 0, //!< Unknown type (this should **never** happen)
            PBM, //!< Portable Bitmap (PBM)
            PGM_8BIT, //!< Portable Graymap (PGM) with 8-bit depth
            PGM_16BIT, //!< Portable Graymap (PGM) with 16-bit depth
            PPM_8BIT, //!< Portable Pixmap (PPM) with 8-bit depth
            PPM_16BIT, //!< Portable Pixmap (PPM) with 16-bit depth
            TYPE_COUNT //!< Total count of elements
        };

        //! Returns the string representation of the given type
        static inline std::string toString(const Type& type)
        {
            // Check result
            switch (type) {
            case Type::UNKNOWN:
                return "UNKNOWN";
            case Type::PBM:
                return "PBM";
            case Type::PGM_8BIT:
                return "PGM_8BIT";
            case Type::PGM_16BIT:
                return "PGM_16BIT";
            case Type::PPM_8BIT:
                return "PPM_8BIT";
            case Type::PPM_16BIT:
                return "PPM_16BIT";
            default:
                break;
            }

            // Unknown selection
            assert(false);
            return "UNKNOWN";
        }

        //! Returns the file extension (with dot) of the given type
        static inline std::string fileExtension(const Type& type)
        {
            if (type == Type::PBM)
                return ".pbm";
            else if (type == Type::PGM_8BIT || type == Type::PGM_16BIT)
                return ".pgm";
            else if (type == Type::PPM_8BIT || type == Type::PPM_16BIT)
                return ".ppm";
            else
                return ".UNKNOWN";
        }

        // Construction
        // ------------
    public:
        //! Default constructor
        explicit PNMFileData(const Type& type = Type::UNKNOWN)
            : m_type(type)
        {
        }

        //! Typed constructor (with size) (rows=height, columns=width)
        PNMFileData(const Type& type, const size_t& rows, const size_t& columns)
            : PNMFileData(type)
        {
            if (type == Type::PBM)
                m_dataPBM.resize(rows, columns);
            else if (type == Type::PGM_8BIT)
                m_dataPGM8.resize(rows, columns);
            else if (type == Type::PGM_16BIT)
                m_dataPGM16.resize(rows, columns);
            else if (type == Type::PPM_8BIT)
                m_dataPPM8.resize(rows, columns);
            else if (type == Type::PPM_16BIT)
                m_dataPPM16.resize(rows, columns);
            else
                assert(false);
        }

        // Operators
        // ---------
    public:
        //! Comparison operator: **equality**
        inline bool operator==(const PNMFileData& reference) const
        {
            // Compare members
            if (m_type != reference.m_type)
                return false;
            if (m_dataPBM != reference.m_dataPBM)
                return false;
            if (m_dataPGM8 != reference.m_dataPGM8)
                return false;
            if (m_dataPGM16 != reference.m_dataPGM16)
                return false;
            if (m_dataPPM8 != reference.m_dataPPM8)
                return false;
            if (m_dataPPM16 != reference.m_dataPPM16)
                return false;

            // All members are equal -> equal
            return true;
        }

        //! Comparison operator: **inequality**
        inline bool operator!=(const PNMFileData& reference) const { return !(*this == reference); }

        // Members
        // -------
    protected:
        Type m_type = Type::UNKNOWN; //!< \copybrief type()
        PBM m_dataPBM; //!< \copybrief dataPBM()
        PGM8 m_dataPGM8; //!< \copybrief dataPGM8()
        PGM16 m_dataPGM16; //!< \copybrief dataPGM16()
        PPM8 m_dataPPM8; //!< \copybrief dataPPM8()
        PPM16 m_dataPPM16; //!< \copybrief dataPPM16()

        // Getters
        // -------
    public:
        //! Data type of this container
        inline const Type& type() const { return m_type; }

        //! Returns a string representation of the **this** type
        inline std::string typeString() const { return toString(m_type); }

        //! Raw data as Portable Bitmap (PBM)
        /*! \attention Depending on \ref type(), a conversion with \ref convertTo() has to be performed in advance! */
        inline const PBM& dataPBM() const
        {
            assert(m_type == Type::PBM);
            return m_dataPBM;
        }

        //! \copydoc dataPBM() const
        inline PBM& dataPBM()
        {
            assert(m_type == Type::PBM);
            return m_dataPBM;
        }

        //! Raw data as Portable Graymap (PGM) with 8-bit depth
        /*! \attention Depending on \ref type(), a conversion with \ref convertTo() has to be performed in advance! */
        inline const PGM8& dataPGM8() const
        {
            assert(m_type == Type::PGM_8BIT);
            return m_dataPGM8;
        }

        //! \copydoc dataPGM8() const
        inline PGM8& dataPGM8()
        {
            assert(m_type == Type::PGM_8BIT);
            return m_dataPGM8;
        }

        //! Raw data as Portable Graymap (PGM) with 16-bit depth
        /*! \attention Depending on \ref type(), a conversion with \ref convertTo() has to be performed in advance! */
        inline const PGM16& dataPGM16() const
        {
            assert(m_type == Type::PGM_16BIT);
            return m_dataPGM16;
        }

        //! \copydoc dataPGM16() const
        inline PGM16& dataPGM16()
        {
            assert(m_type == Type::PGM_16BIT);
            return m_dataPGM16;
        }

        //! Raw data as Portable Pixmap (PPM) with 8-bit depth
        /*! \attention Depending on \ref type(), a conversion with \ref convertTo() has to be performed in advance! */
        inline const PPM8& dataPPM8() const
        {
            assert(m_type == Type::PPM_8BIT);
            return m_dataPPM8;
        }

        //! \copydoc dataPPM8() const
        inline PPM8& dataPPM8()
        {
            assert(m_type == Type::PPM_8BIT);
            return m_dataPPM8;
        }

        //! Raw data as Portable Pixmap (PPM) with 16-bit depth
        /*! \attention Depending on \ref type(), a conversion with \ref convertTo() has to be performed in advance! */
        inline const PPM16& dataPPM16() const
        {
            assert(m_type == Type::PPM_16BIT);
            return m_dataPPM16;
        }

        //! \copydoc dataPPM16() const
        inline PPM16& dataPPM16()
        {
            assert(m_type == Type::PPM_16BIT);
            return m_dataPPM16;
        }

        //! The size of the raster (rows=height, columns=width)
        inline const std::array<size_t, 2>& size() const
        {
            static const std::array<size_t, 2> unknownSize = { 0, 0 };
            if (m_type == Type::PBM)
                return m_dataPBM.size();
            else if (m_type == Type::PGM_8BIT)
                return m_dataPGM8.size();
            else if (m_type == Type::PGM_16BIT)
                return m_dataPGM16.size();
            else if (m_type == Type::PPM_8BIT)
                return m_dataPPM8.size();
            else if (m_type == Type::PPM_16BIT)
                return m_dataPPM16.size();
            else {
                assert(false);
                return unknownSize;
            }
        }

        //! The width of the image (count of columns)
        inline const size_t& width() const { return size()[1]; }

        //! The height of the image (count of rows)
        inline const size_t& height() const { return size()[0]; }

        // Setters
        // -------
    public:
        //! Clears all data buffers
        /*! \param [in] freeMemory If `true` the reserved memory of the buffers is freed (may be useful for large data) */
        inline void clearData(const bool& freeMemory = false)
        {
            m_dataPBM.clear(freeMemory);
            m_dataPGM8.clear(freeMemory);
            m_dataPGM16.clear(freeMemory);
            m_dataPPM8.clear(freeMemory);
            m_dataPPM16.clear(freeMemory);
        }

        //! Clears complete container (clears all data buffers and resets type)
        /*! \param [in] freeMemory If `true` the reserved memory of the buffers is freed (may be useful for large data) */
        inline void clear(const bool& freeMemory = false)
        {
            clearData(freeMemory);
            m_type = Type::UNKNOWN;
        }

        //! Setter for \copybrief type() const
        /*!
         * \warning Clears all data buffers
         *
         * \param [in] type The new desired type
         * \param [in] freeMemory If `true` the reserved memory of the buffers is freed (may be useful for large data)
         */
        inline void setType(const Type& type, const bool& freeMemory = false)
        {
            if (type == m_type)
                return;
            clearData(freeMemory);
            m_type = type;
        }

        //! Setter for \copybrief dataPBM() const
        /*!
         * \param [in] data The new data (input)
         * \param [in] freeMemory If `true` the reserved memory of the remaining buffers is freed (may be useful for large data)
         */
        inline void setData(const PBM& data, const bool& freeMemory = false) { setData(data, m_dataPBM, Type::PBM, freeMemory); }

        //! Setter for \copybrief dataPGM8() const
        /*!
         * \param [in] data The new data (input)
         * \param [in] freeMemory If `true` the reserved memory of the remaining buffers is freed (may be useful for large data)
         */
        inline void setData(const PGM8& data, const bool& freeMemory = false) { setData(data, m_dataPGM8, Type::PGM_8BIT, freeMemory); }

        //! Setter for \copybrief dataPGM16() const
        /*!
         * \param [in] data The new data (input)
         * \param [in] freeMemory If `true` the reserved memory of the remaining buffers is freed (may be useful for large data)
         */
        inline void setData(const PGM16& data, const bool& freeMemory = false) { setData(data, m_dataPGM16, Type::PGM_16BIT, freeMemory); }

        //! Setter for \copybrief dataPPM8() const
        /*!
         * \param [in] data The new data (input)
         * \param [in] freeMemory If `true` the reserved memory of the remaining buffers is freed (may be useful for large data)
         */
        inline void setData(const PPM8& data, const bool& freeMemory = false) { setData(data, m_dataPPM8, Type::PPM_8BIT, freeMemory); }

        //! Setter for \copybrief dataPPM16() const
        /*!
         * \param [in] data The new data (input)
         * \param [in] freeMemory If `true` the reserved memory of the remaining buffers is freed (may be useful for large data)
         */
        inline void setData(const PPM16& data, const bool& freeMemory = false) { setData(data, m_dataPPM16, Type::PPM_16BIT, freeMemory); }

        //! Converts the current data (according to \ref type()) to the specified type
        /*!
         * \param [in] type The new desired type
         * \param [in] freeMemory If `true` the reserved memory of the remaining buffers is freed (may be useful for large data)
         */
        inline void convertTo(const Type& type, const bool& freeMemory = false)
        {
            if (type == Type::PBM) {
                if (m_type == Type::PGM_8BIT)
                    convertRaster(m_dataPGM8, m_dataPBM, freeMemory);
                else if (m_type == Type::PGM_16BIT)
                    convertRaster(m_dataPGM16, m_dataPBM, freeMemory);
                else if (m_type == Type::PPM_8BIT)
                    convertRaster(m_dataPPM8, m_dataPBM, freeMemory);
                else if (m_type == Type::PPM_16BIT)
                    convertRaster(m_dataPPM16, m_dataPBM, freeMemory);
            } else if (type == Type::PGM_8BIT) {
                if (m_type == Type::PBM)
                    convertRaster(m_dataPBM, m_dataPGM8, freeMemory);
                else if (m_type == Type::PGM_16BIT)
                    convertRaster(m_dataPGM16, m_dataPGM8, freeMemory);
                else if (m_type == Type::PPM_8BIT)
                    convertRaster(m_dataPPM8, m_dataPGM8, freeMemory);
                else if (m_type == Type::PPM_16BIT)
                    convertRaster(m_dataPPM16, m_dataPGM8, freeMemory);
            } else if (type == Type::PGM_16BIT) {
                if (m_type == Type::PBM)
                    convertRaster(m_dataPBM, m_dataPGM16, freeMemory);
                else if (m_type == Type::PGM_8BIT)
                    convertRaster(m_dataPGM8, m_dataPGM16, freeMemory);
                else if (m_type == Type::PPM_8BIT)
                    convertRaster(m_dataPPM8, m_dataPGM16, freeMemory);
                else if (m_type == Type::PPM_16BIT)
                    convertRaster(m_dataPPM16, m_dataPGM16, freeMemory);

            } else if (type == Type::PPM_8BIT) {
                if (m_type == Type::PBM)
                    convertRaster(m_dataPBM, m_dataPPM8, freeMemory);
                else if (m_type == Type::PGM_8BIT)
                    convertRaster(m_dataPGM8, m_dataPPM8, freeMemory);
                else if (m_type == Type::PGM_16BIT)
                    convertRaster(m_dataPGM16, m_dataPPM8, freeMemory);
                else if (m_type == Type::PPM_16BIT)
                    convertRaster(m_dataPPM16, m_dataPPM8, freeMemory);
            } else if (type == Type::PPM_16BIT) {
                if (m_type == Type::PBM)
                    convertRaster(m_dataPBM, m_dataPPM16, freeMemory);
                else if (m_type == Type::PGM_8BIT)
                    convertRaster(m_dataPGM8, m_dataPPM16, freeMemory);
                else if (m_type == Type::PGM_16BIT)
                    convertRaster(m_dataPGM16, m_dataPPM16, freeMemory);
                else if (m_type == Type::PPM_8BIT)
                    convertRaster(m_dataPPM8, m_dataPPM16, freeMemory);
                m_type = Type::PPM_16BIT;
            }
            m_type = type;
        }

        // Helpers
        // -------
    protected:
        //! Sets data for a given data structure
        /*!
         * \param [in] inputData The input data structure
         * \param [in] buffer The target buffer (reference to corresponding member)
         * \param [in] type The data type of the input data structure
         * \param [in] freeMemory If `true` the reserved memory of the buffers is freed (may be useful for large data)
         */
        template <typename RasterType>
        inline void setData(const RasterType& inputData, RasterType& buffer, const Type& type, const bool& freeMemory = false)
        {
            clearData(freeMemory);
            m_type = type;
            buffer = inputData;
        }

        // Convert PBMCell to...
        //! Converts the given input cell to the given output cell (tries to preserve as much information as possible)
        static inline void convertCell(const PBMCell& input, PGM8Cell& output) { output = (input == PBMSample::WHITE) ? std::numeric_limits<PGM8Sample>::max() : std::numeric_limits<PGM8Sample>::lowest(); }
        //! Converts the given input cell to the given output cell (tries to preserve as much information as possible)
        static inline void convertCell(const PBMCell& input, PGM16Cell& output) { output = (input == PBMSample::WHITE) ? std::numeric_limits<PGM16Sample>::max() : std::numeric_limits<PGM16Sample>::lowest(); }
        //! Converts the given input cell to the given output cell (tries to preserve as much information as possible)
        static inline void convertCell(const PBMCell& input, PPM8Cell& output) { (input == PBMSample::WHITE) ? output.fill(std::numeric_limits<PPM8Sample>::max()) : output.fill(std::numeric_limits<PPM8Sample>::lowest()); }
        //! Converts the given input cell to the given output cell (tries to preserve as much information as possible)
        static inline void convertCell(const PBMCell& input, PPM16Cell& output) { (input == PBMSample::WHITE) ? output.fill(std::numeric_limits<PPM16Sample>::max()) : output.fill(std::numeric_limits<PPM16Sample>::lowest()); }

        // Convert PGM8Cell to...
        //! Converts the given input cell to the given output cell (tries to preserve as much information as possible)
        static inline void convertCell(const PGM8Cell& input, PBMCell& output) { output = (input <= ((uint64_t)std::numeric_limits<PGM8Sample>::lowest() + (uint64_t)std::numeric_limits<PGM8Sample>::max()) / 2) ? PBMSample::BLACK : PBMSample::WHITE; }
        //! Converts the given input cell to the given output cell (tries to preserve as much information as possible)
        static inline void convertCell(const PGM8Cell& input, PGM16Cell& output) { output = ((PGM16Sample)input) * 257; }
        //! Converts the given input cell to the given output cell (tries to preserve as much information as possible)
        static inline void convertCell(const PGM8Cell& input, PPM8Cell& output) { output.fill(input); }
        //! Converts the given input cell to the given output cell (tries to preserve as much information as possible)
        static inline void convertCell(const PGM8Cell& input, PPM16Cell& output) { output.fill(((PGM16Sample)input) * 257); }

        // Convert PGM16Cell to...
        //! Converts the given input cell to the given output cell (tries to preserve as much information as possible)
        static inline void convertCell(const PGM16Cell& input, PBMCell& output) { output = (input <= ((uint64_t)std::numeric_limits<PGM16Sample>::lowest() + (uint64_t)std::numeric_limits<PGM16Sample>::max()) / 2) ? PBMSample::BLACK : PBMSample::WHITE; }
        //! Converts the given input cell to the given output cell (tries to preserve as much information as possible)
        static inline void convertCell(const PGM16Cell& input, PGM8Cell& output) { output = input / 257; }
        //! Converts the given input cell to the given output cell (tries to preserve as much information as possible)
        static inline void convertCell(const PGM16Cell& input, PPM8Cell& output) { output.fill(input / 257); }
        //! Converts the given input cell to the given output cell (tries to preserve as much information as possible)
        static inline void convertCell(const PGM16Cell& input, PPM16Cell& output) { output.fill(input); }

        // Convert PPM8Cell to...
        //! Converts the given input cell to the given output cell (tries to preserve as much information as possible)
        static inline void convertCell(const PPM8Cell& input, PBMCell& output) { output = (((uint64_t)input[0] + (uint64_t)input[1] + (uint64_t)input[2]) / 3 <= ((uint64_t)std::numeric_limits<PPM8Sample>::lowest() + (uint64_t)std::numeric_limits<PPM8Sample>::max()) / 2) ? PBMSample::BLACK : PBMSample::WHITE; }
        //! Converts the given input cell to the given output cell (tries to preserve as much information as possible)
        static inline void convertCell(const PPM8Cell& input, PGM8Cell& output) { output = ((uint64_t)input[0] + (uint64_t)input[1] + (uint64_t)input[2]) / 3; }
        //! Converts the given input cell to the given output cell (tries to preserve as much information as possible)
        static inline void convertCell(const PPM8Cell& input, PGM16Cell& output) { output = (((uint64_t)input[0] + (uint64_t)input[1] + (uint64_t)input[2]) / 3) * 257; }
        //! Converts the given input cell to the given output cell (tries to preserve as much information as possible)
        static inline void convertCell(const PPM8Cell& input, PPM16Cell& output)
        {
            // Loop unrolled for best performance
            output[0] = ((PPM16Sample)input[0]) * 257;
            output[1] = ((PPM16Sample)input[1]) * 257;
            output[2] = ((PPM16Sample)input[2]) * 257;
        }

        // Convert PPM16Cell to...
        //! Converts the given input cell to the given output cell (tries to preserve as much information as possible)
        static inline void convertCell(const PPM16Cell& input, PBMCell& output) { output = (((uint64_t)input[0] + (uint64_t)input[1] + (uint64_t)input[2]) / 3 <= ((uint64_t)std::numeric_limits<PPM16Sample>::lowest() + (uint64_t)std::numeric_limits<PPM16Sample>::max()) / 2) ? PBMSample::BLACK : PBMSample::WHITE; }
        //! Converts the given input cell to the given output cell (tries to preserve as much information as possible)
        static inline void convertCell(const PPM16Cell& input, PGM8Cell& output) { output = (((uint64_t)input[0] + (uint64_t)input[1] + (uint64_t)input[2]) / 3) / 257; }
        //! Converts the given input cell to the given output cell (tries to preserve as much information as possible)
        static inline void convertCell(const PPM16Cell& input, PGM16Cell& output) { output = ((uint64_t)input[0] + (uint64_t)input[1] + (uint64_t)input[2]) / 3; }
        //! Converts the given input cell to the given output cell (tries to preserve as much information as possible)
        static inline void convertCell(const PPM16Cell& input, PPM8Cell& output)
        {
            // Loop unrolled for best performance
            output[0] = input[0] / 257;
            output[1] = input[1] / 257;
            output[2] = input[2] / 257;
        }

        //! Converts the given input raster to the given output raster (tries to preserve as much information as possible)
        /*!
         * \warning The input raster will be cleared within this operation!
         *
         * \param [in] input The input raster (will be cleared!)
         * \param [out] output The output raster
         * \param [in] freeMemory If `true` the reserved memory of the input buffer is freed (may be useful for large data)
         */
        template <typename InputRaster, typename OutputRaster>
        static inline void convertRaster(InputRaster& input, OutputRaster& output, const bool& freeMemory = false)
        {
            output.resize(input.size());
            for (size_t i = 0; i < input.elementCount(); i++)
                convertCell(input[i], output[i]);
            input.clear(freeMemory);
        }

        // Input and output
        // ----------------
    protected:
        //! Returns maximum count of ASCII characters per sample (for the current data type)
        inline size_t maximumASCIICharactersPerSample() const
        {
            if (m_type == PNMFileData::Type::PBM)
                return 1; // 0...1
            else if (m_type == PNMFileData::Type::PGM_8BIT || m_type == PNMFileData::Type::PPM_8BIT)
                return 3; // 0...255
            else
                return 5; // 0...65535
        }

        //! Determines the count of digits in an unsigned integer number
        static inline size_t getCountOfDigits(const uint64_t& number)
        {
            size_t digits = 1;
            uint64_t remainder = number;
            while (remainder > 9) {
                remainder /= 10;
                digits++;
            }
            return digits;
        }

        //! Finds the next unsigned integer in the given ASCII stream and returns its start- and end-index (automatically skips comments)
        /*!
         * \param [in] stream The stream in which the search should take place
         * \param [in] startSearchFrom Position in stream from which the search should start (must **not** be within an "open" comment!)
         * \return Pair of indices denoting the start- and end-index of the next unsigned integer in the stream, or size of the stream in case of an error
         */
        static inline std::array<encoding::CharacterStreamSize, 2> findNextUnsignedInteger(const encoding::CharacterStream& stream, const encoding::CharacterStreamSize& startSearchFrom)
        {
            const encoding::CharacterStreamSize notSet = stream.size();
            std::array<encoding::CharacterStreamSize, 2> returnValue = { notSet, stream.size() - 1 };
            bool inComment = false;
            for (size_t i = startSearchFrom; i < stream.size(); i++) {
                if (inComment == true) {
                    if (stream[i] == (char)'\r' || stream[i] == (char)'\n')
                        inComment = false;
                } else {
                    if (stream[i] >= (char)'0' && stream[i] <= (char)'9') {
                        if (returnValue[0] == notSet)
                            returnValue[0] = i;
                    } else {
                        if (returnValue[0] != notSet) {
                            returnValue[1] = i - 1;
                            break;
                        } else if (stream[i] == (char)'#')
                            inComment = true;
                    }
                }
            }
            return returnValue;
        }

        //! Extracts the next unsigned integer in the given ASCII stream (automatically skips comments)
        /*!
         * \param [in] stream The stream in which the search should take place
         * \param [in] startSearchFrom Position in stream from which the search should start (must **not** be within an "open" comment!)
         * \param [out] value Reference to unsigned integer to be set
         * \return Index of last character in stream, or stream size in case of an error
         */
        template <typename UnsignedIntegerType>
        static inline encoding::CharacterStreamSize extractNextUnsignedInteger(const encoding::CharacterStream& stream, const encoding::CharacterStreamSize& startSearchFrom, UnsignedIntegerType& value)
        {
            const auto position = findNextUnsignedInteger(stream, startSearchFrom);
            if (position[0] >= stream.size())
                return stream.size();
            encoding::decode(stream, position[0], position[1] - position[0] + 1, value);
            return position[1];
        }

        //! Encodes the data buffer to the given stream (ascii or binary)
        /*!
         * \param [out] stream Stream to append the encoded data buffer to
         * \param [in] encoding The encoding to be used
         */
        void encodeBuffer(encoding::CharacterStream& stream, const PNMFileEncoding::Type& encoding) const
        {
            // Initialize helpers
            const std::array<size_t, 2>& currentSize = size();
            const size_t& rows = currentSize[0];
            const size_t& columns = currentSize[1];
            const size_t cellCount = rows * columns;
            const size_t charactersPerSample = maximumASCIICharactersPerSample();
            static constexpr size_t maximumCharactersPerLine = 70;
            const size_t lastNewSamplePositionInLine = maximumCharactersPerLine - charactersPerSample;

            // Encode PBM
            // ----------
            if (m_type == PNMFileData::Type::PBM) {
                if (encoding == PNMFileEncoding::Type::ASCII) {
                    size_t charactersPerLine = 0;
                    for (size_t i = 0; i < cellCount; i++) {
                        if (i > 0) {
                            if (charactersPerLine >= lastNewSamplePositionInLine) {
                                encoding::encode(stream, '\n');
                                charactersPerLine = 0;
                            } else {
                                encoding::encode(stream, ' ');
                                charactersPerLine++;
                            }
                        }
                        charactersPerLine += encoding::encode(stream, static_cast<uint8_t>(m_dataPBM[i]));
                    }
                    encoding::encode(stream, '\n');
                } else {
                    size_t bytesPerRow = columns / 8;
                    if (columns % 8 > 0)
                        bytesPerRow++;
                    for (size_t i = 0; i < rows; i++) {
                        for (size_t byte = 0; byte < bytesPerRow; byte++) {
                            const size_t currentByteStartColumn = byte * 8;
                            uint8_t currentByteValue = 0;
                            for (size_t bit = 0; bit < 8; bit++) {
                                const size_t j = currentByteStartColumn + bit;
                                if (j < columns)
                                    currentByteValue |= static_cast<uint8_t>(m_dataPBM(i, j)) << (7 - bit);
                                else
                                    break;
                            }
                            stream.push_back(currentByteValue);
                        }
                    }
                }
            }
            // Encode PGM
            // ----------
            else if (m_type == PNMFileData::Type::PGM_8BIT || m_type == PNMFileData::Type::PGM_16BIT) {
                if (encoding == PNMFileEncoding::Type::ASCII) {
                    size_t charactersPerLine = 0;
                    for (size_t i = 0; i < cellCount; i++) {
                        if (i > 0) {
                            if (charactersPerLine >= lastNewSamplePositionInLine) {
                                encoding::encode(stream, '\n');
                                charactersPerLine = 0;
                            } else {
                                encoding::encode(stream, ' ');
                                charactersPerLine++;
                            }
                        }
                        if (m_type == PNMFileData::Type::PGM_8BIT)
                            charactersPerLine += encoding::encode(stream, m_dataPGM8[i]);
                        else
                            charactersPerLine += encoding::encode(stream, m_dataPGM16[i]);
                    }
                    encoding::encode(stream, '\n');
                } else {
                    if (m_type == PNMFileData::Type::PGM_8BIT) {
                        for (size_t i = 0; i < cellCount; i++)
                            serialization::serialize(stream, serialization::Endianness::BIG, m_dataPGM8[i]);
                    } else {
                        for (size_t i = 0; i < cellCount; i++)
                            serialization::serialize(stream, serialization::Endianness::BIG, m_dataPGM16[i]);
                    }
                }
            }
            // Encode PPM
            // ----------
            else if (m_type == PNMFileData::Type::PPM_8BIT || m_type == PNMFileData::Type::PPM_16BIT) {
                if (encoding == PNMFileEncoding::Type::ASCII) {
                    size_t charactersPerLine = 0;
                    for (size_t i = 0; i < cellCount; i++) {
                        for (size_t c = 0; c < 3; c++) {
                            if (i > 0 || c > 0) {
                                if (charactersPerLine >= lastNewSamplePositionInLine) {
                                    encoding::encode(stream, '\n');
                                    charactersPerLine = 0;
                                } else {
                                    encoding::encode(stream, ' ');
                                    charactersPerLine++;
                                }
                            }
                            if (m_type == PNMFileData::Type::PPM_8BIT)
                                charactersPerLine += encoding::encode(stream, m_dataPPM8[i][c]);
                            else
                                charactersPerLine += encoding::encode(stream, m_dataPPM16[i][c]);
                        }
                    }
                    encoding::encode(stream, '\n');
                } else {
                    if (m_type == PNMFileData::Type::PPM_8BIT) {
                        for (size_t i = 0; i < cellCount; i++)
                            serialization::serialize(stream, serialization::Endianness::BIG, m_dataPPM8[i]);
                    } else {
                        for (size_t i = 0; i < cellCount; i++)
                            serialization::serialize(stream, serialization::Endianness::BIG, m_dataPPM16[i]);
                    }
                }
            }
        }

        //! Decodes the data buffer from the given stream (ascii or binary)
        /*!
         * \param [in] stream Stream containing the buffer (ascii or binary)
         * \param [in] startIndex Index of first byte of buffer in stream
         * \param [in] encoding Encoding of buffer (ascii or binary)
         * \return `true` on success, `false` otherwise
         */
        bool decodeBuffer(const encoding::CharacterStream& stream, const encoding::CharacterStreamSize& startIndex, const PNMFileEncoding::Type& encoding)
        {
            // Initialize helpers
            const std::array<size_t, 2>& currentSize = size();
            const size_t& rows = currentSize[0];
            const size_t& columns = currentSize[1];
            const size_t cellCount = rows * columns;
            encoding::CharacterStreamSize index = startIndex;

            // Check, if there is any data to decode
            if (cellCount == 0)
                return true; // Nothing to decode

            // Decode PBM
            // ----------
            if (m_type == PNMFileData::Type::PBM) {
                if (encoding == PNMFileEncoding::Type::ASCII) {
                    for (size_t i = 0; i < cellCount; i++) {
                        uint64_t value = 0;
                        index = extractNextUnsignedInteger(stream, index, value);
                        if (index >= stream.size())
                            return false;
                        m_dataPBM[i] = (value > 0) ? PBMSample::BLACK : PBMSample::WHITE;
                        index += 1;
                    }
                } else {
                    size_t bytesPerRow = columns / 8;
                    if (columns % 8 > 0)
                        bytesPerRow++;
                    for (size_t i = 0; i < rows; i++) {
                        for (size_t byte = 0; byte < bytesPerRow; byte++) {
                            if (index >= stream.size())
                                return false;
                            const uint8_t& value = stream[index];
                            const size_t byteStartColumn = 8 * byte;
                            for (size_t bit = 0; bit < 8; bit++) {
                                const size_t j = byteStartColumn + bit;
                                if (j < columns)
                                    m_dataPBM(i, j) = ((value & (1 << (7 - bit))) > 0) ? PBMSample::BLACK : PBMSample::WHITE;
                                else
                                    break;
                            }
                            index += 1;
                        }
                    }
                }
            }
            // Decode PGM
            // ----------
            else if (m_type == PNMFileData::Type::PGM_8BIT || m_type == PNMFileData::Type::PGM_16BIT) {
                if (encoding == PNMFileEncoding::Type::ASCII) {
                    for (size_t i = 0; i < cellCount; i++) {
                        uint64_t value = 0;
                        index = extractNextUnsignedInteger(stream, index, value);
                        if (index >= stream.size())
                            return false;
                        if (m_type == PNMFileData::Type::PGM_8BIT)
                            m_dataPGM8[i] = value;
                        else
                            m_dataPGM16[i] = value;
                        index += 1;
                    }
                } else {
                    if (m_type == PNMFileData::Type::PGM_8BIT) {
                        for (size_t i = 0; i < cellCount; i++) {
                            if (index + sizeof(PGM8Sample) > stream.size())
                                return false;
                            const serialization::BinaryStreamSize bytes = serialization::deSerialize(stream, index, serialization::Endianness::BIG, m_dataPGM8[i]);
                            if (bytes == 0)
                                return false;
                            index += bytes;
                        }
                    } else {
                        for (size_t i = 0; i < cellCount; i++) {
                            if (index + sizeof(PGM16Sample) > stream.size())
                                return false;
                            const serialization::BinaryStreamSize bytes = serialization::deSerialize(stream, index, serialization::Endianness::BIG, m_dataPGM16[i]);
                            if (bytes == 0)
                                return false;
                            index += bytes;
                        }
                    }
                }
            }
            // Decode PPM
            // ----------
            else if (m_type == PNMFileData::Type::PPM_8BIT || m_type == PNMFileData::Type::PPM_16BIT) {
                if (encoding == PNMFileEncoding::Type::ASCII) {
                    for (size_t i = 0; i < cellCount; i++) {
                        for (size_t c = 0; c < 3; c++) {
                            uint64_t value = 0;
                            index = extractNextUnsignedInteger(stream, index, value);
                            if (index >= stream.size())
                                return false;
                            if (m_type == PNMFileData::Type::PPM_8BIT)
                                m_dataPPM8[i][c] = value;
                            else
                                m_dataPPM16[i][c] = value;
                            index += 1;
                        }
                    }
                } else {
                    if (m_type == PNMFileData::Type::PPM_8BIT) {
                        for (size_t i = 0; i < cellCount; i++) {
                            if (index + 3 * sizeof(PPM8Sample) > stream.size())
                                return false;
                            const serialization::BinaryStreamSize bytes = serialization::deSerialize(stream, index, serialization::Endianness::BIG, m_dataPPM8[i]);
                            if (bytes == 0)
                                return false;
                            index += bytes;
                        }
                    } else {
                        for (size_t i = 0; i < cellCount; i++) {
                            if (index + 3 * sizeof(PPM16Sample) > stream.size())
                                return false;
                            const serialization::BinaryStreamSize bytes = serialization::deSerialize(stream, index, serialization::Endianness::BIG, m_dataPPM16[i]);
                            if (bytes == 0)
                                return false;
                            index += bytes;
                        }
                    }
                }
            }

            // Success
            return true;
        }

        // Helpers
        // -------
    protected:
        //! Computes (upper bound for) occupied stream size for encoded file (used to pre-allocate memory)
        /*!
         * \note For ascii format an upper bound is computed. For the binary format the returned value is **exact**.
         *
         * \param [in] encoding The encoding to be used
         * \return Upper bound for (ascii) or exact (binary) stream size
         */
        encoding::CharacterStreamSize streamSizeUpperBoundBuffer(const PNMFileEncoding::Type& encoding) const
        {
            // Initialize helpers
            encoding::CharacterStreamSize returnValue = 0;
            const std::array<size_t, 2>& currentSize = size();
            const size_t& rows = currentSize[0];
            const size_t& columns = currentSize[1];
            const size_t cellCount = rows * columns;
            const size_t charactersPerSample = maximumASCIICharactersPerSample();
            const size_t samplesPerCell = (m_type == PNMFileData::Type::PPM_8BIT || m_type == PNMFileData::Type::PPM_16BIT) ? 3 : 1;

            // Check type of encoding
            if (encoding == PNMFileEncoding::Type::ASCII) {
                returnValue += cellCount * samplesPerCell * (charactersPerSample + 1 /* <-- whitespace/line break */);
                if (cellCount == 0)
                    returnValue += 1; // Single line-break
            } else {
                if (m_type == PNMFileData::Type::PBM) {
                    size_t bytesPerRow = columns / 8;
                    if (columns % 8 > 0)
                        bytesPerRow++;
                    returnValue += bytesPerRow * rows;
                } else if (m_type == PNMFileData::Type::PGM_8BIT)
                    returnValue += cellCount;
                else if (m_type == PNMFileData::Type::PGM_16BIT)
                    returnValue += 2 * cellCount;
                else if (m_type == PNMFileData::Type::PPM_8BIT)
                    returnValue += cellCount * samplesPerCell;
                else if (m_type == PNMFileData::Type::PPM_16BIT)
                    returnValue += 2 * cellCount * samplesPerCell;
            }

            // Pass back estimated size
            return returnValue;
        }
    };

    //! \}
} // namespace io
} // namespace broccoli
