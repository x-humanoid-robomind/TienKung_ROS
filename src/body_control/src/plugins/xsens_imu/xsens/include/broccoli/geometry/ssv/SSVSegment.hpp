/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../../memory/SmartVector.hpp"
#include "SSVLineElement.hpp"
#include "SSVPointElement.hpp"
#include "SSVTriangleElement.hpp"
#include <Eigen/StdVector>
#include <string>

namespace broccoli {
namespace geometry {
    //! Representation of a SSV **segment**
    /*!
     * \ingroup broccoli_geometry_ssv
     * In general a SSV segment represents a rigid body. A SSV segment resembles multiple SSV point/line/triangle elements.
     * All these elements are defined in the local coordinate system of the segment. The segment itself maintains a global "pose" (position and orientation)
     * which relates the local segment coordinate system to the "global" coordinate system of the corresponding SSV scene.
     *
     * \note
     * Original implementation by Poscher, Reinhold, "Effiziente Abstandsberechnungen mit Swept-Sphere Volumen für Echtzeit Kollisionsvermeidung in der Robotik", Technical University of Munich, 2020, Bachelor's thesis, https://mediatum.ub.tum.de/1580089
     */
    class SSVSegment : public io::serialization::SerializableData {
        // Type definitions
        // ----------------
    public:
        using ID = uint32_t; //!< Specifies the type of the ID
        using PointElementList = memory::SmartVector<SSVPointElement, 10, Eigen::aligned_allocator<SSVPointElement>>; //!< Specifies a list of point elements
        using LineElementList = memory::SmartVector<SSVLineElement, 10, Eigen::aligned_allocator<SSVLineElement>>; //!< Specifies a list of line elements
        using TriangleElementList = memory::SmartVector<SSVTriangleElement, 10, Eigen::aligned_allocator<SSVTriangleElement>>; //!< Specifies a list of triangle elements

        // Construction
        // ------------
    public:
        //! Default constructor for an **empty** segment (no elements)
        SSVSegment() = default;

        //! Constructor for an **empty** segment (no elements)
        /*!
         * \param [in] id Initializes \ref id() - \copybrief id()
         * \param [in] name Initializes \ref name() - \copybrief name()
         */
        SSVSegment(const ID& id, const std::string& name = "")
            : m_id(id)
            , m_name(name)
        {
        }

        //! Constructor for a **filled** segment
        /*!
         * \param [in] id Initializes \ref id() - \copybrief id()
         * \param [in] name Initializes \ref name() - \copybrief name()
         * \param [in] pointElementCount Reserves memory for the given count of point elements
         * \param [in] lineElementCount Reserves memory for the given count of line elements
         * \param [in] triangleElementCount Reserves memory for the given count of triangle elements
         */
        SSVSegment(const ID& id, const std::string& name, const size_t& pointElementCount, const size_t& lineElementCount, const size_t& triangleElementCount)
            : SSVSegment(id, name)
        {
            reserveElements(pointElementCount, lineElementCount, triangleElementCount);
        }

        //! Virtual destructor
        virtual ~SSVSegment()
        {
        }

        // Members
        // -------
    protected:
        ID m_id = 0; //!< \copybrief id()
        std::string m_name = ""; //!< \copybrief name()
        Eigen::Vector3d m_position = Eigen::Vector3d::Zero(); //!< \copybrief position()
        Eigen::Matrix3d m_orientation = Eigen::Matrix3d::Identity(); //!< \copybrief orientation()
        bool m_updateRequired = false; //!< \copybrief updateRequired()
        // SSV element lists in local segment coordinate system
        PointElementList m_localPointElements; //!< \copybrief localPointElements()
        LineElementList m_localLineElements; //!< \copybrief localLineElements()
        TriangleElementList m_localTriangleElements; //!< \copybrief localTriangleElements()
        // SSV element lists in global scene coordinate system
        PointElementList m_globalPointElements; //!< \copybrief globalPointElements()
        LineElementList m_globalLineElements; //!< \copybrief globalLineElements()
        TriangleElementList m_globalTriangleElements; //!< \copybrief globalTriangleElements()
        // Bounding box in global scene coordinate system
        Eigen::Vector3d m_globalBoundingBoxMinimum = Eigen::Vector3d::Zero(); //!< \copybrief globalBoundingBoxMinimum()
        Eigen::Vector3d m_globalBoundingBoxMaximum = Eigen::Vector3d::Zero(); //!< \copybrief globalBoundingBoxMaximum()

        // Getters
        // -------
    public:
        //! Unique ID to identify the segment
        inline const ID& id() const { return m_id; }

        //! Name of the segment
        inline const std::string& name() const { return m_name; }

        //! The position of the segment in the scene
        inline const Eigen::Vector3d& position() const { return m_position; }

        //! The orientation of the segment in the scene (rotation matrix from segment COSY to scene COSY)
        inline const Eigen::Matrix3d& orientation() const { return m_orientation; }

        //! Flag to indicate if an update of the global representation (in scene coordinates) is required
        inline const bool& updateRequired() const { return m_updateRequired; }

        //! List of point elements in the **local** (segment) coordinate system
        inline const PointElementList& localPointElements() const { return m_localPointElements; }

        //! List of line elements in the **local** (segment) coordinate system
        inline const LineElementList& localLineElements() const { return m_localLineElements; }

        //! List of triangle elements in the **local** (segment) coordinate system
        inline const TriangleElementList& localTriangleElements() const { return m_localTriangleElements; }

        //! List of point elements in the **global** (scene) coordinate system
        inline const PointElementList& globalPointElements() const
        {
            assert(!m_updateRequired);
            return m_globalPointElements;
        }

        //! List of line elements in the **global** (scene) coordinate system
        inline const LineElementList& globalLineElements() const
        {
            assert(!m_updateRequired);
            return m_globalLineElements;
        }

        //! List of triangle elements in the **global** (scene) coordinate system
        inline const TriangleElementList& globalTriangleElements() const
        {
            assert(!m_updateRequired);
            return m_globalTriangleElements;
        }

        //! Bounding box *minimum* in the **global** (scene) coordinate system
        inline const Eigen::Vector3d& globalBoundingBoxMinimum() const
        {
            assert(!m_updateRequired);
            return m_globalBoundingBoxMinimum;
        }

        //! Bounding box *maximum* in the **global** (scene) coordinate system
        inline const Eigen::Vector3d& globalBoundingBoxMaximum() const
        {
            assert(!m_updateRequired);
            return m_globalBoundingBoxMaximum;
        }

        // Setters
        // -------
    public:
        //! Setter for \copybrief id()
        inline void setID(const ID& id) { m_id = id; }

        //! Setter for \copybrief name()
        inline void setName(const std::string& name) { m_name = name; }

        //! Setter for \ref position() - \copybrief position()
        inline void setPosition(const Eigen::Vector3d& position)
        {
            m_position = position;
            m_updateRequired = true;
        }

        //! Setter for \ref orientation() - \copybrief orientation()
        inline void setOrientation(const Eigen::Matrix3d& orientation)
        {
            m_orientation = orientation;
            m_updateRequired = true;
        }

        //! Clears the lists of all elements of this segment (local and global)
        inline void clearElements()
        {
            m_localPointElements.clear();
            m_globalPointElements.clear();
            m_localLineElements.clear();
            m_globalLineElements.clear();
            m_localTriangleElements.clear();
            m_globalTriangleElements.clear();
            m_updateRequired = false;
        }

        //! Reserves memory for the given count of elements (local and global)
        inline void reserveElements(const size_t& pointElementCount, const size_t& lineElementCount, const size_t& triangleElementCount)
        {
            m_localPointElements.reserve(pointElementCount);
            m_globalPointElements.reserve(pointElementCount);
            m_localLineElements.reserve(lineElementCount);
            m_globalLineElements.reserve(lineElementCount);
            m_localTriangleElements.reserve(triangleElementCount);
            m_globalTriangleElements.reserve(triangleElementCount);
        }

        //! Adds the given **locally** defined point element to this segment
        inline void addElement(const SSVPointElement& pointElement)
        {
            m_localPointElements.push_back(pointElement);
            m_updateRequired = true;
        }

        //! Adds the given **locally** defined line element to this segment
        inline void addElement(const SSVLineElement& lineElement)
        {
            m_localLineElements.push_back(lineElement);
            m_updateRequired = true;
        }

        //! Adds the given **locally** defined triangle element to this segment
        inline void addElement(const SSVTriangleElement& triangleElement)
        {
            m_localTriangleElements.push_back(triangleElement);
            m_updateRequired = true;
        }

        //! Applies the given **uniform** scaling to all **locally** defined elements of this segment (operation is performed in **local** segment coordinate system)
        inline void scaleElements(const double& scaling)
        {
            for (size_t i = 0; i < m_localPointElements.size(); i++)
                m_localPointElements[i].scale(scaling);
            for (size_t i = 0; i < m_localLineElements.size(); i++)
                m_localLineElements[i].scale(scaling);
            for (size_t i = 0; i < m_localTriangleElements.size(); i++)
                m_localTriangleElements[i].scale(scaling);
            m_updateRequired = true;
        }

        //! Applies the given arbitrary scaling to all **locally** defined elements of this segment (operation is performed in **local** segment coordinate system)
        inline void scaleElements(const Eigen::Vector3d& scaling)
        {
            for (size_t i = 0; i < m_localPointElements.size(); i++)
                m_localPointElements[i].scale(scaling);
            for (size_t i = 0; i < m_localLineElements.size(); i++)
                m_localLineElements[i].scale(scaling);
            for (size_t i = 0; i < m_localTriangleElements.size(); i++)
                m_localTriangleElements[i].scale(scaling);
            m_updateRequired = true;
        }

        //! Applies the given rotation to all **locally** defined elements of this segment (operation is performed in **local** segment coordinate system)
        inline void rotateElements(const Eigen::Matrix3d& rotation)
        {
            for (size_t i = 0; i < m_localPointElements.size(); i++)
                m_localPointElements[i].rotate(rotation);
            for (size_t i = 0; i < m_localLineElements.size(); i++)
                m_localLineElements[i].rotate(rotation);
            for (size_t i = 0; i < m_localTriangleElements.size(); i++)
                m_localTriangleElements[i].rotate(rotation);
            m_updateRequired = true;
        }

        //! Applies the given translation to all **locally** defined elements of this segment (operation is performed in **local** segment coordinate system)
        inline void translateElements(const Eigen::Vector3d& translation)
        {
            for (size_t i = 0; i < m_localPointElements.size(); i++)
                m_localPointElements[i].translate(translation);
            for (size_t i = 0; i < m_localLineElements.size(); i++)
                m_localLineElements[i].translate(translation);
            for (size_t i = 0; i < m_localTriangleElements.size(); i++)
                m_localTriangleElements[i].translate(translation);
            m_updateRequired = true;
        }

        // Main Interface
        // --------------
    public:
        //! Updates the segment (only if an update is required)
        /*!
         * Re-computes the list of global elements, i.e., copies the local elements and applies the transform.
         * \return `true` if an update was actually made, `false` otherwise (update was not required)
         */
        inline bool update()
        {
            // Skip, if an update is not required
            if (m_updateRequired == false)
                return false;

            // Copy local element lists
            m_globalPointElements = m_localPointElements;
            m_globalLineElements = m_localLineElements;
            m_globalTriangleElements = m_localTriangleElements;

            // Transform elements
            for (size_t i = 0; i < m_globalPointElements.size(); i++) {
                m_globalPointElements[i].rotate(m_orientation);
                m_globalPointElements[i].translate(m_position);
            }
            for (size_t i = 0; i < m_globalLineElements.size(); i++) {
                m_globalLineElements[i].rotate(m_orientation);
                m_globalLineElements[i].translate(m_position);
            }
            for (size_t i = 0; i < m_globalTriangleElements.size(); i++) {
                m_globalTriangleElements[i].rotate(m_orientation);
                m_globalTriangleElements[i].translate(m_position);
            }

            // Re-compute bounding box
            size_t verticesInBoundingBox = 0;
            for (size_t i = 0; i < m_globalPointElements.size(); i++)
                addVertexToGlobalBoundingBox(m_globalPointElements[i].point0(), m_globalPointElements[i].radius(), verticesInBoundingBox);
            for (size_t i = 0; i < m_globalLineElements.size(); i++) {
                addVertexToGlobalBoundingBox(m_globalLineElements[i].point0(), m_globalLineElements[i].radius(), verticesInBoundingBox);
                addVertexToGlobalBoundingBox(m_globalLineElements[i].point1(), m_globalLineElements[i].radius(), verticesInBoundingBox);
            }
            for (size_t i = 0; i < m_globalTriangleElements.size(); i++) {
                addVertexToGlobalBoundingBox(m_globalTriangleElements[i].point0(), m_globalTriangleElements[i].radius(), verticesInBoundingBox);
                addVertexToGlobalBoundingBox(m_globalTriangleElements[i].point1(), m_globalTriangleElements[i].radius(), verticesInBoundingBox);
                addVertexToGlobalBoundingBox(m_globalTriangleElements[i].point2(), m_globalTriangleElements[i].radius(), verticesInBoundingBox);
            }
            if (verticesInBoundingBox == 0) {
                m_globalBoundingBoxMinimum = Eigen::Vector3d::Zero();
                m_globalBoundingBoxMaximum = Eigen::Vector3d::Zero();
            }

            // Reset update flag
            m_updateRequired = false;

            // Update was performed
            return true;
        }

        //! Computes an estimation of the cost of \ref update()
        /*!
         * The estimation if based on a benchmark of translation and rotation of SSV elements (nanoseconds). The benchmark did not cover copying element lists or computing bounding boxes.
         * However, we assume that the costs for these operations behave similar to translation and rotation.
         * \return An estimated cost value for this segment
         */
        inline double estimateUpdateCost() const
        {
            // (hard-coded values obtained from a benchmark)
            return ((double)m_localPointElements.size() * 5.5 + (double)m_localLineElements.size() * 7.5 + (double)m_localTriangleElements.size() * 13.5);
        }

        // Helpers
        // -------
    public:
        //! Checks if the segment is valid
        /*! \return `true`, if the segment is valid, `false` otherwise. */
        inline bool isValid() const
        {
            // Check, if there is at least one element
            if (m_localPointElements.size() == 0 && m_localLineElements.size() == 0 && m_localTriangleElements.size() == 0)
                return false;

            // Check, if all elements are valid
            for (size_t i = 0; i < m_localPointElements.size(); i++) {
                if (m_localPointElements[i].isValid() == false)
                    return false;
            }
            for (size_t i = 0; i < m_localLineElements.size(); i++) {
                if (m_localLineElements[i].isValid() == false)
                    return false;
            }
            for (size_t i = 0; i < m_localTriangleElements.size(); i++) {
                if (m_localTriangleElements[i].isValid() == false)
                    return false;
            }

            // No errors found -> valid
            return true;
        }

    protected:
        //! Updates the global bounding box with the given vertex
        /*!
         * \param [in] vertex The vertex in consideration
         * \param [in] radius The corresponding radius of the vertex
         * \param [in,out] verticesInBoundingBox Total count of vertices considered by the bounding box (gets updated by this function call)
         */
        inline void addVertexToGlobalBoundingBox(const Eigen::Vector3d& vertex, const double& radius, size_t& verticesInBoundingBox)
        {
            // Initialize helpers
            const Eigen::Vector3d radiusVector = Eigen::Vector3d(radius, radius, radius);

            // Check, if this is the first vertex to be considered
            if (verticesInBoundingBox == 0) {
                // ...yes, first vertex -> just use bounding box of vertex as global bounding box
                m_globalBoundingBoxMinimum = vertex - radiusVector;
                m_globalBoundingBoxMaximum = vertex + radiusVector;
            } else {
                // ...no, not first vertex -> update bounding box by bounding box of vertex
                const Eigen::Vector3d vertexBoundingBoxMinimum = vertex - radiusVector;
                const Eigen::Vector3d vertexBoundingBoxMaximum = vertex + radiusVector;
                for (Eigen::Index i = 0; i < 3; i++) {
                    if (vertexBoundingBoxMinimum(i) < m_globalBoundingBoxMinimum(i))
                        m_globalBoundingBoxMinimum(i) = vertexBoundingBoxMinimum(i);
                    if (vertexBoundingBoxMaximum(i) > m_globalBoundingBoxMaximum(i))
                        m_globalBoundingBoxMaximum(i) = vertexBoundingBoxMaximum(i);
                }
            }

            // Update counter
            verticesInBoundingBox++;
        }

        // Serialization
        // -------------
    protected:
        // Serialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize serializePayload(io::serialization::BinaryStream& stream, const io::serialization::Endianness& endianness) const
        {
            return io::serialization::serialize(stream, endianness, m_id, m_name, m_position, m_orientation, m_localPointElements, m_localLineElements, m_localTriangleElements);
        }

        // Deserialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize deSerializePayload(const io::serialization::BinaryStream& stream, const io::serialization::BinaryStreamSize& index, const io::serialization::BinaryStreamSize& payloadSize, const io::serialization::Endianness& endianness)
        {
            (void)payloadSize; // unused
            const io::serialization::BinaryStreamSize returnValue = io::serialization::deSerialize(stream, index, endianness, m_id, m_name, m_position, m_orientation, m_localPointElements, m_localLineElements, m_localTriangleElements);
            m_updateRequired = true;
            return returnValue;
        }

        // Triangularization
        // -----------------
    public:
        //! Creates a triangle mesh representing this segment
        /*!
         * \param [in] stepsPerPi Count of discretizing steps for approximating the angle \f$ \pi \f$, constraint: has to be greater than 1
         * \param [in] global If `true`, the global element poses are used, otherwise the local element poses
         * \return Created mesh (empty mesh in case of an error, e.g. invalid input)
         */
        CGMesh createMesh(const uint64_t& stepsPerPi, const bool& global = false) const
        {
            // Check input
            if (stepsPerPi < 2 || (global == true && m_updateRequired)) {
                assert(false);
                return CGMesh();
            }

            // Get element lists
            const PointElementList& pointElementList = (global == true) ? m_globalPointElements : m_localPointElements;
            const LineElementList& lineElementList = (global == true) ? m_globalLineElements : m_localLineElements;
            const TriangleElementList& triangleElementList = (global == true) ? m_globalTriangleElements : m_localTriangleElements;

            // Get count of elements
            const size_t totalElementCount = pointElementList.size() + lineElementList.size() + triangleElementList.size();

            // Check element count
            if (totalElementCount == 0)
                return CGMesh(); // Empty segment -> empty mesh
            else if (totalElementCount == 1) {
                // Only one element -> no need for merging
                if (pointElementList.size() == 1)
                    return pointElementList[0].createMesh(stepsPerPi);
                else if (lineElementList.size() == 1)
                    return lineElementList[0].createMesh(stepsPerPi);
                else
                    return triangleElementList[0].createMesh(stepsPerPi);
            } else {
                // More than one element -> merge meshes
                std::vector<CGMesh, Eigen::aligned_allocator<CGMesh>> subMeshList;
                std::vector<const CGMesh*> subMeshPointerList;
                subMeshList.reserve(totalElementCount);
                subMeshPointerList.reserve(totalElementCount);
                for (size_t i = 0; i < pointElementList.size(); i++) {
                    subMeshList.emplace_back(pointElementList[i].createMesh(stepsPerPi));
                    subMeshPointerList.push_back(&subMeshList.back());
                }
                for (size_t i = 0; i < lineElementList.size(); i++) {
                    subMeshList.emplace_back(lineElementList[i].createMesh(stepsPerPi));
                    subMeshPointerList.push_back(&subMeshList.back());
                }
                for (size_t i = 0; i < triangleElementList.size(); i++) {
                    subMeshList.emplace_back(triangleElementList[i].createMesh(stepsPerPi));
                    subMeshPointerList.push_back(&subMeshList.back());
                }
                return CGMeshTools::merge(subMeshPointerList);
            }
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };
} // namespace geometry
} // namespace broccoli

#endif // HAVE_EIGEN3
