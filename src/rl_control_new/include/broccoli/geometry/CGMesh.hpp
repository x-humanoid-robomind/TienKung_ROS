/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../core/floats.hpp"
#include "../io/serialization/serialization.hpp"
#include "CGMaterial.hpp"
#include <Eigen/Dense>
#include <array>
#include <vector>

namespace broccoli {
namespace geometry {
    /*!
     * \addtogroup broccoli_geometry
     * \{
     */

    //! Specification of result types of CGMesh algorithms
    enum class CGMeshResult : uint8_t {
        UNKNOWN = 0, //!< Unknown result (this should **never** happen)
        SUCCESS, //!< Algorithm was successful
        ERROR_INVALID_NORMALBUFFER, //!< An **error** occured: The normal buffer is invalid
        ERROR_INVALID_COLORBUFFER, //!< An **error** occured: The color buffer is invalid
        ERROR_INVALID_INDEXBUFFER, //!< An **error** occured: The index buffer is invalid
        ERROR_INVALID_INDEX, //!< An **error** occured: The given index/indices is/are invalid
        CGMESHRESULT_COUNT //!< Total count of elements
    };

    //! Returns the string representation of the given result type for CGMesh algorithms
    static inline std::string CGMeshResultString(const CGMeshResult& result)
    {
        // Check result
        switch (result) {
        case CGMeshResult::UNKNOWN:
            return "UNKNOWN";
        case CGMeshResult::SUCCESS:
            return "SUCCESS";
        case CGMeshResult::ERROR_INVALID_NORMALBUFFER:
            return "ERROR_INVALID_NORMALBUFFER";
        case CGMeshResult::ERROR_INVALID_COLORBUFFER:
            return "ERROR_INVALID_COLORBUFFER";
        case CGMeshResult::ERROR_INVALID_INDEXBUFFER:
            return "ERROR_INVALID_INDEXBUFFER";
        case CGMeshResult::ERROR_INVALID_INDEX:
            return "ERROR_INVALID_INDEX";
        default:
            break;
        }

        // Unknown selection
        assert(false);
        return "UNKNOWN";
    }

    //! Generic mesh used in <b>C</b>omputer <b>G</b>raphics
    /*!
     * The mesh supports indexed vertex buffers for minimal memory consumption and optimal transformation performance.
     * Moreover, various shading models are supported through the color- and material-buffer.
     *
     * Contains buffers for
     *  * vertices (\ref broccoli::geometry::CGMesh::m_vertexBuffer),
     *  * optional: per-vertex-normals (\ref broccoli::geometry::CGMesh::m_normalBuffer),
     *  * optional: per-vertex-colors (\ref broccoli::geometry::CGMesh::m_colorBuffer) and
     *  * optional: per-triangle-materials (\ref broccoli::geometry::CGMesh::m_materialBuffer).
     *
     * The triangles representing the mesh are defined through the index buffer (\ref broccoli::geometry::CGMesh::m_indexBuffer). Each element in the index buffer contains
     *  * the indices of the three vertices (optional: normals/colors) describing the triangle in the vertex (optional: normal/color) buffer and
     *  * optional: the index of the material of the triangle in the material buffer.
     * See \ref broccoli::geometry::CGMesh::m_indexBuffer for more details on the layout.
     */
    class CGMesh : public io::serialization::SerializableData {
    public:
        // Type definitions
        using IndexType = uint32_t; //!< Data type for coefficients in the index buffer
        using ColorChannelType = uint8_t; //!< Data type for coefficients in the color buffer
        using ColorType = Eigen::Matrix<ColorChannelType, 4, 1>; //!< Data type for a single color value in the color buffer

        //! Constructor (empty mesh)
        CGMesh()
        {
        }

        //! Destructor
        virtual ~CGMesh()
        {
        }

        //! Comparison operator: **equality**
        bool operator==(const CGMesh& reference) const
        {
            // Compare members
            if (m_vertexBuffer.rows() != reference.m_vertexBuffer.rows() || m_vertexBuffer.cols() != reference.m_vertexBuffer.cols() || !m_vertexBuffer.isApprox(reference.m_vertexBuffer))
                return false;
            if (m_normalBuffer.rows() != reference.m_normalBuffer.rows() || m_normalBuffer.cols() != reference.m_normalBuffer.cols() || !m_normalBuffer.isApprox(reference.m_normalBuffer))
                return false;
            if (m_colorBuffer.rows() != reference.m_colorBuffer.rows() || m_colorBuffer.cols() != reference.m_colorBuffer.cols() || !m_colorBuffer.isApprox(reference.m_colorBuffer))
                return false;
            if (m_materialBuffer != reference.m_materialBuffer)
                return false;
            if (m_indexBuffer.rows() != reference.m_indexBuffer.rows() || m_indexBuffer.cols() != reference.m_indexBuffer.cols() || !m_indexBuffer.isApprox(reference.m_indexBuffer))
                return false;

            // All members are equal -> equal
            return true;
        }

        //! Comparison operator: **inequality**
        bool operator!=(const CGMesh& reference) const { return !(*this == reference); }

        // Members
        // -------
        //! Buffer of **vertex** positions
        /*!
         * Layout: \f[ \left[\begin{array}{ccc} x_1 & x_2 & \cdots \\ y_1 & y_2 & \cdots \\ z_1 & z_2 & \cdots \end{array}\right] \f]
         */
        Eigen::Matrix<double, 3, Eigen::Dynamic> m_vertexBuffer;

        //! Buffer of **per-vertex normals** (empty if no normal information is available)
        /*!
         * Layout:
         * \f[ \left[\begin{array}{ccc} n_{x,1} & n_{x,2} & \cdots \\ n_{y,1} & n_{y,2} & \cdots \\ n_{z,1} & n_{z,2} & \cdots \end{array}\right] \f]
         *
         * \attention Each normal belongs to a vertex, thus the column count of the buffer has to match the column count of \ref broccoli::geometry::CGMesh::m_vertexBuffer! However, it may be 0 if no normal information is available at all.
         */
        Eigen::Matrix<double, 3, Eigen::Dynamic> m_normalBuffer;

        //! Buffer of **per-vertex colors** (empty if no color information is available)
        /*!
         * Layout: \f[ \left[\begin{array}{ccc} R_1 & R_2 & \cdots \\ G_1 & G_2 & \cdots\\ B_1 & B_2 & \cdots\\ A_1 & A_2 & \cdots \end{array}\right] \f]
         *
         * Colors:
         *  * R (red) = \f$ 0 \dots 255\f$
         *  * G (green) = \f$ 0 \dots 255\f$
         *  * B (blue) = \f$ 0 \dots 255\f$
         *  * A (alpha) = \f$ 0 \dots 255\f$
         *
         * \attention Each color belongs to a vertex, thus the size of the buffer has to match the size of \ref broccoli::geometry::CGMesh::m_vertexBuffer! However, it may be 0 if no color information is available at all.
         */
        Eigen::Matrix<ColorChannelType, 4, Eigen::Dynamic> m_colorBuffer;

        //! Buffer of attached materials of type \ref CGMaterial (empty if no material information is available)
        std::vector<CGMaterial> m_materialBuffer;

        //! Buffer of vertex/normal/color/material **indices**
        /*!
         * Layout: \f[ \left[\begin{array}{ccc} v_{first,1} & v_{first,2} & \cdots \\ v_{second,1} & v_{second,2} & \cdots \\ v_{third,1} & v_{third,2} & \cdots \\ m_{idx,1} & m_{idx,2} & \cdots \end{array}\right] \f]
         *
         * Each column describes one triangle:
         *  * \f$ v_{first,n} \geq 0 \f$ Index of the **first** vertex (and corresponding normal and color) in \ref broccoli::geometry::CGMesh::m_vertexBuffer (and \ref broccoli::geometry::CGMesh::m_normalBuffer and \ref broccoli::geometry::CGMesh::m_colorBuffer) of the \f$n\f$-th triangle
         *  * \f$ v_{second,n} \geq 0 \f$ Index of the **second** vertex (and corresponding normal and color) in \ref broccoli::geometry::CGMesh::m_vertexBuffer (and \ref broccoli::geometry::CGMesh::m_normalBuffer and \ref broccoli::geometry::CGMesh::m_colorBuffer) of the \f$n\f$-th triangle
         *  * \f$ v_{third,n} \geq 0 \f$ Index of the **third** vertex (and corresponding normal and color) in \ref broccoli::geometry::CGMesh::m_vertexBuffer (and \ref broccoli::geometry::CGMesh::m_normalBuffer and \ref broccoli::geometry::CGMesh::m_colorBuffer) of the \f$n\f$-th triangle
         *  * \f$ m_{idx,n} \geq 0 \f$ Optional: Index of the material in \ref broccoli::geometry::CGMesh::m_materialBuffer of the \f$n\f$-th triangle.
         *
         * The fourth row is **optional**. If the \ref broccoli::geometry::CGMesh::m_indexBuffer has only three rows, no materials are linked to the triangles.
         */
        Eigen::Matrix<IndexType, Eigen::Dynamic, Eigen::Dynamic> m_indexBuffer;

        // Generic
        // -------
        //! Checks, if the mesh is properly defined
        /*!
         * \param [out] result Pointer to flag in which the result should be stored
         * \return `true`, if the mesh is valid, `false` otherwise.
         *
         * \warning Checks, if colors, linked materials and indices are valid. Depending on the mesh size / material count this can be very expensive!
         */
        virtual bool isValid(CGMeshResult* const result = nullptr) const
        {
            // Check, if normal buffer is of appropriate size (per-vertex normals)
            if (m_normalBuffer.cols() > 0 && m_normalBuffer.cols() != m_vertexBuffer.cols()) {
                // Normal buffer is available but it does not match the vertex buffer
                if (result != nullptr)
                    *result = CGMeshResult::ERROR_INVALID_NORMALBUFFER;
                return false;
            }

            // Check, if color buffer is of appropriate size (per-vertex colors)
            if (m_colorBuffer.cols() > 0 && m_colorBuffer.cols() != m_vertexBuffer.cols()) {
                // Color buffer is available but it does not match the vertex buffer
                if (result != nullptr)
                    *result = CGMeshResult::ERROR_INVALID_COLORBUFFER;
                return false;
            }

            // Check, if index buffer is of appropriate size
            if (m_indexBuffer.cols() > 0 && m_indexBuffer.rows() != 3 && m_indexBuffer.rows() != 4) {
                if (result != nullptr)
                    *result = CGMeshResult::ERROR_INVALID_INDEXBUFFER;
                return false;
            }

            // Check, if vertex indices in index buffer are valid
            for (Eigen::Index j = 0; j < m_indexBuffer.cols(); j++) {
                for (Eigen::Index i = 0; i < 3; i++) {
                    if (m_indexBuffer(i, j) >= m_vertexBuffer.cols()) {
                        if (result != nullptr)
                            *result = CGMeshResult::ERROR_INVALID_INDEXBUFFER;
                        return false;
                    }
                }
            }

            // Check, if material indices in index buffer are valid
            if (m_indexBuffer.rows() == 4) {
                for (Eigen::Index j = 0; j < m_indexBuffer.cols(); j++) {
                    if (m_indexBuffer(3, j) >= m_materialBuffer.size()) {
                        if (result != nullptr)
                            *result = CGMeshResult::ERROR_INVALID_INDEXBUFFER;
                        return false;
                    }
                }
            }

            // Otherwise -> valid
            if (result != nullptr)
                *result = CGMeshResult::SUCCESS;
            return true;
        }

        // Mesh operations
        // ---------------
        //! Generates a map which reflects the links between the vertex- and the triangle-buffer
        /*!
         * \warning Depending on the mesh size this is an expensive operation!
         *
         * \return Mapping between vertex- and triangle-buffer [index-of-investigated-vertex-in-vertex-buffer][running index] = index-of-connected-triangle-in-index-buffer
         */
        virtual std::vector<std::vector<IndexType>> vertexTriangleMap() const
        {
            // Check validity of mesh first
            assert(isValid());

            // Setup return value
            std::vector<std::vector<IndexType>> map(m_vertexBuffer.cols());

            // Check, if there is something to do
            if (m_vertexBuffer.cols() == 0 || m_indexBuffer.cols() == 0)
                return map; // nothing to do...

            // Pre-allocate memory for second dimension of buffer
            for (size_t i = 0; i < map.size(); i++)
                map[i].reserve(6); // <- typical count of connected triangles for a chessboard-like mesh pattern

            // Iterate through triangles
            for (Eigen::Index j = 0; j < m_indexBuffer.cols(); j++) {
                // Iterate through vertices of triangle
                for (Eigen::Index i = 0; i < 3; i++) {
                    const auto& vertexIndex = m_indexBuffer(i, j);
                    if (vertexIndex < map.size()) {
                        if (map[vertexIndex].size() == 0 || (int)map[vertexIndex].back() != j) {
                            if (map[vertexIndex].capacity() < map[vertexIndex].size() + 1)
                                map[vertexIndex].reserve(2 * map[vertexIndex].capacity()); // Double capacity everytime we hit the limit
                            map[vertexIndex].push_back(j);
                        }
                        // else: triangle is already in map -> do nothing
                    }
                    // else: invalid index -> do nothing
                }
            }

            // Pass back map
            return map;
        }

        //! Generates a map which reflects the links between the material- and the triangle-buffer
        /*!
         * \warning Depending on the mesh size this is an expensive operation!
         *
         * \return Mapping between material- and triangle-buffer [index-of-investigated-material-in-material-buffer][running index] = index-of-connected-triangle-in-index-buffer
         */
        virtual std::vector<std::vector<IndexType>> materialTriangleMap() const
        {
            // Check validity of mesh first
            assert(isValid());

            // Setup return value
            std::vector<std::vector<IndexType>> map(m_materialBuffer.size());

            // Check, if there is something to do
            if (m_materialBuffer.size() == 0 || m_indexBuffer.cols() == 0 || m_indexBuffer.rows() != 4)
                return map; // nothing to do...

            // Pre-allocate memory for second dimension of buffer
            size_t estimatedTrianglesPerMaterial = m_indexBuffer.cols() / m_materialBuffer.size(); // triangle-count / material-count
            if (estimatedTrianglesPerMaterial < 1)
                estimatedTrianglesPerMaterial = 1;
            for (size_t i = 0; i < map.size(); i++)
                map[i].reserve(2 * estimatedTrianglesPerMaterial); // <- use estimate with safety margin

            // Iterate through triangles
            for (Eigen::Index j = 0; j < m_indexBuffer.cols(); j++) {
                const auto& materialIndex = m_indexBuffer(3, j);
                if (materialIndex < map.size()) {
                    if (map[materialIndex].size() == 0 || (int)map[materialIndex].back() != j) {
                        if (map[materialIndex].capacity() < map[materialIndex].size() + 1)
                            map[materialIndex].reserve(2 * map[materialIndex].capacity()); // Double capacity everytime we hit the limit
                        map[materialIndex].push_back(j);
                    }
                    // else: triangle is already in map -> do nothing
                }
                // else: invalid index -> do nothing
            }

            // Pass back map
            return map;
        }

        //! Normalizes the selected normals in the normal buffer
        /*!
         * \param [in] firstNormalIndex Index of the first normal to normalize in the \ref broccoli::geometry::CGMesh::m_normalBuffer
         * \param [in] lastNormalIndex Index of the last normal to normalize in the \ref broccoli::geometry::CGMesh::m_normalBuffer
         * \return `true` on success, `false` otherwise (invalid indices or dimension mismatch)
         */
        virtual bool normalizeNormals(const IndexType& firstNormalIndex, const IndexType& lastNormalIndex)
        {
            // Check validity of mesh first
            assert(isValid());

            // Check, if given indices are valid
            if (firstNormalIndex > lastNormalIndex || (int)lastNormalIndex >= m_normalBuffer.cols()) {
                // The normal buffer does not contain the specified indices
                assert(false);
                return false;
            }

            // Pass through selected normals
            for (IndexType n = firstNormalIndex; n <= lastNormalIndex; n++)
                m_normalBuffer.col(n).normalize();

            // Success
            return true;
        }

        //! Normalizes **all** normals in the normal buffer
        virtual void normalizeNormals()
        {
            // Skip, if buffer is empty
            if (m_normalBuffer.cols() <= 0)
                return;

            // Otherwise: normalize all normals in the buffer
            normalizeNormals(0, m_normalBuffer.cols() - 1);
        }

        //! Re-computes the selected normals in the normal buffer
        /*!
         * The selected per-vertex-normals in the \ref broccoli::geometry::CGMesh::m_normalBuffer are recomputed as the **averaged per-triangle-normals** of all connected triangles.
         *
         * \warning Depending on the mesh size this is an expensive operation!
         *
         * \param [in] firstNormalIndex Index of the first normal to recompute in the \ref broccoli::geometry::CGMesh::m_normalBuffer
         * \param [in] lastNormalIndex Index of the last normal to recompute in the \ref broccoli::geometry::CGMesh::m_normalBuffer
         * \param [out] result Pointer to flag in which the result should be stored
         * \return `true` on success, `false` otherwise (invalid indices or dimension mismatch)
         */
        virtual bool recomputeNormals(const IndexType& firstNormalIndex, const IndexType& lastNormalIndex, CGMeshResult* const result = nullptr)
        {
            // Check validity of mesh first
            assert(isValid());

            // Check, if given indices are valid
            if (firstNormalIndex > lastNormalIndex || (int)lastNormalIndex >= m_normalBuffer.cols()) {
                // The normal buffer does not contain the specified indices
                if (result != nullptr)
                    *result = CGMeshResult::ERROR_INVALID_INDEX;
                assert(false);
                return false;
            }

            // Compute mapping between vertices/normals and triangles
            const auto indexMap = vertexTriangleMap();

            // Generate buffer for per-triangle normals
            Eigen::Matrix<double, 3, Eigen::Dynamic> perTriangleNormals; // Each column is the per-triangle normal related to the corresponding triangle (column in index buffer)
            perTriangleNormals.resize(Eigen::NoChange, m_indexBuffer.cols());
            std::vector<bool> perTriangleNormalComputed;
            perTriangleNormalComputed.resize(perTriangleNormals.cols(), false); // Flag indicating, if the corresponding per-triangle normal has already been computed

            // Iterate through per-vertex-normals to recompute
            for (IndexType n = firstNormalIndex; n <= lastNormalIndex; n++) {
                // Set per-vertex normal to zero (for computing average)
                Eigen::Vector3d perVertexNormal(0, 0, 0);

                // Iterate through all connected triangles
                const IndexType triangleCount = indexMap[n].size(); // Count of triangles connected to the corresponding vertex
                for (IndexType t = 0; t < triangleCount; t++) {
                    // Get index of the connected triangle in the index buffer
                    const IndexType triangleIndex = indexMap[n][t];

                    // Check, if per-triangle normal has already been computed for this triangle
                    if (perTriangleNormalComputed[triangleIndex] == false) {
                        // Compute per-triangle normal
                        const auto& v0 = m_vertexBuffer.col(m_indexBuffer(0, triangleIndex));
                        const auto& v1 = m_vertexBuffer.col(m_indexBuffer(1, triangleIndex));
                        const auto& v2 = m_vertexBuffer.col(m_indexBuffer(2, triangleIndex));
                        perTriangleNormals.col(triangleIndex) = (v1 - v0).cross(v2 - v1).normalized();
                        perTriangleNormalComputed[triangleIndex] = true;
                    }

                    // Use the computed per-triangle normal
                    perVertexNormal += perTriangleNormals.col(triangleIndex);
                }

                // Normalize, if there is more than one contributor
                if (triangleCount > 1)
                    perVertexNormal.normalize();

                // Store in normal buffer
                m_normalBuffer.col(n) = perVertexNormal;
            }

            // Success
            if (result != nullptr)
                *result = CGMeshResult::SUCCESS;
            return true;
        }

        //! Re-computes **all** normals in the normal buffer
        /*!
         * The per-vertex-normals are recomputed as the averaged per-triangle-normals of all connected triangles. Resizes the normal buffer if necessary.
         *
         * \warning Depending on the mesh size this is an expensive operation!
         */
        virtual void recomputeNormals()
        {
            // Check validity of mesh first
            assert(isValid());

            // Resize normal buffer if necessary
            m_normalBuffer.resize(Eigen::NoChange, m_vertexBuffer.cols());

            // Skip, if there are no normals to recompute
            if (m_normalBuffer.cols() == 0)
                return;

            // Otherwise: recompute all normals
            recomputeNormals(0, m_normalBuffer.cols() - 1);
        }

        //! Applies the given translation to the mesh
        /*!
         * Only vertices are subject to translation. Normals remain unchanged.
         *
         * \f[ x_{new} = x_{old} + t\f]
         *
         * \param [in] translation The vector \f$ t \f$ to **add** to the position.
         */
        virtual void translate(const Eigen::Vector3d& translation)
        {
            // Check validity of mesh first
            assert(isValid());

            // Translate vertex buffer
            if (m_vertexBuffer.cols() > 0)
                for (Eigen::Index i = 0; i < m_vertexBuffer.cols(); i++)
                    m_vertexBuffer.col(i) += translation;
        }

        //! Applies the given rotation to the mesh
        /*!
         * Vertices and normals are subject to rotation.
         *
         * \f[ x_{new} = A * x_{old} \quad \mbox{and} \quad n_{new} = A * n_{old} \f]
         *
         * \param [in] rotation Rotation matrix \f$ A \in \mathbb{R}^{3\times 3}\f$ used for transformation
         */
        virtual void rotate(const Eigen::Matrix3d& rotation)
        {
            // Check validity of mesh first
            assert(isValid());

            // Rotate vertex buffer
            if (m_vertexBuffer.cols() > 0)
                m_vertexBuffer = (rotation * m_vertexBuffer).eval();

            // Rotate normal buffer
            if (m_normalBuffer.cols() > 0)
                m_normalBuffer = (rotation * m_normalBuffer).eval();
        }

        //! Applies the given rotation to the mesh
        /*!
         * Vertices and normals are subject to rotation.
         *
         * Computation is equivalent to \f$ x_{new} = q * x_{old} * q^{-1} \f$. However, for performance reasons the quaternion is transformed to a corresponding rotation matrix.
         *
         * \param [in] rotation Quaternion \f$ q \f$ used for transformation
         */
        virtual void rotate(const Eigen::Quaterniond& rotation) { rotate(rotation.toRotationMatrix()); }

        //! Applies the given **uniform** scaling to the mesh
        /*!
         * Since the scaling is uniform, only vertices are affected by the scaling.
         *
         * \param [in] scaling The scaling factor used for transformation
         */
        virtual void scale(const double& scaling)
        {
            // Check validity of mesh first
            assert(isValid());

            // Uniform scaling -> only scale vertex buffer
            if (m_vertexBuffer.cols() > 0)
                if (core::isEqual(scaling, 1.0) == false)
                    m_vertexBuffer *= scaling;
        }

        //! Applies the given arbitrary scaling to the mesh
        /*!
         * If the scaling is **non-uniform**, vertices **and** normals are subject to scaling. Otherwise only vertices are transformed.
         *
         * \f[ x_{new} = \left[\begin{array}{ccc}s_x & 0 & 0 \\ 0 & s_y & 0 \\ 0 & 0 & s_z\end{array}\right] x_{old} \quad \mbox{and} \quad n_{new} = \left[\begin{array}{ccc}\frac{1}{s_x} & 0 & 0 \\ 0 & \frac{1}{s_y} & 0 \\ 0 & 0 & \frac{1}{s_z}\end{array}\right] x_{old}\f]
         *
         * \attention Normals are normalized after scaling since they change their length!
         *
         * \param [in] scaling Scaling vector \f$ [s_x,\,s_y,\,s_z] \f$ used for transformation
         */
        virtual void scale(const Eigen::Vector3d& scaling)
        {
            // Check if mesh is valid
            assert(isValid());

            // Check if scaling is uniform
            if (core::isEqual(scaling(0), scaling(1)) && core::isEqual(scaling(1), scaling(2))) {
                // Uniform scaling
                scale(scaling(0));
            } else {
                // Non-uniform scaling -> independent scaling for each component
                for (Eigen::Index i = 0; i < 3; i++) {
                    const double& si = scaling(i);
                    if (core::isEqual(si, 1.0) == false) {
                        // Scale vertex buffer
                        if (m_vertexBuffer.cols() > 0)
                            m_vertexBuffer.row(i) *= si;

                        // Scale normal buffer
                        if (m_normalBuffer.cols() > 0) {
                            double si_inv = 1e9; // Reciprocal of si
                            if (fabs(si) > 1e-9) // Avoids division by zero
                                si_inv = 1.0 / si;
                            m_normalBuffer.row(i) *= si_inv;
                        }
                    }
                }

                // Normalize all normals in the buffer
                normalizeNormals();
            }
        }

        //! Applies the given homogeneous transform to the mesh
        /*!
         * Vertices and normals are subject to transformation.
         *
         * \attention The homogeneous transformation matrix **must not** contain any scaling!
         *
         * \param [in] transform Homogeneous transformation matrix containing rotation and translation)
         */
        virtual void transform(const Eigen::Isometry3d& transform)
        {
            // First step: rotate
            rotate(transform.linear());

            // Second step: translate
            translate(transform.translation());
        }

        //! Change the coordinate system (applies the corresponding transformation to the buffers)
        /*!
         * Convenience function. Internally triggers rotation and translation.
         *
         * \note Imagine, that the body frame of an object coincides with the world coordinate system. The \p newOrigin, \p newXAxis and \p newZAxis then
         * describe the new body frame when seen from the world frame. The mesh buffers are transformed to be given in the world frame.
         *
         * \param [in] newOrigin Origin of **new** coordinate system
         * \param [in] newXAxis x-axis of **new** coordinate system (has to be normalized!)
         * \param [in] newZAxis z-axis of **new** coordinate system (has to be normalized!)
         */
        virtual void changeCoordinateSystem(const Eigen::Vector3d& newOrigin, const Eigen::Vector3d& newXAxis, const Eigen::Vector3d& newZAxis)
        {
            // Setup rotation matrix
            const Eigen::Vector3d newYAxis = newZAxis.cross(newXAxis).normalized();
            Eigen::Matrix3d rotation;
            rotation.col(0) = newXAxis;
            rotation.col(1) = newYAxis;
            rotation.col(2) = newZAxis;

            // First step: rotate
            rotate(rotation);

            // Second step: translate
            translate(newOrigin);
        }

    protected:
        // Serialization
        // -------------
        // Serialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize serializePayload(io::serialization::BinaryStream& stream, const io::serialization::Endianness& endianness) const
        {
            return io::serialization::serialize(stream, endianness, m_vertexBuffer, m_normalBuffer, m_colorBuffer, m_materialBuffer, m_indexBuffer);
        }

        // Deserialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize deSerializePayload(const io::serialization::BinaryStream& stream, const io::serialization::BinaryStreamSize& index, const io::serialization::BinaryStreamSize& payloadSize, const io::serialization::Endianness& endianness)
        {
            (void)payloadSize; // unused
            return io::serialization::deSerialize(stream, index, endianness, m_vertexBuffer, m_normalBuffer, m_colorBuffer, m_materialBuffer, m_indexBuffer);
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };

    //! \}
} // namespace geometry
} // namespace broccoli

#endif // HAVE_EIGEN3
