/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../core/math.hpp"
#include "../memory/MultiVector.hpp"
#include "CGMesh.hpp"
#include "CGMeshTools.hpp"
#include "CylindricalCoordinates.hpp"
#include "SphericalCoordinates.hpp"
#include <Eigen/StdVector>
#include <algorithm>
#include <array>
#include <assert.h>
#include <map>

namespace broccoli {
namespace geometry {
    /*!
     * \addtogroup broccoli_geometry
     * \{
     */

    //! Provides methods for convenient creation of \ref broccoli::geometry::CGMesh objects
    class CGMeshFactory {
    public:
        // Type definitions
        using IndexType = CGMesh::IndexType; //!< \copydoc broccoli::geometry::CGMesh::IndexType
        using Triangle = Eigen::Matrix<IndexType, 3, 1>; //!< Type definition of a single triangle of the mesh (vertex indices)
        using TriangleAllocator = Eigen::aligned_allocator<Triangle>; //!< Custom allocator for new triangles
        using TriangleList = std::vector<Triangle, TriangleAllocator>; //!< Type of triangle lists

        // Primitives
        // ----------
    public:
        //! Creates a plane (centered)
        /*!
         * \param [in] sizeX The plane dimension in **local** x-direction \f$l_x>0\f$
         * \param [in] sizeY The plane dimension in **local** y-direction \f$l_y>0\f$
         * \return Created mesh (empty mesh in case of an error, e.g. invalid input)
         */
        static inline CGMesh createPlane(const double& sizeX, const double& sizeY)
        {
            // Initialize mesh
            CGMesh mesh;

            // Check input
            if (sizeX <= 0 || sizeY <= 0) {
                assert(false);
                return mesh;
            }

            // Initialize helpers
            const double sizeXhalf = sizeX / 2.0;
            const double sizeYhalf = sizeY / 2.0;

            /* Setup vertex buffer
             *      y
             *      ^
             *      |
             *  3-------2
             *  |   | / |
             *  |   /---|-->x
             *  | /     |
             *  0-------1
             */
            mesh.m_vertexBuffer.resize(Eigen::NoChange, 4 /* <-- 4 vertices */);
            mesh.m_vertexBuffer.col(0) = Eigen::Vector3d(-sizeXhalf, -sizeYhalf, 0.0);
            mesh.m_vertexBuffer.col(1) = Eigen::Vector3d(sizeXhalf, -sizeYhalf, 0.0);
            mesh.m_vertexBuffer.col(2) = Eigen::Vector3d(sizeXhalf, sizeYhalf, 0.0);
            mesh.m_vertexBuffer.col(3) = Eigen::Vector3d(-sizeXhalf, sizeYhalf, 0.0);

            // Setup normal buffer
            mesh.m_normalBuffer.resize(Eigen::NoChange, 4 /* <-- 4 vertices */);
            for (int i = 0; i < 4; i++)
                mesh.m_normalBuffer.col(i) = Eigen::Vector3d(0.0, 0.0, 1.0);

            // Setup index buffer
            mesh.m_indexBuffer.resize(3, 2 /* <-- 2 triangles */);
            mesh.m_indexBuffer << 0, 0, //
                1, 2, //
                2, 3;

            // Pass back created mesh
            return mesh;
        }

        //! Creates a box with six faces in 3D space (centered)
        /*!
         * \param [in] sizeX The box dimension in **local** x-direction \f$l_x>0\f$
         * \param [in] sizeY The box dimension in **local** y-direction \f$l_y>0\f$
         * \param [in] sizeZ The box dimension in **local** z-direction \f$l_z>0\f$
         * \return Created mesh (empty mesh in case of an error, e.g. invalid input)
         */
        static inline CGMesh createBox(const double& sizeX, const double& sizeY, const double& sizeZ)
        {
            // Initialize mesh
            CGMesh mesh;

            // Check input
            if (sizeX <= 0 || sizeY <= 0 || sizeZ <= 0) {
                assert(false);
                return mesh;
            }

            // Initialize helpers
            const double sizeXhalf = sizeX / 2.0;
            const double sizeYhalf = sizeY / 2.0;
            const double sizeZhalf = sizeZ / 2.0;

            // Pre-compute vertices
            /*
             *                Z
             *                |
             *          7x----|----------x6
             *          /|    |         /|
             *         / |    |        / |
             *        /  |    |       /  |
             *      4x---------------x5  |
             *       |   |    |      |   |
             *       |   |    o------|-------Y
             *       |   |   /       |   |
             *       |  3x--/--------|---x2
             *       |  /  /         |  /
             *       | /  /          | /
             *       |/  X           |/
             *      0x---------------x1
             */
            Eigen::Matrix<double, 3, 8 /* <-- 8 corners */> vertices;
            vertices << sizeXhalf, sizeXhalf, -sizeXhalf, -sizeXhalf, sizeXhalf, sizeXhalf, -sizeXhalf, -sizeXhalf, //
                -sizeYhalf, sizeYhalf, sizeYhalf, -sizeYhalf, -sizeYhalf, sizeYhalf, sizeYhalf, -sizeYhalf, //
                -sizeZhalf, -sizeZhalf, -sizeZhalf, -sizeZhalf, sizeZhalf, sizeZhalf, sizeZhalf, sizeZhalf;

            // Pre-compute normals
            Eigen::Matrix<double, 3, 6 /* <-- 6 sides */> normals;
            normals << 1, -1, 0, 0, 0, 0, //
                0, 0, 1, -1, 0, 0, //
                0, 0, 0, 0, 1, -1;

            // Setup vertex buffer
            mesh.m_vertexBuffer.resize(Eigen::NoChange, 6 * 4 /* <-- 6 sides, 4 vertices per side */);
            // Face: positive x
            mesh.m_vertexBuffer.col(0) = vertices.col(0);
            mesh.m_vertexBuffer.col(1) = vertices.col(1);
            mesh.m_vertexBuffer.col(2) = vertices.col(5);
            mesh.m_vertexBuffer.col(3) = vertices.col(4);
            // Face: negative x
            mesh.m_vertexBuffer.col(4) = vertices.col(2);
            mesh.m_vertexBuffer.col(5) = vertices.col(3);
            mesh.m_vertexBuffer.col(6) = vertices.col(7);
            mesh.m_vertexBuffer.col(7) = vertices.col(6);
            // Face: positive y
            mesh.m_vertexBuffer.col(8) = vertices.col(1);
            mesh.m_vertexBuffer.col(9) = vertices.col(2);
            mesh.m_vertexBuffer.col(10) = vertices.col(6);
            mesh.m_vertexBuffer.col(11) = vertices.col(5);
            // Face: negative y
            mesh.m_vertexBuffer.col(12) = vertices.col(3);
            mesh.m_vertexBuffer.col(13) = vertices.col(0);
            mesh.m_vertexBuffer.col(14) = vertices.col(4);
            mesh.m_vertexBuffer.col(15) = vertices.col(7);
            // Face: positive z
            mesh.m_vertexBuffer.col(16) = vertices.col(4);
            mesh.m_vertexBuffer.col(17) = vertices.col(5);
            mesh.m_vertexBuffer.col(18) = vertices.col(6);
            mesh.m_vertexBuffer.col(19) = vertices.col(7);
            // Face: negative z
            mesh.m_vertexBuffer.col(20) = vertices.col(3);
            mesh.m_vertexBuffer.col(21) = vertices.col(2);
            mesh.m_vertexBuffer.col(22) = vertices.col(1);
            mesh.m_vertexBuffer.col(23) = vertices.col(0);

            // Setup normal buffer
            mesh.m_normalBuffer.resize(Eigen::NoChange, 6 * 4 /* <-- 6 sides, 4 vertices per side */);
            for (int i = 0; i < 6; i++)
                for (int j = 0; j < 4; j++)
                    mesh.m_normalBuffer.col(i * 4 + j) = normals.col(i);

            // Setup index buffer
            mesh.m_indexBuffer.resize(3, 6 * 2 /* <-- 6 sides, 2 triangles per side */);
            mesh.m_indexBuffer << 0, 0, 4, 4, 8, 8, 12, 12, 16, 16, 20, 20, //
                1, 2, 5, 6, 9, 10, 13, 14, 17, 18, 21, 22, //
                2, 3, 6, 7, 10, 11, 14, 15, 18, 19, 22, 23;

            // Pass back created mesh
            return mesh;
        }

        //! Creates an icosphere with the given number of subdivisions
        /*!
         * \remark The subdivision algorithm is adapted from Soft­ware­schnei­de­rei GmbH https://schneide.blog/2016/07/15/generating-an-icosphere-in-c/
         *
         * \param [in] radius The radius of the sphere \f$r>0\f$
         * \param [in] subDivisions The number of subdivisions for mesh refinement
         * \return Created mesh (empty mesh in case of an error, e.g. invalid input)
         */
        static inline CGMesh createIcoSphere(const double& radius, const uint64_t& subDivisions = 0)
        {
            // Type definitions
            using EdgeNewVertexLookupTable = std::map<std::pair<IndexType, IndexType>, IndexType>; // Maps a given edge (vertex indices, lower index first) to the generated "new" vertex during subdivision

            // Initialize mesh
            CGMesh mesh;

            // Check input
            if (radius <= 0) {
                assert(false);
                return mesh;
            }

            // Initialize helpers
            static const double a = 1.0;
            static const double b = (1.0 + std::sqrt(5.0)) / 2.0;
            static const double c = std::sqrt(a * a + b * b);
            static const double na = a / c;
            static const double nb = b / c;
            const double sa = na * radius;
            const double sb = nb * radius;

            // Compute edge/vertex/triangle count
            static const uint64_t originalVertexCount = 12; // Vertex count of an icosaeder
            static const uint64_t originalEdgeCount = 30; // Edge count of an icosaeder
            static const uint64_t originalTriangleCount = 20; // Triangle count of an icosaeder
            uint64_t vertexCount = originalVertexCount;
            uint64_t edgeCount = originalEdgeCount;
            uint64_t triangleCount = originalTriangleCount;
            for (uint64_t i = 0; i < subDivisions; i++) {
                // Compute delta
                const uint64_t additionalVertices = edgeCount; // For each edge a single vertex is added
                const uint64_t additionalEdges = edgeCount + 3 * triangleCount; // Each edge is split up into two edges + each triangle creates 3 new "interior" edges
                const uint64_t additionalTriangles = 3 * triangleCount; // Each triangle is split up into four triangles

                // Update counts
                vertexCount += additionalVertices;
                edgeCount += additionalEdges;
                triangleCount += additionalTriangles;
            }

            // Setup vertex buffer (fill with original icosaeder)
            mesh.m_vertexBuffer.resize(Eigen::NoChange, vertexCount);
            mesh.m_vertexBuffer.col(0) = Eigen::Vector3d(-sa, sb, 0.0);
            mesh.m_vertexBuffer.col(1) = Eigen::Vector3d(sa, sb, 0.0);
            mesh.m_vertexBuffer.col(2) = Eigen::Vector3d(-sa, -sb, 0.0);
            mesh.m_vertexBuffer.col(3) = Eigen::Vector3d(sa, -sb, 0.0);
            mesh.m_vertexBuffer.col(4) = Eigen::Vector3d(0.0, -sa, sb);
            mesh.m_vertexBuffer.col(5) = Eigen::Vector3d(0.0, sa, sb);
            mesh.m_vertexBuffer.col(6) = Eigen::Vector3d(0.0, -sa, -sb);
            mesh.m_vertexBuffer.col(7) = Eigen::Vector3d(0.0, sa, -sb);
            mesh.m_vertexBuffer.col(8) = Eigen::Vector3d(sb, 0.0, -sa);
            mesh.m_vertexBuffer.col(9) = Eigen::Vector3d(sb, 0.0, sa);
            mesh.m_vertexBuffer.col(10) = Eigen::Vector3d(-sb, 0.0, -sa);
            mesh.m_vertexBuffer.col(11) = Eigen::Vector3d(-sb, 0.0, sa);

            // Setup normal buffer (fill with original icosaeder)
            mesh.m_normalBuffer.resize(Eigen::NoChange, vertexCount);
            mesh.m_normalBuffer.col(0) = Eigen::Vector3d(-na, nb, 0.0);
            mesh.m_normalBuffer.col(1) = Eigen::Vector3d(na, nb, 0.0);
            mesh.m_normalBuffer.col(2) = Eigen::Vector3d(-na, -nb, 0.0);
            mesh.m_normalBuffer.col(3) = Eigen::Vector3d(na, -nb, 0.0);
            mesh.m_normalBuffer.col(4) = Eigen::Vector3d(0.0, -na, nb);
            mesh.m_normalBuffer.col(5) = Eigen::Vector3d(0.0, na, nb);
            mesh.m_normalBuffer.col(6) = Eigen::Vector3d(0.0, -na, -nb);
            mesh.m_normalBuffer.col(7) = Eigen::Vector3d(0.0, na, -nb);
            mesh.m_normalBuffer.col(8) = Eigen::Vector3d(nb, 0.0, -na);
            mesh.m_normalBuffer.col(9) = Eigen::Vector3d(nb, 0.0, na);
            mesh.m_normalBuffer.col(10) = Eigen::Vector3d(-nb, 0.0, -na);
            mesh.m_normalBuffer.col(11) = Eigen::Vector3d(-nb, 0.0, na);

            // Compute triangle list (may be changed during the subdivision process)
            TriangleList triangleList;
            triangleList.reserve(triangleCount);
            triangleList.push_back(Triangle(0, 11, 5));
            triangleList.push_back(Triangle(0, 5, 1));
            triangleList.push_back(Triangle(0, 1, 7));
            triangleList.push_back(Triangle(0, 7, 10));
            triangleList.push_back(Triangle(0, 10, 11));
            triangleList.push_back(Triangle(1, 5, 9));
            triangleList.push_back(Triangle(5, 11, 4));
            triangleList.push_back(Triangle(11, 10, 2));
            triangleList.push_back(Triangle(10, 7, 6));
            triangleList.push_back(Triangle(7, 1, 8));
            triangleList.push_back(Triangle(3, 9, 4));
            triangleList.push_back(Triangle(3, 4, 2));
            triangleList.push_back(Triangle(3, 2, 6));
            triangleList.push_back(Triangle(3, 6, 8));
            triangleList.push_back(Triangle(3, 8, 9));
            triangleList.push_back(Triangle(4, 9, 5));
            triangleList.push_back(Triangle(2, 4, 11));
            triangleList.push_back(Triangle(6, 2, 10));
            triangleList.push_back(Triangle(8, 6, 7));
            triangleList.push_back(Triangle(9, 8, 1));

            // Compute subdivisions
            if (subDivisions > 0) {
                IndexType currentVertexBufferSize = originalVertexCount; // Count of vertices in the vertex buffer
                TriangleList newTriangleList; // New triangle list (updated in each iteration of the subdivision process)
                newTriangleList.reserve(triangleCount);

                // Repeat for each subdivision
                for (uint64_t s = 0; s < subDivisions; s++) {
                    // Re-initialize helpers
                    EdgeNewVertexLookupTable edgeVertexMap; // Maps edges to the new vertices they create
                    newTriangleList.clear();

                    // Pass through all triangles
                    for (size_t t = 0; t < triangleList.size(); t++) {
                        const Triangle& currentTriangle = triangleList[t]; // The "old" triangle to be subdivided
                        std::array<IndexType, 3> currentTriangleNewVertexIndices; // Indices of new vertices created by subdividing the current triangle

                        // Pass through edges of this triangle
                        for (int e = 0; e < 3; e++) {
                            const IndexType& firstEdgeVertexIndex = currentTriangle(e); // First vertex of this edge
                            const IndexType secondEdgeVertexIndex = currentTriangle((e + 1) % 3); // Second vertex of this edge

                            // Create new edge/vertex pair and add it to the lookup-table (if it is not contained already)
                            EdgeNewVertexLookupTable::key_type key(firstEdgeVertexIndex, secondEdgeVertexIndex);
                            if (key.first > key.second) // Ensure, that first edge vertex has lower index
                                std::swap(key.first, key.second);
                            auto insertResult = edgeVertexMap.insert({ key, currentVertexBufferSize });
                            const bool isNewVertex = insertResult.second; // Flag indicating, if this is a new vertex, or it is already in the lookup table
                            const IndexType edgeCenterVertexIndex = insertResult.first->second; // Index of the vertex in the center of the currently investigated edge

                            // Check, if this is a new vertex
                            if (isNewVertex == true) {
                                // Compute vertex position and normal
                                mesh.m_normalBuffer.col(currentVertexBufferSize) = (mesh.m_vertexBuffer.col(firstEdgeVertexIndex) + mesh.m_vertexBuffer.col(secondEdgeVertexIndex)).normalized(); // Centered in edge + move to unit sphere surface
                                mesh.m_vertexBuffer.col(currentVertexBufferSize) = mesh.m_normalBuffer.col(currentVertexBufferSize) * radius; // Move to surface of sphere (radius!)
                                currentVertexBufferSize++;
                            }

                            // Update "new" vertex indices of triangle
                            currentTriangleNewVertexIndices[e] = edgeCenterVertexIndex;
                        }

                        // Split "old" triangle into 4 smaller triangles
                        newTriangleList.push_back(Triangle(currentTriangle(0), currentTriangleNewVertexIndices[0], currentTriangleNewVertexIndices[2]));
                        newTriangleList.push_back(Triangle(currentTriangle(1), currentTriangleNewVertexIndices[1], currentTriangleNewVertexIndices[0]));
                        newTriangleList.push_back(Triangle(currentTriangle(2), currentTriangleNewVertexIndices[2], currentTriangleNewVertexIndices[1]));
                        newTriangleList.push_back(Triangle(currentTriangleNewVertexIndices[0], currentTriangleNewVertexIndices[1], currentTriangleNewVertexIndices[2]));
                    }

                    // Update triangle lists
                    triangleList.swap(newTriangleList);
                }
            }

            // Setup index buffer
            mesh.m_indexBuffer.resize(3, triangleList.size());
            for (size_t i = 0; i < triangleList.size(); i++)
                mesh.m_indexBuffer.col(i) = triangleList[i];

            // Pass back created mesh
            return mesh;
        }

        //! Creates a slice of a circle in 3D space (uses default cylindrical coordinates \ref CylindricalCoordinates)
        /*!
         * \param [in] minimumRadius Inner radius \f$ r_{min} \f$, constraint: \f$ r_{min} \geq 0 \f$
         * \param [in] maximumRadius Outer radius \f$ r_{max} \f$, constraint: \f$ r_{max} > r_{min} \f$
         * \param [in] stepsRadius Step count \f$ r_{step} \f$ for discretizing \f$ r \f$, constraint: \f$ r_{step} > 0 \f$
         * \param [in] minimumPhi Minimum angle \f$ \varphi_{min} \f$ (rad), constraint: \f$ \varphi_{min} \in [0, 2\pi[ \f$
         * \param [in] maximumPhi Maximum angle \f$ \varphi_{max} \f$ (rad), constraint: \f$ \varphi_{max} \in ]\varphi_{min}, 2\pi]\f$
         * \param [in] stepsPhi Step count \f$ \varphi_{step} \f$ for discretizing \f$ \varphi \f$, constraint: \f$ \varphi_{step} > 0 \f$; for \f$ \varphi_{step} < 2 \f$: \f$ \varphi_{max} - \varphi_{min} < \pi \f$; for \f$ \varphi_{step} < 3 \f$: \f$ \varphi_{max} - \varphi_{min} < 2\pi \f$
         * \return Created mesh (empty mesh in case of an error, e.g. invalid input)
         */
        static inline CGMesh createCircleSlice(const double& minimumRadius, const double& maximumRadius, const uint64_t& stepsRadius, const double& minimumPhi, const double& maximumPhi, const uint64_t& stepsPhi)
        {
            return createCylinderSlice(minimumRadius, maximumRadius, stepsRadius, minimumPhi, maximumPhi, stepsPhi, 0.0, 0.0, 0);
        }

        //! Creates a sector of a circle in 3D space (uses default cylindrical coordinates \ref CylindricalCoordinates)
        /*!
         * \param [in] radius Outer radius \f$ r \f$, constraint: \f$ r > 0 \f$
         * \param [in] minimumPhi Minimum angle \f$ \varphi_{min} \f$ (rad), constraint: \f$ \varphi_{min} \in [0, 2\pi[ \f$
         * \param [in] maximumPhi Maximum angle \f$ \varphi_{max} \f$ (rad), constraint: \f$ \varphi_{max} \in ]\varphi_{min}, 2\pi]\f$
         * \param [in] stepsPhi Step count \f$ \varphi_{step} \f$ for discretizing \f$ \varphi \f$, constraint: \f$ \varphi_{step} > 0 \f$; for \f$ \varphi_{step} < 2 \f$: \f$ \varphi_{max} - \varphi_{min} < \pi \f$; for \f$ \varphi_{step} < 3 \f$: \f$ \varphi_{max} - \varphi_{min} < 2\pi \f$
         * \return Created mesh (empty mesh in case of an error, e.g. invalid input)
         */
        static inline CGMesh createCircleSector(const double& radius, const double& minimumPhi, const double& maximumPhi, const uint64_t& stepsPhi)
        {
            return createCylinderSlice(0.0, radius, 1, minimumPhi, maximumPhi, stepsPhi, 0.0, 0.0, 0);
        }

        //! Creates a circle in 3D space (uses default cylindrical coordinates \ref CylindricalCoordinates)
        /*!
         * \param [in] radius Outer radius \f$ r \f$, constraint: \f$ r > 0 \f$
         * \param [in] stepsPhi Step count \f$ \varphi_{step} \f$ for discretizing \f$ \varphi \f$, constraint: \f$ \varphi_{step} > 2 \f$
         * \return Created mesh (empty mesh in case of an error, e.g. invalid input)
         */
        static inline CGMesh createCircle(const double& radius, const uint64_t& stepsPhi)
        {
            return createCylinderSlice(0.0, radius, 1, 0.0, 2.0 * M_PI, stepsPhi, 0.0, 0.0, 0);
        }

        //! Creates a slice of a cylinder in 3D space (uses default cylindrical coordinates \ref CylindricalCoordinates)
        /*!
         * \param [in] minimumRadius Inner radius \f$ r_{min} \f$, constraint: \f$ r_{min} \geq 0 \f$
         * \param [in] maximumRadius Outer radius \f$ r_{max} \f$, constraint: \f$ r_{max} > 0 \f$, \f$ r_{max} \geq r_{min} \f$ (\f$ r_{max} \f$-surface only for \f$ r_{max} = r_{min} \f$)
         * \param [in] stepsRadius Step count \f$ r_{step} \f$ for discretizing \f$ r \f$, constraint: for \f$ r_{step} = 0 \f$: \f$ r_{max} = r_{min} \f$ and \f$ \varphi_{step} > 0\f$ and \f$ z_{step} > 0\f$; for \f$ r_{step} > 0 \f$: \f$ r_{max} > r_{min} \f$
         * \param [in] minimumPhi Minimum angle \f$ \varphi_{min} \f$ (rad), constraint: \f$ \varphi_{min} \in [0, 2\pi] \f$
         * \param [in] maximumPhi Maximum angle \f$ \varphi_{max} \f$ (rad), constraint: \f$ \varphi_{max} \in [\varphi_{min}, 2\pi]\f$ (\f$ \varphi_{max} \f$-surface only for \f$ \varphi_{max} = \varphi_{min} \f$)
         * \param [in] stepsPhi Step count \f$ \varphi_{step} \f$ for discretizing \f$ \varphi \f$, constraint: for \f$ \varphi_{step} = 0 \f$: \f$ \varphi_{max} = \varphi_{min} \f$ and \f$ r_{step} > 0\f$ and \f$ z_{step} > 0\f$; for \f$ \varphi_{step} > 0 \f$: \f$ \varphi_{max} > \varphi_{min} \f$; for \f$ \varphi_{step} < 2 \f$: \f$ \varphi_{max} - \varphi_{min} < \pi \f$; for \f$ \varphi_{step} < 3 \f$: \f$ \varphi_{max} - \varphi_{min} < 2\pi \f$
         * \param [in] minimumZ Minimum \f$ z_{min} \f$ coordinate, constraint: none
         * \param [in] maximumZ Maximum \f$ z_{max} \f$ coordinate, constraint: \f$ z_{max} >= z_{min} \f$ (\f$ z_{max} \f$-surface only for \f$ z_{max} = z_{min} \f$)
         * \param [in] stepsZ Step count \f$ z_{step} \f$ for discretizing \f$ z \f$, constraint: for \f$ z_{step} = 0 \f$: \f$ z_{max} = z_{min} \f$ and \f$ r_{step} > 0\f$ and \f$ \varphi_{step} > 0\f$; for \f$ z_{step} > 0 \f$: \f$ z_{max} > z_{min} \f$
         * \return Created mesh (empty mesh in case of an error, e.g. invalid input)
         */
        static inline CGMesh createCylinderSlice(const double& minimumRadius, const double& maximumRadius, const uint64_t& stepsRadius, const double& minimumPhi, const double& maximumPhi, const uint64_t& stepsPhi, const double& minimumZ, const double& maximumZ, const uint64_t& stepsZ)
        {
            // Initialize mesh
            CGMesh mesh;

            // Compute ranges
            const double rangeRadius = maximumRadius - minimumRadius;
            const double rangePhi = maximumPhi - minimumPhi;
            const double rangeZ = maximumZ - minimumZ;

            // Check if minimum/maximum values are the same
            const bool sameRadius = core::isZero(rangeRadius);
            const bool samePhi = core::isZero(rangePhi);
            const bool sameZ = core::isZero(rangeZ);

            // Check inputs
            if (minimumRadius < 0.0 || maximumRadius <= 0.0 || maximumRadius < minimumRadius || (stepsRadius == 0 && (!sameRadius || stepsPhi == 0 || stepsZ == 0)) || (stepsRadius > 0 && sameRadius) || //
                minimumPhi < 0.0 || maximumPhi < minimumPhi || maximumPhi > 2 * M_PI || (stepsPhi == 0 && (!samePhi || stepsRadius == 0 || stepsZ == 0)) || (stepsPhi > 0 && samePhi) || (stepsPhi < 2 && rangePhi >= M_PI) || (stepsPhi < 3 && rangePhi >= 2.0 * M_PI) || //
                maximumZ < minimumZ || (stepsZ == 0 && (!sameZ || stepsRadius == 0 || stepsPhi == 0)) || (stepsZ > 0 && sameZ)) {
                assert(false);
                return mesh;
            }

            // Check special cases
            const bool minimumRadiusZero = core::isZero(minimumRadius);
            const bool fullPhi = core::isZero(rangePhi - 2 * M_PI);

            // Compute step size
            const double deltaRadius = (stepsRadius > 0) ? rangeRadius / stepsRadius : 0.0;
            const double deltaPhi = (stepsPhi > 0) ? rangePhi / stepsPhi : 0.0;
            const double deltaZ = (stepsZ > 0) ? rangeZ / stepsZ : 0.0;

            // Compute discrete coordinate values
            memory::SmartVector<double, 16, std::allocator<double>> valuesRadius(stepsRadius + 1);
            for (size_t i = 0; i < valuesRadius.size(); i++)
                valuesRadius[i] = minimumRadius + i * deltaRadius;
            memory::SmartVector<double, 128, std::allocator<double>> valuesPhi(stepsPhi + 1);
            for (size_t i = 0; i < valuesPhi.size(); i++)
                valuesPhi[i] = minimumPhi + i * deltaPhi;
            memory::SmartVector<double, 16, std::allocator<double>> valuesZ(stepsZ + 1);
            for (size_t i = 0; i < valuesZ.size(); i++)
                valuesZ[i] = minimumZ + i * deltaZ;

            // Compute vertex and triangle count (maximum and minimum radius)
            uint64_t vertexCountMaximumRadius = 0;
            uint64_t triangleCountMaximumRadius = 0;
            uint64_t vertexCountMinimumRadius = 0;
            uint64_t triangleCountMinimumRadius = 0;
            if (stepsPhi > 0 && stepsZ > 0) {
                // Maximum radius
                if (fullPhi == false)
                    vertexCountMaximumRadius = valuesPhi.size() * valuesZ.size();
                else
                    vertexCountMaximumRadius = (valuesPhi.size() - 1) * valuesZ.size();
                triangleCountMaximumRadius = stepsPhi * stepsZ * 2;

                // Minimum radius
                if (sameRadius == false && minimumRadiusZero == false) {
                    vertexCountMinimumRadius = vertexCountMaximumRadius;
                    triangleCountMinimumRadius = triangleCountMaximumRadius;
                }
            }

            // Compute vertex and triangle count (maximum and minimum phi)
            uint64_t vertexCountMaximumPhi = 0;
            uint64_t triangleCountMaximumPhi = 0;
            uint64_t vertexCountMinimumPhi = 0;
            uint64_t triangleCountMinimumPhi = 0;
            if (stepsRadius > 0 && stepsZ > 0 && fullPhi == false) {
                // Maximum phi
                vertexCountMaximumPhi = valuesRadius.size() * valuesZ.size();
                triangleCountMaximumPhi = stepsRadius * stepsZ * 2;

                // Minimum phi
                if (samePhi == false) {
                    vertexCountMinimumPhi = vertexCountMaximumPhi;
                    triangleCountMinimumPhi = triangleCountMaximumPhi;
                }
            }

            // Compute vertex and triangle count (maximum and minimum z)
            uint64_t vertexCountMaximumZ = 0;
            uint64_t triangleCountMaximumZ = 0;
            uint64_t vertexCountMinimumZ = 0;
            uint64_t triangleCountMinimumZ = 0;
            if (stepsRadius > 0 && stepsPhi > 0) {
                // Maximum z
                if (minimumRadiusZero == false) {
                    if (fullPhi == false)
                        vertexCountMaximumZ = valuesRadius.size() * valuesPhi.size();
                    else
                        vertexCountMaximumZ = valuesRadius.size() * (valuesPhi.size() - 1);
                    triangleCountMaximumZ = stepsRadius * stepsPhi * 2;
                } else {
                    if (fullPhi == false)
                        vertexCountMaximumZ = 1 + (valuesRadius.size() - 1) * valuesPhi.size();
                    else
                        vertexCountMaximumZ = 1 + (valuesRadius.size() - 1) * (valuesPhi.size() - 1);
                    triangleCountMaximumZ = (2 * stepsRadius - 1) * stepsPhi;
                }

                // Minimum z
                if (sameZ == false) {
                    vertexCountMinimumZ = vertexCountMaximumZ;
                    triangleCountMinimumZ = triangleCountMaximumZ;
                }
            }

            // Compute total vertex and triangle count
            uint64_t vertexCount = vertexCountMaximumRadius + vertexCountMinimumRadius + vertexCountMaximumPhi + vertexCountMinimumPhi + vertexCountMaximumZ + vertexCountMinimumZ;
            uint64_t triangleCount = triangleCountMaximumRadius + triangleCountMinimumRadius + triangleCountMaximumPhi + triangleCountMinimumPhi + triangleCountMaximumZ + triangleCountMinimumZ;

            // Allocate buffers
            mesh.m_vertexBuffer.resize(Eigen::NoChange, vertexCount);
            mesh.m_normalBuffer.resize(Eigen::NoChange, vertexCount);
            mesh.m_indexBuffer.resize(3, triangleCount);
            IndexType currentVertexIndex = 0;
            IndexType currentTriangleIndex = 0;

            // Draw surfaces for maximum and minimum parameter (m=0 -> minimum, m=1 -> maximum)
            for (int m = 0; m <= 1; m++) {
                const bool isMinimumSurface = (m == 0);
                const bool isMaximumSurface = !isMinimumSurface;

                // Draw surface for maximum and minimum radius
                if ((isMinimumSurface && triangleCountMinimumRadius > 0) || (isMaximumSurface && triangleCountMaximumRadius > 0)) {
                    const IndexType gridVertexStartIndex = currentVertexIndex;
                    const double& radius = (isMinimumSurface) ? minimumRadius : maximumRadius;

                    // Create vertices with normals
                    const size_t endIndexPhi = (fullPhi == true) ? valuesPhi.size() - 2 : valuesPhi.size() - 1;
                    CylindricalCoordinates coordinates(radius, 0.0, 0.0);
                    for (size_t i = 0; i <= endIndexPhi; i++) {
                        coordinates.setPhi(valuesPhi[i]);
                        for (size_t j = 0; j < valuesZ.size(); j++) {
                            coordinates.setZ(valuesZ[j]);
                            addVertexWithNormal(mesh, currentVertexIndex, coordinates.toCartesian(), coordinates.toCartesianFirstDerivativeRadiusNormalized(), isMinimumSurface);
                        }
                    }

                    // Create triangles
                    for (size_t i = 0; i < stepsPhi; i++) {
                        for (size_t j = 0; j < stepsZ; j++) {
                            const IndexType index0 = gridVertexStartIndex + i * valuesZ.size() + j;
                            const IndexType index1 = (fullPhi == true && i + 1 == stepsPhi) ? gridVertexStartIndex + j : index0 + valuesZ.size();
                            addQuad(mesh, currentTriangleIndex, index0, index1, index1 + 1, index0 + 1, isMinimumSurface);
                        }
                    }
                }

                // Draw surface for maximum and minimum phi
                if ((isMinimumSurface && triangleCountMinimumPhi > 0) || (isMaximumSurface && triangleCountMaximumPhi > 0)) {
                    const IndexType gridVertexStartIndex = currentVertexIndex;
                    const double& phi = (isMinimumSurface) ? minimumPhi : maximumPhi;

                    // Create vertices with normals
                    CylindricalCoordinates coordinates(0.0, phi, 0.0);
                    for (size_t i = 0; i < valuesRadius.size(); i++) {
                        coordinates.setRadius(valuesRadius[i]);
                        for (size_t j = 0; j < valuesZ.size(); j++) {
                            coordinates.setZ(valuesZ[j]);
                            addVertexWithNormal(mesh, currentVertexIndex, coordinates.toCartesian(), coordinates.toCartesianFirstDerivativePhiNormalized(), isMinimumSurface);
                        }
                    }

                    // Create triangles
                    for (size_t i = 0; i < stepsRadius; i++) {
                        for (size_t j = 0; j < stepsZ; j++) {
                            const IndexType index0 = gridVertexStartIndex + i * valuesZ.size() + j;
                            const IndexType index3 = index0 + valuesZ.size();
                            addQuad(mesh, currentTriangleIndex, index0, index0 + 1, index3 + 1, index3, isMinimumSurface);
                        }
                    }
                }

                // Draw surface for maximum and minimum z
                if ((isMinimumSurface && triangleCountMinimumZ > 0) || (isMaximumSurface && triangleCountMaximumZ > 0)) {
                    const IndexType gridVertexStartIndex = currentVertexIndex;
                    const double& z = (isMinimumSurface) ? minimumZ : maximumZ;

                    // Create vertices with normals
                    CylindricalCoordinates coordinates(0.0, 0.0, z);
                    if (minimumRadiusZero == true)
                        addVertexWithNormal(mesh, currentVertexIndex, coordinates.toCartesian(), coordinates.toCartesianFirstDerivativeZNormalized(), isMinimumSurface);
                    const size_t startIndexRadius = (minimumRadiusZero == true) ? 1 : 0;
                    const size_t endIndexPhi = (fullPhi == true) ? valuesPhi.size() - 2 : valuesPhi.size() - 1;
                    for (size_t i = 0; i <= endIndexPhi; i++) {
                        coordinates.setPhi(valuesPhi[i]);
                        for (size_t j = startIndexRadius; j < valuesRadius.size(); j++) {
                            coordinates.setRadius(valuesRadius[j]);
                            addVertexWithNormal(mesh, currentVertexIndex, coordinates.toCartesian(), coordinates.toCartesianFirstDerivativeZNormalized(), isMinimumSurface);
                        }
                    }

                    // Create triangles
                    for (size_t i = 0; i < stepsPhi; i++) {
                        for (size_t j = 0; j < stepsRadius; j++) {
                            if (minimumRadiusZero == false) {
                                const IndexType index0 = gridVertexStartIndex + valuesRadius.size() * i + j;
                                const IndexType index3 = (fullPhi == true && i + 1 == stepsPhi) ? gridVertexStartIndex + j : index0 + valuesRadius.size();
                                addQuad(mesh, currentTriangleIndex, index0, index0 + 1, index3 + 1, index3, isMinimumSurface);
                            } else {
                                if (j == 0) {
                                    const IndexType index0 = gridVertexStartIndex;
                                    const IndexType index1 = gridVertexStartIndex + 1 + (valuesRadius.size() - 1) * i;
                                    const IndexType index2 = (fullPhi == true && i + 1 == stepsPhi) ? gridVertexStartIndex + 1 : index1 + (valuesRadius.size() - 1);
                                    addTriangle(mesh, currentTriangleIndex, index0, index1, index2, isMinimumSurface);
                                } else {
                                    const IndexType index0 = gridVertexStartIndex + (valuesRadius.size() - 1) * i + j;
                                    const IndexType index3 = (fullPhi == true && i + 1 == stepsPhi) ? gridVertexStartIndex + j : index0 + (valuesRadius.size() - 1);
                                    addQuad(mesh, currentTriangleIndex, index0, index0 + 1, index3 + 1, index3, isMinimumSurface);
                                }
                            }
                        }
                    }
                }
            }

            // Pass back created mesh
            return mesh;
        }

        //! Creates a sector of a cylinder in 3D space (uses default cylindrical coordinates \ref CylindricalCoordinates)
        /*!
         * \param [in] radius Outer radius \f$ r \f$, constraint: \f$ r > 0 \f$
         * \param [in] minimumPhi Minimum angle \f$ \varphi_{min} \f$ (rad), constraint: \f$ \varphi_{min} \in [0, 2\pi] \f$
         * \param [in] maximumPhi Maximum angle \f$ \varphi_{max} \f$ (rad), constraint: \f$ \varphi_{max} \in [\varphi_{min}, 2\pi]\f$ (\f$ \varphi_{max} \f$-surface only for \f$ \varphi_{max} = \varphi_{min} \f$)
         * \param [in] stepsPhi Step count \f$ \varphi_{step} \f$ for discretizing \f$ \varphi \f$, constraint: for \f$ \varphi_{step} = 0 \f$: \f$ \varphi_{max} = \varphi_{min} \f$ and \f$ z_{step} > 0\f$; for \f$ \varphi_{step} > 0 \f$: \f$ \varphi_{max} > \varphi_{min} \f$; for \f$ \varphi_{step} < 2 \f$: \f$ \varphi_{max} - \varphi_{min} < \pi \f$; for \f$ \varphi_{step} < 3 \f$: \f$ \varphi_{max} - \varphi_{min} < 2\pi \f$
         * \param [in] minimumZ Minimum \f$ z_{min} \f$ coordinate, constraint: none
         * \param [in] maximumZ Maximum \f$ z_{max} \f$ coordinate, constraint: \f$ z_{max} >= z_{min} \f$ (\f$ z_{max} \f$-surface only for \f$ z_{max} = z_{min} \f$)
         * \param [in] stepsZ Step count \f$ z_{step} \f$ for discretizing \f$ z \f$, constraint: for \f$ z_{step} = 0 \f$: \f$ z_{max} = z_{min} \f$ and \f$ \varphi_{step} > 0\f$; for \f$ z_{step} > 0 \f$: \f$ z_{max} > z_{min} \f$
         * \return Created mesh (empty mesh in case of an error, e.g. invalid input)
         */
        static inline CGMesh createCylinderSector(const double& radius, const double& minimumPhi, const double& maximumPhi, const uint64_t& stepsPhi, const double& minimumZ, const double& maximumZ, const uint64_t& stepsZ)
        {
            return createCylinderSlice(0.0, radius, 1, minimumPhi, maximumPhi, stepsPhi, minimumZ, maximumZ, stepsZ);
        }

        //! Creates a cylinder in 3D space (uses default cylindrical coordinates \ref CylindricalCoordinates)
        /*!
         * \param [in] radius Outer radius \f$ r \f$, constraint: \f$ r > 0 \f$
         * \param [in] stepsPhi Step count \f$ \varphi_{step} \f$ for discretizing \f$ \varphi \f$, constraint: \f$ \varphi_{step} > 2 \f$
         * \param [in] minimumZ Minimum \f$ z_{min} \f$ coordinate, constraint: none
         * \param [in] maximumZ Maximum \f$ z_{max} \f$ coordinate, constraint: \f$ z_{max} >= z_{min} \f$ (\f$ z_{max} \f$-surface only for \f$ z_{max} = z_{min} \f$)
         * \param [in] stepsZ Step count \f$ z_{step} \f$ for discretizing \f$ z \f$, constraint: for \f$ z_{step} = 0 \f$: \f$ z_{max} = z_{min} \f$; for \f$ z_{step} > 0 \f$: \f$ z_{max} > z_{min} \f$
         * \return Created mesh (empty mesh in case of an error, e.g. invalid input)
         */
        static inline CGMesh createCylinder(const double& radius, const uint64_t& stepsPhi, const double& minimumZ, const double& maximumZ, const uint64_t& stepsZ)
        {
            return createCylinderSlice(0.0, radius, 1, 0.0, 2.0 * M_PI, stepsPhi, minimumZ, maximumZ, stepsZ);
        }

        //! Creates a slice of a sphere in 3D space (uses default spherical coordinates \ref SphericalCoordinates)
        /*!
         * \param [in] minimumRadius Inner radius \f$ r_{min} \f$, constraint: \f$ r_{min} \geq 0 \f$
         * \param [in] maximumRadius Outer radius \f$ r_{max} \f$, constraint: \f$ r_{max} > 0 \f$, \f$ r_{max} \geq r_{min} \f$ (\f$ r_{max} \f$-surface only for \f$ r_{max} = r_{min} \f$)
         * \param [in] stepsRadius Step count \f$ r_{step} \f$ for discretizing \f$ r \f$, constraint: for \f$ r_{step} = 0 \f$: \f$ r_{max} = r_{min} \f$ and \f$ \varphi_{step} > 0\f$ and \f$ \vartheta_{step} > 0\f$; for \f$ r_{step} > 0 \f$: \f$ r_{max} > r_{min} \f$
         * \param [in] minimumPhi Minimum angle \f$ \varphi_{min} \f$ (rad), constraint: \f$ \varphi_{min} \in [0, 2\pi] \f$
         * \param [in] maximumPhi Maximum angle \f$ \varphi_{max} \f$ (rad), constraint: \f$ \varphi_{max} \in [\varphi_{min}, 2\pi]\f$ (\f$ \varphi_{max} \f$-surface only for \f$ \varphi_{max} = \varphi_{min} \f$)
         * \param [in] stepsPhi Step count \f$ \varphi_{step} \f$ for discretizing \f$ \varphi \f$, constraint: for \f$ \varphi_{step} = 0 \f$: \f$ \varphi_{max} = \varphi_{min} \f$ and \f$ r_{step} > 0\f$ and \f$ \vartheta_{step} > 0\f$; for \f$ \varphi_{step} > 0 \f$: \f$ \varphi_{max} > \varphi_{min} \f$; for \f$ \varphi_{step} < 2 \f$: \f$ \varphi_{max} - \varphi_{min} < \pi \f$; for \f$ \varphi_{step} < 3 \f$: \f$ \varphi_{max} - \varphi_{min} < 2\pi \f$
         * \param [in] minimumTheta Minimum angle \f$ \vartheta_{min} \f$ (rad), constraint: \f$ \vartheta_{min} \in [0, \pi[ \f$
         * \param [in] maximumTheta Maximum angle \f$ \vartheta_{max} \f$ (rad), constraint: \f$ \vartheta_{max} \in ]0, \pi] \f$ and \f$ \vartheta_{max} >= \vartheta_{min} \f$ (\f$ \vartheta_{max} \f$-surface only for \f$ \vartheta_{max} = \vartheta_{min} \f$)
         * \param [in] stepsTheta Step count \f$ \vartheta_{step} \f$ for discretizing \f$ \vartheta \f$, constraint: for \f$ \vartheta_{step} = 0 \f$: \f$ \vartheta_{max} = \vartheta_{min} \f$ and \f$ r_{step} > 0\f$ and \f$ \varphi_{step} > 0\f$; for \f$ \vartheta_{step} > 0 \f$: \f$ \vartheta_{max} > \vartheta_{min} \f$; for \f$ \vartheta_{step} < 2 \f$: \f$ \vartheta_{max} - \vartheta_{min} < \pi \f$
         * \return Created mesh (empty mesh in case of an error, e.g. invalid input)
         */
        static inline CGMesh createSphereSlice(const double& minimumRadius, const double& maximumRadius, const uint64_t& stepsRadius, const double& minimumPhi, const double& maximumPhi, const uint64_t& stepsPhi, const double& minimumTheta, const double& maximumTheta, const uint64_t& stepsTheta)
        {
            // Initialize mesh
            CGMesh mesh;

            // Compute ranges
            const double rangeRadius = maximumRadius - minimumRadius;
            const double rangePhi = maximumPhi - minimumPhi;
            const double rangeTheta = maximumTheta - minimumTheta;

            // Check if minimum/maximum values are the same
            const bool sameRadius = core::isZero(rangeRadius);
            const bool samePhi = core::isZero(rangePhi);
            const bool sameTheta = core::isZero(rangeTheta);

            // Check inputs
            if (minimumRadius < 0.0 || maximumRadius <= 0.0 || maximumRadius < minimumRadius || (stepsRadius == 0 && (!sameRadius || stepsPhi == 0 || stepsTheta == 0)) || (stepsRadius > 0 && sameRadius) || //
                minimumPhi < 0.0 || maximumPhi < minimumPhi || maximumPhi > 2 * M_PI || (stepsPhi == 0 && (!samePhi || stepsRadius == 0 || stepsTheta == 0)) || (stepsPhi > 0 && samePhi) || (stepsPhi < 2 && rangePhi >= M_PI) || (stepsPhi < 3 && rangePhi >= 2.0 * M_PI) || //
                minimumTheta < 0.0 || minimumTheta >= M_PI || maximumTheta <= 0.0 || maximumTheta > M_PI || maximumTheta < minimumTheta || (stepsTheta == 0 && (!sameTheta || stepsRadius == 0 || stepsPhi == 0)) || (stepsTheta > 0 && sameTheta) || (stepsTheta < 2 && rangeTheta >= M_PI)) {
                assert(false);
                return mesh;
            }

            // Check special cases
            const bool minimumRadiusZero = core::isZero(minimumRadius);
            const bool fullPhi = core::isZero(rangePhi - 2 * M_PI);
            const bool minimumThetaZero = core::isZero(minimumTheta);
            const bool maximumThetaPi = core::isZero(maximumTheta - M_PI);

            // Compute step size
            const double deltaRadius = (stepsRadius > 0) ? rangeRadius / stepsRadius : 0.0;
            const double deltaPhi = (stepsPhi > 0) ? rangePhi / stepsPhi : 0.0;
            const double deltaTheta = (stepsTheta > 0) ? rangeTheta / stepsTheta : 0.0;

            // Compute discrete coordinate values
            memory::SmartVector<double, 16, std::allocator<double>> valuesRadius(stepsRadius + 1);
            for (size_t i = 0; i < valuesRadius.size(); i++)
                valuesRadius[i] = minimumRadius + i * deltaRadius;
            memory::SmartVector<double, 128, std::allocator<double>> valuesPhi(stepsPhi + 1);
            for (size_t i = 0; i < valuesPhi.size(); i++)
                valuesPhi[i] = minimumPhi + i * deltaPhi;
            memory::SmartVector<double, 128, std::allocator<double>> valuesTheta(stepsTheta + 1);
            for (size_t i = 0; i < valuesTheta.size(); i++)
                valuesTheta[i] = minimumTheta + i * deltaTheta;

            // Compute vertex and triangle count (maximum and minimum radius)
            uint64_t vertexCountMaximumRadius = 0;
            uint64_t triangleCountMaximumRadius = 0;
            uint64_t vertexCountMinimumRadius = 0;
            uint64_t triangleCountMinimumRadius = 0;
            if (stepsPhi > 0 && stepsTheta > 0) {
                // Maximum radius
                const size_t newPhiValues = (fullPhi == true) ? valuesPhi.size() - 1 : valuesPhi.size();
                vertexCountMaximumRadius = newPhiValues * valuesTheta.size();
                triangleCountMaximumRadius = stepsPhi * stepsTheta * 2;
                if (minimumThetaZero == true) {
                    vertexCountMaximumRadius -= newPhiValues - 1;
                    triangleCountMaximumRadius -= stepsPhi;
                }
                if (maximumThetaPi == true) {
                    vertexCountMaximumRadius -= newPhiValues - 1;
                    triangleCountMaximumRadius -= stepsPhi;
                }

                // Minimum radius
                if (sameRadius == false && minimumRadiusZero == false) {
                    vertexCountMinimumRadius = vertexCountMaximumRadius;
                    triangleCountMinimumRadius = triangleCountMaximumRadius;
                }
            }

            // Compute vertex and triangle count (maximum and minimum phi)
            uint64_t vertexCountMaximumPhi = 0;
            uint64_t triangleCountMaximumPhi = 0;
            uint64_t vertexCountMinimumPhi = 0;
            uint64_t triangleCountMinimumPhi = 0;
            if (stepsRadius > 0 && stepsTheta > 0 && fullPhi == false) {
                // Maximum phi
                vertexCountMaximumPhi = valuesRadius.size() * valuesTheta.size();
                triangleCountMaximumPhi = stepsRadius * stepsTheta * 2;
                if (minimumRadiusZero == true) {
                    vertexCountMaximumPhi -= valuesTheta.size() - 1;
                    triangleCountMaximumPhi -= stepsTheta;
                }

                // Minimum phi
                if (samePhi == false) {
                    vertexCountMinimumPhi = vertexCountMaximumPhi;
                    triangleCountMinimumPhi = triangleCountMaximumPhi;
                }
            }

            // Compute vertex and triangle count (maximum and minimum theta)
            uint64_t vertexCountMaximumTheta = 0;
            uint64_t triangleCountMaximumTheta = 0;
            uint64_t vertexCountMinimumTheta = 0;
            uint64_t triangleCountMinimumTheta = 0;
            if (stepsRadius > 0 && stepsPhi > 0) {
                const size_t newPhiValues = (fullPhi == true) ? valuesPhi.size() - 1 : valuesPhi.size();
                uint64_t vertexCountTheta = valuesRadius.size() * newPhiValues;
                uint64_t triangleCountTheta = stepsRadius * stepsPhi * 2;
                if (minimumRadiusZero == true) {
                    vertexCountTheta -= newPhiValues - 1;
                    triangleCountTheta -= stepsPhi;
                }

                // Maximum theta
                if (maximumThetaPi == false) {
                    vertexCountMaximumTheta = vertexCountTheta;
                    triangleCountMaximumTheta = triangleCountTheta;
                }

                // Minimum theta
                if (sameTheta == false && minimumThetaZero == false) {
                    vertexCountMinimumTheta = vertexCountTheta;
                    triangleCountMinimumTheta = triangleCountTheta;
                }
            }

            // Compute total vertex and triangle count
            uint64_t vertexCount = vertexCountMaximumRadius + vertexCountMinimumRadius + vertexCountMaximumPhi + vertexCountMinimumPhi + vertexCountMaximumTheta + vertexCountMinimumTheta;
            uint64_t triangleCount = triangleCountMaximumRadius + triangleCountMinimumRadius + triangleCountMaximumPhi + triangleCountMinimumPhi + triangleCountMaximumTheta + triangleCountMinimumTheta;

            // Allocate buffers
            mesh.m_vertexBuffer.resize(Eigen::NoChange, vertexCount);
            mesh.m_normalBuffer.resize(Eigen::NoChange, vertexCount);
            mesh.m_indexBuffer.resize(3, triangleCount);
            IndexType currentVertexIndex = 0;
            IndexType currentTriangleIndex = 0;

            // Draw surfaces for maximum and minimum parameter (m=0 -> minimum, m=1 -> maximum)
            for (int m = 0; m <= 1; m++) {
                const bool isMinimumSurface = (m == 0);
                const bool isMaximumSurface = !isMinimumSurface;

                // Draw surface for maximum and minimum radius
                if ((isMinimumSurface && triangleCountMinimumRadius > 0) || (isMaximumSurface && triangleCountMaximumRadius > 0)) {
                    const double& radius = (isMinimumSurface) ? minimumRadius : maximumRadius;

                    // Create vertices with normals
                    SphericalCoordinates coordinates(radius, 0.0, 0.0);
                    const size_t minimumThetaZeroVertexIndex = currentVertexIndex;
                    if (minimumThetaZero == true)
                        addVertexWithNormal(mesh, currentVertexIndex, coordinates.toCartesian(), coordinates.toCartesianFirstDerivativeRadiusNormalized(), isMinimumSurface);
                    const size_t maximumThetaPiVertexIndex = currentVertexIndex;
                    if (maximumThetaPi == true) {
                        coordinates.setTheta(maximumTheta);
                        addVertexWithNormal(mesh, currentVertexIndex, coordinates.toCartesian(), coordinates.toCartesianFirstDerivativeRadiusNormalized(), isMinimumSurface);
                    }
                    const size_t endIndexPhi = (fullPhi == true) ? valuesPhi.size() - 2 : valuesPhi.size() - 1;
                    const size_t startIndexTheta = (minimumThetaZero == true) ? 1 : 0;
                    const size_t endIndexTheta = (maximumThetaPi == true) ? valuesTheta.size() - 2 : valuesTheta.size() - 1;
                    const IndexType gridVertexStartIndex = currentVertexIndex;
                    for (size_t i = 0; i <= endIndexPhi; i++) {
                        coordinates.setPhi(valuesPhi[i]);
                        for (size_t j = startIndexTheta; j <= endIndexTheta; j++) {
                            coordinates.setTheta(valuesTheta[j]);
                            addVertexWithNormal(mesh, currentVertexIndex, coordinates.toCartesian(), coordinates.toCartesianFirstDerivativeRadiusNormalized(), isMinimumSurface);
                        }
                    }

                    // Create triangles
                    const size_t thetaIndexRange = endIndexTheta - startIndexTheta;
                    for (size_t i = 0; i < stepsPhi; i++) {
                        // Rectangular cells
                        for (size_t j = 0; j < thetaIndexRange; j++) {
                            const IndexType index0 = gridVertexStartIndex + (thetaIndexRange + 1) * i + j;
                            const IndexType index3 = (fullPhi == true && i + 1 == stepsPhi) ? gridVertexStartIndex + j : index0 + (thetaIndexRange + 1);
                            addQuad(mesh, currentTriangleIndex, index0, index0 + 1, index3 + 1, index3, isMinimumSurface);
                        }

                        // Triangle cells (theta-min = 0)
                        if (minimumThetaZero == true) {
                            const IndexType index1 = gridVertexStartIndex + (thetaIndexRange + 1) * i;
                            const IndexType index2 = (fullPhi == true && i + 1 == stepsPhi) ? gridVertexStartIndex : index1 + (thetaIndexRange + 1);
                            addTriangle(mesh, currentTriangleIndex, minimumThetaZeroVertexIndex, index1, index2, isMinimumSurface);
                        }

                        // Triangle cells (theta-max = pi)
                        if (maximumThetaPi == true) {
                            const IndexType index2 = gridVertexStartIndex + (thetaIndexRange + 1) * i + thetaIndexRange;
                            const IndexType index1 = (fullPhi == true && i + 1 == stepsPhi) ? gridVertexStartIndex + thetaIndexRange : index2 + (thetaIndexRange + 1);
                            addTriangle(mesh, currentTriangleIndex, maximumThetaPiVertexIndex, index1, index2, isMinimumSurface);
                        }
                    }
                }

                // Draw surface for maximum and minimum phi
                if ((isMinimumSurface && triangleCountMinimumPhi > 0) || (isMaximumSurface && triangleCountMaximumPhi > 0)) {
                    const double& phi = (isMinimumSurface) ? minimumPhi : maximumPhi;

                    // Create vertices with normals
                    SphericalCoordinates coordinates(0.0, phi, 0.0);
                    const size_t minimumRadiusZeroVertexIndex = currentVertexIndex;
                    if (minimumRadiusZero == true)
                        addVertexWithNormal(mesh, currentVertexIndex, coordinates.toCartesian(), coordinates.toCartesianFirstDerivativePhiNormalized(), isMinimumSurface);
                    const size_t startIndexRadius = (minimumRadiusZero == true) ? 1 : 0;
                    const IndexType gridVertexStartIndex = currentVertexIndex;
                    for (size_t i = startIndexRadius; i < valuesRadius.size(); i++) {
                        coordinates.setRadius(valuesRadius[i]);
                        for (size_t j = 0; j < valuesTheta.size(); j++) {
                            coordinates.setTheta(valuesTheta[j]);
                            addVertexWithNormal(mesh, currentVertexIndex, coordinates.toCartesian(), coordinates.toCartesianFirstDerivativePhiNormalized(), isMinimumSurface);
                        }
                    }

                    // Create triangles
                    const size_t radiusIndexRange = valuesRadius.size() - 1 - startIndexRadius;
                    for (size_t i = 0; i < stepsTheta; i++) {
                        // Rectangular cells
                        for (size_t j = 0; j < radiusIndexRange; j++) {
                            const IndexType index0 = gridVertexStartIndex + valuesTheta.size() * j + i;
                            const IndexType index1 = index0 + valuesTheta.size();
                            addQuad(mesh, currentTriangleIndex, index0, index1, index1 + 1, index0 + 1, isMinimumSurface);
                        }

                        // Triangle cells (r-min = 0)
                        if (minimumRadiusZero == true) {
                            const IndexType index1 = gridVertexStartIndex + i;
                            addTriangle(mesh, currentTriangleIndex, minimumRadiusZeroVertexIndex, index1, index1 + 1, isMinimumSurface);
                        }
                    }
                }

                // Draw surface for maximum and minimum theta
                if ((isMinimumSurface && triangleCountMinimumTheta > 0) || (isMaximumSurface && triangleCountMaximumTheta > 0)) {
                    const double& theta = (isMinimumSurface) ? minimumTheta : maximumTheta;

                    // Create vertices with normals
                    SphericalCoordinates coordinates(0.0, 0.0, theta);
                    const size_t minimumRadiusZeroVertexIndex = currentVertexIndex;
                    if (minimumRadiusZero == true)
                        addVertexWithNormal(mesh, currentVertexIndex, coordinates.toCartesian(), coordinates.toCartesianFirstDerivativeThetaNormalized(), isMinimumSurface);
                    const size_t endIndexPhi = (fullPhi == true) ? valuesPhi.size() - 2 : valuesPhi.size() - 1;
                    const size_t startIndexRadius = (minimumRadiusZero == true) ? 1 : 0;
                    const IndexType gridVertexStartIndex = currentVertexIndex;
                    for (size_t i = 0; i <= endIndexPhi; i++) {
                        coordinates.setPhi(valuesPhi[i]);
                        for (size_t j = startIndexRadius; j < valuesRadius.size(); j++) {
                            coordinates.setRadius(valuesRadius[j]);
                            addVertexWithNormal(mesh, currentVertexIndex, coordinates.toCartesian(), coordinates.toCartesianFirstDerivativeThetaNormalized(), isMinimumSurface);
                        }
                    }

                    // Create triangles
                    const size_t radiusIndexRange = valuesRadius.size() - 1 - startIndexRadius;
                    for (size_t i = 0; i < stepsPhi; i++) {
                        // Rectangular cells
                        for (size_t j = 0; j < radiusIndexRange; j++) {
                            const IndexType index0 = gridVertexStartIndex + (radiusIndexRange + 1) * i + j;
                            const IndexType index1 = (fullPhi == true && i + 1 == stepsPhi) ? gridVertexStartIndex + j : index0 + (radiusIndexRange + 1);
                            addQuad(mesh, currentTriangleIndex, index0, index1, index1 + 1, index0 + 1, isMinimumSurface);
                        }

                        // Triangle cells (radius-min = 0)
                        if (minimumRadiusZero == true) {
                            const IndexType index0 = gridVertexStartIndex + (radiusIndexRange + 1) * i;
                            const IndexType index1 = (fullPhi == true && i + 1 == stepsPhi) ? gridVertexStartIndex : index0 + (radiusIndexRange + 1);
                            addTriangle(mesh, currentTriangleIndex, minimumRadiusZeroVertexIndex, index1, index0, isMinimumSurface);
                        }
                    }
                }
            }

            // Pass back created mesh
            return mesh;
        }

        //! Creates a sphere in 3D space (uses default spherical coordinates \ref SphericalCoordinates)
        /*!
         * \param [in] radius Outer radius \f$ r \f$, constraint: \f$ r > 0 \f$
         * \param [in] stepsPhi Step count \f$ \varphi_{step} \f$ for discretizing \f$ \varphi \f$, constraint: \f$ \varphi_{step} > 2 \f$
         * \param [in] stepsTheta Step count \f$ \vartheta_{step} \f$ for discretizing \f$ \vartheta \f$, constraint: \f$ \vartheta_{step} > 1 \f$
         * \return Created mesh (empty mesh in case of an error, e.g. invalid input)
         */
        static inline CGMesh createSphere(const double& radius, const uint64_t& stepsPhi, const uint64_t& stepsTheta)
        {
            return createSphereSlice(radius, radius, 0, 0.0, 2.0 * M_PI, stepsPhi, 0.0, M_PI, stepsTheta);
        }

        //! Creates a cone in 3D space (uses default cylindrical coordinates \ref CylindricalCoordinates)
        /*!
         * \param [in] radius Bottom radius \f$ r \f$, constraint: \f$ r > 0 \f$
         * \param [in] stepsPhi Step count \f$ \varphi_{step} \f$ for discretizing \f$ \varphi \f$, constraint: \f$ \varphi_{step} > 2 \f$
         * \param [in] minimumZ Minimum \f$ z_{min} \f$ coordinate, constraint: none
         * \param [in] maximumZ Maximum \f$ z_{max} \f$ coordinate, constraint: \f$ z_{max} > z_{min} \f$
         * \param [in] fillBase If `true` the base will be filled, otherwise it remains open
         * \return Created mesh (empty mesh in case of an error, e.g. invalid input)
         */
        static inline CGMesh createCone(const double& radius, const uint64_t& stepsPhi, const double& minimumZ, const double& maximumZ, const bool& fillBase)
        {
            // Initialize mesh
            CGMesh mesh;

            // Check inputs
            if (radius <= 0.0 || stepsPhi < 3 || maximumZ <= minimumZ) {
                assert(false);
                return mesh;
            }

            // Compute step size
            const double deltaPhi = 2.0 * M_PI / stepsPhi;

            // Compute discrete coordinate values
            memory::SmartVector<double, 128, std::allocator<double>> valuesPhi(stepsPhi);
            for (size_t i = 0; i < valuesPhi.size(); i++)
                valuesPhi[i] = i * deltaPhi;

            // Compute total vertex and triangle count
            uint64_t vertexCount = 1 + stepsPhi;
            uint64_t triangleCount = stepsPhi;
            if (fillBase == true) {
                vertexCount *= 2;
                triangleCount *= 2;
            }

            // Allocate buffers
            mesh.m_vertexBuffer.resize(Eigen::NoChange, vertexCount);
            mesh.m_normalBuffer.resize(Eigen::NoChange, vertexCount);
            mesh.m_indexBuffer.resize(3, triangleCount);
            IndexType currentVertexIndex = 0;
            IndexType currentTriangleIndex = 0;

            // Pre-compute helpers
            const double coneHeight = maximumZ - minimumZ;
            const double shellMantleLength = sqrt(radius * radius + coneHeight * coneHeight);
            const Eigen::Vector3d shellTopVertex = Eigen::Vector3d(0.0, 0.0, maximumZ);

            // Generate shell
            // --------------
            // Add vertices of shell
            const IndexType shellTopVertexIndex = currentVertexIndex;
            addVertexWithNormal(mesh, currentVertexIndex, shellTopVertex, Eigen::Vector3d::UnitZ());
            const IndexType shellStartVertexIndex = currentVertexIndex;
            CylindricalCoordinates coordinates(radius, 0.0, minimumZ);
            for (size_t i = 0; i < valuesPhi.size(); i++) {
                coordinates.setPhi(valuesPhi[i]);
                const Eigen::Vector3d shellBottomVertex = coordinates.toCartesian();
                const Eigen::Vector3d shellBottomTangent = coordinates.toCartesianFirstDerivativePhiNormalized();
                const Eigen::Vector3d shellBottomNormal = shellBottomTangent.cross(shellTopVertex - shellBottomVertex) / shellMantleLength;
                addVertexWithNormal(mesh, currentVertexIndex, shellBottomVertex, shellBottomNormal);
            }

            // Add triangles of shell
            for (size_t i = 0; i < stepsPhi; i++) {
                const IndexType index0 = shellStartVertexIndex + i;
                const IndexType index1 = (i + 1 == stepsPhi) ? shellStartVertexIndex : index0 + 1;
                addTriangle(mesh, currentTriangleIndex, index0, index1, shellTopVertexIndex);
            }

            // Generate base
            // -------------
            if (fillBase == true) {
                const Eigen::Vector3d baseNormal = -Eigen::Vector3d::UnitZ();

                // Add vertices of base
                const IndexType baseCenterVertexIndex = currentVertexIndex;
                addVertexWithNormal(mesh, currentVertexIndex, Eigen::Vector3d(0.0, 0.0, minimumZ), baseNormal);
                const IndexType baseStartVertexIndex = currentVertexIndex;
                for (size_t i = 0; i < valuesPhi.size(); i++)
                    addVertexWithNormal(mesh, currentVertexIndex, mesh.m_vertexBuffer.col(shellStartVertexIndex + i), baseNormal);

                // Add triangles of base
                for (size_t i = 0; i < stepsPhi; i++) {
                    const IndexType index2 = baseStartVertexIndex + i;
                    const IndexType index1 = (i + 1 == stepsPhi) ? baseStartVertexIndex : index2 + 1;
                    addTriangle(mesh, currentTriangleIndex, baseCenterVertexIndex, index1, index2);
                }
            }

            // Pass back created mesh
            return mesh;
        }

        //! Extrudes an outline (curve) along a 3D path (linear segments)
        /*!
         * \note Automatically skips coinciding and collinear waypoints of the path.
         *
         * \param [in] wayPoints List of path waypoints in 3D space (as matrix, each column represents a waypoint), constraint: at least two non-coinciding waypoints (otherwise resulting mesh is empty)
         * \param [in] outline Mesh containing the vertices and per-vertex normals (optional) of the outline (curve) in the **local** waypoint coordinate system (origin = waypoint, positive z-axis = path direction), constraint: at least two vertices (otherwise resulting mesh is empty)
         * \param [in] closeOutline If `true`, the first and last vertex of the outline will be connected (same as for neighboring vertices)
         * \param [in] firstOutlineXAxis Direction of x-axis of outline geometry at **first** waypoint of the path (has to be **normalized** and **perpendicular** to the path) (mandatory input)
         * \param [out] lastOutlineRotation Rotation matrix of outline geometry at **last** waypoint of the path (optional output)
         * \return Created mesh (empty if (a) less than two non-coinciding waypoints or (b) less than two vertices in outline)
         */
        static inline CGMesh extrudeOutlineAlong3DPath(const Eigen::Matrix<double, 3, Eigen::Dynamic>& wayPoints, const CGMesh& outline, const bool& closeOutline, const Eigen::Vector3d& firstOutlineXAxis, Eigen::Matrix3d* const lastOutlineRotation = nullptr)
        {
            // Initialize helpers
            const bool withNormals = outline.m_normalBuffer.cols() > 0 && outline.m_normalBuffer.cols() == outline.m_vertexBuffer.cols();

            // Check inputs
            if (wayPoints.cols() < 2 || outline.m_vertexBuffer.cols() < 2)
                return CGMesh();

            // Pre-process segments of path (connections between waypoints) and compute preliminary index list of waypoints (skip coinciding waypoints)
            Eigen::Matrix<double, 3, Eigen::Dynamic> segments; // Matrix containing directions (columns) from a waypoint to the next waypoint (NORMALIZED)
            segments.resize(Eigen::NoChange, wayPoints.cols() - 1);
            std::vector<Eigen::Index> preliminiaryWayPointIndices; // List of waypoint indices WITHOUT coinciding waypoints
            preliminiaryWayPointIndices.reserve(wayPoints.cols());
            preliminiaryWayPointIndices.push_back(0); // First point is ALWAYS part of the path
            for (Eigen::Index i = 0; i < segments.cols(); i++) {
                const Eigen::Vector3d toNextWayPoint = wayPoints.col(i + 1) - wayPoints.col(i);
                const double distance = toNextWayPoint.norm();
                if (core::isZero(distance) == false) {
                    preliminiaryWayPointIndices.push_back(i + 1);
                    segments.col(i) = toNextWayPoint / distance;
                }
            }
            if (preliminiaryWayPointIndices.size() < 2)
                return CGMesh();

            // Compute final index list of waypoints (skip collinear waypoints)
            std::vector<Eigen::Index> wayPointIndices; // List of waypoint indices WITHOUT coinciding waypoints and WITHOUT collinear waypoints
            wayPointIndices.reserve(preliminiaryWayPointIndices.size());
            wayPointIndices.push_back(preliminiaryWayPointIndices.front()); // First point is ALWAYS part of the path
            for (size_t i = 1; i + 1 < preliminiaryWayPointIndices.size(); i++)
                if (core::isZero(1.0 - segments.col(preliminiaryWayPointIndices[i] - 1).dot(segments.col(preliminiaryWayPointIndices[i + 1] - 1))) == false)
                    wayPointIndices.push_back(preliminiaryWayPointIndices[i]);
            wayPointIndices.push_back(preliminiaryWayPointIndices.back()); // Last point is ALWAYS part of the path

            // Initialize mesh
            CGMesh mesh;

            // Create vertex and normal buffer
            const uint64_t verticesPerOutline = outline.m_vertexBuffer.cols();
            const uint64_t vertexCount = verticesPerOutline * wayPointIndices.size();
            mesh.m_vertexBuffer.resize(Eigen::NoChange, vertexCount);
            if (withNormals)
                mesh.m_normalBuffer.resize(Eigen::NoChange, vertexCount);
            Eigen::Matrix3d previousOutlineRotationMatrix, currentOutlineRotationMatrix;
            for (size_t i = 0; i < wayPointIndices.size(); i++) {
                const uint64_t startVertexIndex = verticesPerOutline * i;

                // Compute outline z-axis
                if (i == 0)
                    currentOutlineRotationMatrix.col(2) = segments.col(wayPointIndices[1] - 1); // Use direction of first segment
                else if (i + 1 == wayPointIndices.size())
                    currentOutlineRotationMatrix.col(2) = segments.col(wayPointIndices.back() - 1); // Use direction of last segment
                else {
                    const Eigen::Vector3d currentOutlineZAxis = segments.col(wayPointIndices[i] - 1) + segments.col(wayPointIndices[i + 1] - 1);
                    const double currentOutlineZAxisLength = currentOutlineZAxis.norm();
                    if (core::isZero(currentOutlineZAxisLength) == false)
                        currentOutlineRotationMatrix.col(2) = currentOutlineZAxis / currentOutlineZAxisLength;
                    else // Path "flips" (goes in other direction) -> use previous segment direction as normal
                        currentOutlineRotationMatrix.col(2) = segments.col(wayPointIndices[i] - 1);
                }

                // Compute outline x-axis and y-axis
                if (i == 0) {
                    currentOutlineRotationMatrix.col(0) = firstOutlineXAxis;
                    currentOutlineRotationMatrix.col(1) = currentOutlineRotationMatrix.col(2).cross(currentOutlineRotationMatrix.col(0));
                } else {
                    // Project previous outline x-axis to current intersection plane normal
                    currentOutlineRotationMatrix.col(0) = previousOutlineRotationMatrix.col(0) - currentOutlineRotationMatrix.col(2).dot(previousOutlineRotationMatrix.col(0)) * currentOutlineRotationMatrix.col(2);
                    currentOutlineRotationMatrix.col(1) = previousOutlineRotationMatrix.col(1) - currentOutlineRotationMatrix.col(2).dot(previousOutlineRotationMatrix.col(1)) * currentOutlineRotationMatrix.col(2);
                    const double currentOutlineXAxisLength = currentOutlineRotationMatrix.col(0).norm();
                    const double currentOutlineYAxisLength = currentOutlineRotationMatrix.col(1).norm();
                    if (currentOutlineXAxisLength >= currentOutlineYAxisLength) {
                        // Use x-axis as reference for better numerical condition
                        currentOutlineRotationMatrix.col(0) /= currentOutlineXAxisLength;
                        currentOutlineRotationMatrix.col(1) = currentOutlineRotationMatrix.col(2).cross(currentOutlineRotationMatrix.col(0));
                    } else {
                        // Use y-axis as reference for better numerical condition
                        currentOutlineRotationMatrix.col(1) /= currentOutlineYAxisLength;
                        currentOutlineRotationMatrix.col(0) = currentOutlineRotationMatrix.col(1).cross(currentOutlineRotationMatrix.col(2));
                    }
                }

                // Compute vertex buffer (rotation and translation)
                mesh.m_vertexBuffer.block(0, startVertexIndex, 3, verticesPerOutline) = currentOutlineRotationMatrix * outline.m_vertexBuffer;
                for (uint64_t j = startVertexIndex; j < startVertexIndex + verticesPerOutline; j++)
                    mesh.m_vertexBuffer.col(j) += wayPoints.col(wayPointIndices[i]);

                // Compute normal buffer (rotation)
                if (withNormals == true)
                    mesh.m_normalBuffer.block(0, startVertexIndex, 3, verticesPerOutline) = currentOutlineRotationMatrix * outline.m_normalBuffer;

                // Prepare for next step
                if (i + 1 < wayPointIndices.size())
                    previousOutlineRotationMatrix = currentOutlineRotationMatrix;
            }

            // Write last outline rotation to output variable if requested
            if (lastOutlineRotation != nullptr)
                *lastOutlineRotation = currentOutlineRotationMatrix;

            // Create index buffer
            const uint64_t trianglesPerSegment = (closeOutline == true) ? 2 * verticesPerOutline : 2 * (verticesPerOutline - 1);
            const uint64_t triangleCount = trianglesPerSegment * (wayPointIndices.size() - 1);
            mesh.m_indexBuffer.resize(3, triangleCount);
            IndexType currentTriangleIndex = 0;
            for (size_t i = 0; i + 1 < wayPointIndices.size(); i++) {
                const IndexType startVertexIndex = verticesPerOutline * i;
                for (uint64_t j = 0; j + 1 < verticesPerOutline; j++) {
                    const IndexType index0 = startVertexIndex + j;
                    const IndexType index1 = index0 + 1;
                    addQuad(mesh, currentTriangleIndex, index0, index1, index1 + verticesPerOutline, index0 + verticesPerOutline);
                }
                if (closeOutline == true) {
                    const IndexType& index1 = startVertexIndex;
                    const IndexType index0 = startVertexIndex + verticesPerOutline - 1;
                    addQuad(mesh, currentTriangleIndex, index0, index1, index1 + verticesPerOutline, index0 + verticesPerOutline);
                }
            }

            // Pass back created mesh
            return mesh;
        }

        //! Creates a cylindrical 3D path
        /*!
         * \note Automatically skips coinciding and collinear waypoints of the path.
         *
         * \param [in] wayPoints List of path waypoints in 3D space (as matrix, each column represents a waypoint), constraint: at least two non-coinciding waypoints (otherwise resulting mesh is empty)
         * \param [in] radius Outer radius \f$ r \f$, constraint: \f$ r > 0 \f$
         * \param [in] stepsPhi Step count \f$ \varphi_{step} \f$ for discretizing \f$ \varphi \f$, constraint: \f$ \varphi_{step} > 2 \f$
         * \param [in] fillFlatCaps If `true`, the start and the end of the path is filled with a flat cap (circle)
         * \param [in] fillRoundCaps If `true`, the start and the end of the path is filled with a round cap (sphere)
         * \return Created mesh (empty if less than two non-coinciding waypoints)
         */
        static inline CGMesh createCylindrical3DPath(const Eigen::Matrix<double, 3, Eigen::Dynamic>& wayPoints, const double& radius, const uint64_t& stepsPhi, const bool& fillFlatCaps, const bool& fillRoundCaps)
        {
            // Check inputs
            if (radius <= 0.0 || stepsPhi < 3) {
                assert(false);
                return CGMesh();
            }

            // Try to obtain axis of first path segment
            bool allWayPointsCoincide = true;
            Eigen::Vector3d firstSegmentAxis;
            for (Eigen::Index i = 0; i + 1 < wayPoints.cols(); i++) {
                const Eigen::Vector3d toNextWayPoint = wayPoints.col(i + 1) - wayPoints.col(i);
                const double distance = toNextWayPoint.norm();
                if (core::isZero(distance) == false) {
                    allWayPointsCoincide = false;
                    firstSegmentAxis = toNextWayPoint / distance;
                    break;
                }
            }
            if (allWayPointsCoincide == true)
                return CGMesh();

            // Compute outline
            CGMesh outline;
            outline.m_vertexBuffer.resize(Eigen::NoChange, stepsPhi);
            outline.m_normalBuffer.resize(Eigen::NoChange, stepsPhi);
            CylindricalCoordinates coordinates(radius, 0.0, 0.0);
            const double deltaPhi = 2.0 * M_PI / stepsPhi;
            for (size_t i = 0; i < stepsPhi; i++) {
                coordinates.setPhi(i * deltaPhi);
                outline.m_vertexBuffer.col(i) = coordinates.toCartesian();
                outline.m_normalBuffer.col(i) = coordinates.toCartesianFirstDerivativeRadiusNormalized();
            }

            // Extrude outline
            const Eigen::Vector3d firstOutlineXAxis = core::math::findPerpendicularVector(firstSegmentAxis).normalized();
            Eigen::Matrix3d lastOutlineRotation;
            const CGMesh extrudedOutline = extrudeOutlineAlong3DPath(wayPoints, outline, true, firstOutlineXAxis, &lastOutlineRotation);

            // Special case: no caps -> done
            if (fillFlatCaps == false && fillRoundCaps == false)
                return extrudedOutline;

            // Setup meshes for caps
            CGMesh startCap;
            if (fillFlatCaps == true)
                startCap = createCircle(radius, stepsPhi);
            else
                startCap = createSphereSlice(radius, radius, 0, 0.0, 2.0 * M_PI, stepsPhi, 0.0, 0.5 * M_PI, std::ceil(stepsPhi / 4.0));
            CGMesh endCap = startCap;
            startCap.changeCoordinateSystem(wayPoints.col(0), firstOutlineXAxis, -firstSegmentAxis);
            endCap.changeCoordinateSystem(wayPoints.col(wayPoints.cols() - 1), lastOutlineRotation.col(0), lastOutlineRotation.col(2));

            // Merge meshes
            std::vector<const CGMesh*> subMeshPointerList;
            subMeshPointerList.reserve(3);
            subMeshPointerList.push_back(&extrudedOutline);
            subMeshPointerList.push_back(&startCap);
            subMeshPointerList.push_back(&endCap);
            return CGMeshTools::merge(subMeshPointerList);
        }

        // Volumes
        // -------
    public:
        // Type definitions for marching cubes algorithm
        using MarchingCubesDensityGrid = memory::MultiVector<double, 3, std::allocator<double>>; //!< Type of grid for density data (marching cubes algorithm)
        using MarchingCubesColorGrid = memory::MultiVector<CGMesh::ColorType, 3>; //!< Type of grid for color data (marching cubes algorithm)

        //! Creates a volume using the Marching Cubes algorithm
        /*!
         * \remark Original publication of Marching Cubes algorithm: William E. Lorensen, Harvey E. Cline: Marching Cubes: A high resolution 3D surface construction algorithm. In: Computer Graphics, Vol. 21, Nr. 4, Juli 1987, S. 163–169
         *
         * \remark The algorithm is adapted from http://paulbourke.net/geometry/polygonise/. However, it has been extended to create an indexed mesh with interpolated normals and colors.
         *
         * \param [in] densityGrid Grid containing the density data for each grid point
         * \param [in] densityThreshold Threshold value for the isosurface (density values below this threshold are considered as "empty")
         * \param [in] dimension Dimensions (x, y, z) of the **complete grid** (outer dimension, used for scaling)
         * \param [in] withNormals If `true` the normal buffer is generated from the density gradient
         * \param [in] colorGrid Grid containing the color data for each grid point (optional, must have same size as \p densityGrid)
         * \param [in] interpolateColors If `true` colors are interpolated between the "occupied" grid point and the "empty" grid point of an edge, otherwise the color of the "occupied" point of an edge is used
         * \return Created mesh (empty mesh in case of an error, e.g. invalid input)
         */
        static inline CGMesh createVolumeMarchingCubes(const MarchingCubesDensityGrid& densityGrid, const double& densityThreshold, const Eigen::Vector3d& dimension, const bool& withNormals, const MarchingCubesColorGrid& colorGrid = MarchingCubesColorGrid(), const bool& interpolateColors = false)
        {
            // Check input
            const auto gridSize = densityGrid.size();
            if (gridSize[0] <= 1 || gridSize[1] <= 1 || gridSize[2] <= 1) {
                // Cell-count is zero -> not enough data for creating the mesh
                return CGMesh();
            }

            // Initialize helpers
            const size_t totalCellCount = (gridSize[0] - 1) * (gridSize[1] - 1) * (gridSize[2] - 1); // Total count of cells in the grid
            const Eigen::Vector3d cellDimension(dimension(0) / ((double)gridSize[0] - 1), dimension(1) / ((double)gridSize[1] - 1), dimension(2) / (gridSize[2] - 1)); // Dimension of a single grid cell

            // Determine, if color data should be processed (only if size of grids match)
            const bool withColors = (colorGrid.size() == gridSize);

            // Look-up table for cubes
            /*
             * 1. index: 256 possible cube configurations
             * 2. index: 3-tuples of local cell edge indices (0-11) linked to the the triangles of this cube (list of tuples stops with "-1", maximum 5 triangles per cube)
             */
            static const std::array<std::array<int8_t, 16>, 256> cubeLookupTable = { { { { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1 } },
                { { 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1 } },
                { { 3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1 } },
                { { 3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1 } },
                { { 9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1 } },
                { { 9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1 } },
                { { 2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1 } },
                { { 8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1 } },
                { { 9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1 } },
                { { 4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1 } },
                { { 3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1 } },
                { { 1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1 } },
                { { 4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1 } },
                { { 4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1 } },
                { { 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1 } },
                { { 1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1 } },
                { { 5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1 } },
                { { 2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1 } },
                { { 9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1 } },
                { { 2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1 } },
                { { 10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1 } },
                { { 4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1 } },
                { { 5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1 } },
                { { 5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1 } },
                { { 9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1 } },
                { { 1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1 } },
                { { 10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1 } },
                { { 8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1 } },
                { { 2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1 } },
                { { 7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1 } },
                { { 9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1 } },
                { { 2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1 } },
                { { 11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1 } },
                { { 9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1 } },
                { { 5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1 } },
                { { 11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1 } },
                { { 11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1 } },
                { { 1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1 } },
                { { 9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1 } },
                { { 5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1 } },
                { { 2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1 } },
                { { 5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1 } },
                { { 6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1 } },
                { { 3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1 } },
                { { 6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1 } },
                { { 5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1 } },
                { { 1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1 } },
                { { 10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1 } },
                { { 6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1 } },
                { { 1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1 } },
                { { 8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1 } },
                { { 7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1 } },
                { { 3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1 } },
                { { 5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1 } },
                { { 0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1 } },
                { { 9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1 } },
                { { 8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1 } },
                { { 5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1 } },
                { { 0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1 } },
                { { 6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1 } },
                { { 10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1 } },
                { { 10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1 } },
                { { 8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1 } },
                { { 1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1 } },
                { { 3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1 } },
                { { 0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1 } },
                { { 10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1 } },
                { { 3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1 } },
                { { 6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1 } },
                { { 9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1 } },
                { { 8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1 } },
                { { 3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1 } },
                { { 6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1 } },
                { { 10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1 } },
                { { 10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1 } },
                { { 1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1 } },
                { { 2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1 } },
                { { 7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1 } },
                { { 7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1 } },
                { { 2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1 } },
                { { 1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1 } },
                { { 11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1 } },
                { { 8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1 } },
                { { 0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1 } },
                { { 7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1 } },
                { { 10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1 } },
                { { 2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1 } },
                { { 6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1 } },
                { { 7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1 } },
                { { 2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1 } },
                { { 1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1 } },
                { { 10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1 } },
                { { 10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1 } },
                { { 0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1 } },
                { { 7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1 } },
                { { 6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1 } },
                { { 8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1 } },
                { { 6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1 } },
                { { 4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1 } },
                { { 10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1 } },
                { { 8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1 } },
                { { 1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1 } },
                { { 8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1 } },
                { { 10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1 } },
                { { 4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1 } },
                { { 10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1 } },
                { { 5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1 } },
                { { 11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1 } },
                { { 9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1 } },
                { { 6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1 } },
                { { 7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1 } },
                { { 3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1 } },
                { { 7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1 } },
                { { 9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1 } },
                { { 3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1 } },
                { { 6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1 } },
                { { 9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1 } },
                { { 1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1 } },
                { { 4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1 } },
                { { 7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1 } },
                { { 6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1 } },
                { { 3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1 } },
                { { 0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1 } },
                { { 6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1 } },
                { { 0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1 } },
                { { 11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1 } },
                { { 6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1 } },
                { { 5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1 } },
                { { 9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1 } },
                { { 1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1 } },
                { { 1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1 } },
                { { 10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1 } },
                { { 0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1 } },
                { { 5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1 } },
                { { 10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1 } },
                { { 11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1 } },
                { { 9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1 } },
                { { 7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1 } },
                { { 2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1 } },
                { { 8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1 } },
                { { 9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1 } },
                { { 9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1 } },
                { { 1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1 } },
                { { 9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1 } },
                { { 9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1 } },
                { { 5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1 } },
                { { 0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1 } },
                { { 10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1 } },
                { { 2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1 } },
                { { 0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1 } },
                { { 0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1 } },
                { { 9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1 } },
                { { 5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1 } },
                { { 3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1 } },
                { { 5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1 } },
                { { 8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1 } },
                { { 9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1 } },
                { { 1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1 } },
                { { 3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1 } },
                { { 4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1 } },
                { { 9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1 } },
                { { 11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1 } },
                { { 11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1 } },
                { { 2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1 } },
                { { 9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1 } },
                { { 3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1 } },
                { { 1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1 } },
                { { 4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1 } },
                { { 4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1 } },
                { { 3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1 } },
                { { 3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1 } },
                { { 0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1 } },
                { { 9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1 } },
                { { 1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { 0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } },
                { { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } } } };

            // Data type for edges in the data grid
            class Edge {
            public:
                bool m_processed = false; // Flag indicating, if this edge has already been processed (i.e. it will create a corresponding vertex in the output mesh)
                IndexType m_vertexIndex = 0; // Index of corresponding output vertex in vertex buffer (unset if not processed)
            };

            // Allocate buffer for edges
            /*
             * Each point in the grid has exactly one connected edge in positive x-direction [][][][0], y-direction [][][][1] and z-direction [][][][2].
             * Note that the very last grid cell in each dimensions might not have all 3 connected edges (as they would lie outside of the grid).
             * However, these are not processed anyway and only account for a very small amount of extra (unused) memory.
             */
            memory::MultiVector<Edge, 4, std::allocator<Edge> /* <- no Eigen members */> edgeBuffer(gridSize[0], gridSize[1], gridSize[2], 3);
            const size_t totalEdgeCount = edgeBuffer.elementCount(); // Total count of edges in the grid (contains unused "extra" cells lying outside of the grid)

            // Data type for density gradients
            class DensityGradient {
            public:
                bool m_processed = false; // Flag indicating, if this density gradient has already been processed
                Eigen::Vector3d m_gradient = Eigen::Vector3d::Zero(); // Density gradient at a certain location in the grid (unset if not processed)
            };

            // Allocate buffer for density gradient
            memory::MultiVector<DensityGradient, 3, std::allocator<DensityGradient> /* <- single Eigen member is not vectorizable */> densityGradientBuffer;
            if (withNormals == true)
                densityGradientBuffer.resize(gridSize);

            // Allocate buffer for vertices, normals, colors and triangles
            std::vector<Eigen::Vector3d /* <- not vectorizable */> vertexBuffer;
            vertexBuffer.reserve(totalEdgeCount); // Each edge may create a vertex in the output mesh
            std::vector<Eigen::Vector3d /* <- not vectorizable */> normalBuffer;
            if (withNormals == true)
                normalBuffer.reserve(totalEdgeCount); // Each edge may create a (per-vertex) normal in the output mesh
            std::vector<CGMesh::ColorType, Eigen::aligned_allocator<CGMesh::ColorType> /* <- vectorizable */> colorBuffer;
            if (withColors == true)
                colorBuffer.reserve(totalEdgeCount); // Each edge may create a (per-vertex) color in the output mesh
            std::vector<Eigen::Matrix<IndexType, 3, 1> /* <- not vectorizable */> indexBuffer;
            indexBuffer.reserve(totalCellCount * 5); // Maximum 5 triangles per grid cell

            // Iterate through grid cells
            for (size_t x = 0; x + 1 < gridSize[0]; x++) {
                for (size_t y = 0; y + 1 < gridSize[1]; y++) {
                    for (size_t z = 0; z + 1 < gridSize[2]; z++) {
                        // Get indices of cell points and edges
                        /*
                         *           Z
                         *           |
                         *           |      4
                         *          4x---------------x5
                         *          /|              /|
                         *        7/ |            5/ |
                         *        /  |   6        /  |
                         *      7x---------------x6  |9
                         *       |  8|           |   |
                         *       |   |           |   |
                         *       |   |      0    |10 |
                         *     11|  0x-----------|---x1----Y
                         *       |  /            |  /
                         *       | /3            | /1
                         *       |/              |/
                         *      3x---------------x2
                         *      /       2
                         *     /
                         *    X
                         */

                        // Location of the local grid cell point in the global grid (x, y, z)
                        const std::array<std::array<size_t, 3>, 8 /* <- 8 corner points per grid cell */> cellPointsGridLocation{ {
                            { { x, y, z } }, // Point 0
                            { { x, y + 1, z } }, // Point 1
                            { { x + 1, y + 1, z } }, // Point 2
                            { { x + 1, y, z } }, // Point 3
                            { { x, y, z + 1 } }, // Point 4
                            { { x, y + 1, z + 1 } }, // Point 5
                            { { x + 1, y + 1, z + 1 } }, // Point 6
                            { { x + 1, y, z + 1 } }, // Point 7
                        } };

                        // Location of the local grid cell edge in the edge buffer (x, y, z, direction)
                        const std::array<std::array<size_t, 4>, 12 /* <- 12 edges per grid cell */> cellEdgesGridLocation{ {
                            { { x, y, z, 1 } }, // Edge 0 (point 0, positive y-direction)
                            { { x, y + 1, z, 0 } }, // Edge 1 (point 1, positive x-direction)
                            { { x + 1, y, z, 1 } }, // Edge 2 (point 3, positive y-direction)
                            { { x, y, z, 0 } }, // Edge 3 (point 0 positive x-direction)
                            { { x, y, z + 1, 1 } }, // Edge 4 (point 4, positive y-direction)
                            { { x, y + 1, z + 1, 0 } }, // Edge 5 (point 5, positive x-direction)
                            { { x + 1, y, z + 1, 1 } }, // Edge 6 (point 7, positive y-direction)
                            { { x, y, z + 1, 0 } }, // Edge 7 (point 4, positive x-direction)
                            { { x, y, z, 2 } }, // Edge 8 (point 0, positive z-direction)
                            { { x, y + 1, z, 2 } }, // Edge 9 (point 1, positive z-direction)
                            { { x + 1, y + 1, z, 2 } }, // Edge 10 (point 2, positive z-direction)
                            { { x + 1, y, z, 2 } } // Edge 11 (point 3, positive z-direction)
                        } };

                        // Indices of linked local cell points for each local cell edge
                        const std::array<std::array<uint8_t, 2>, 12 /* <- 12 edges per grid cell */> cellEdgesPointIndices{ {
                            { { 0, 1 } }, // Edge 0
                            { { 1, 2 } }, // Edge 1
                            { { 2, 3 } }, // Edge 2
                            { { 3, 0 } }, // Edge 3
                            { { 4, 5 } }, // Edge 4
                            { { 5, 6 } }, // Edge 5
                            { { 6, 7 } }, // Edge 6
                            { { 7, 4 } }, // Edge 7
                            { { 0, 4 } }, // Edge 8
                            { { 1, 5 } }, // Edge 9
                            { { 2, 6 } }, // Edge 10
                            { { 3, 7 } } // Edge 11
                        } };

                        // Determine the variant of the cube (out of the 256 possibilities)
                        uint8_t cubeIndex = 0;
                        if (densityGrid(cellPointsGridLocation[0]) < densityThreshold)
                            cubeIndex |= 1;
                        if (densityGrid(cellPointsGridLocation[1]) < densityThreshold)
                            cubeIndex |= 2;
                        if (densityGrid(cellPointsGridLocation[2]) < densityThreshold)
                            cubeIndex |= 4;
                        if (densityGrid(cellPointsGridLocation[3]) < densityThreshold)
                            cubeIndex |= 8;
                        if (densityGrid(cellPointsGridLocation[4]) < densityThreshold)
                            cubeIndex |= 16;
                        if (densityGrid(cellPointsGridLocation[5]) < densityThreshold)
                            cubeIndex |= 32;
                        if (densityGrid(cellPointsGridLocation[6]) < densityThreshold)
                            cubeIndex |= 64;
                        if (densityGrid(cellPointsGridLocation[7]) < densityThreshold)
                            cubeIndex |= 128;

                        // Create triangles
                        for (size_t i = 0; cubeLookupTable[cubeIndex][i] != -1; i += 3) {
                            // Create new triangle
                            indexBuffer.resize(indexBuffer.size() + 1);

                            // Process edges (or reuse old data)
                            for (size_t e = 0; e < 3; e++) {
                                // Get edge
                                const int8_t& cellEdgeIndex = cubeLookupTable[cubeIndex][i + e]; // Local cell index of current edge
                                Edge& edge = edgeBuffer(cellEdgesGridLocation[cellEdgeIndex]); // Reference to current edge

                                // Check, if this edge has already been processed
                                if (edge.m_processed == false) {
                                    // ...no -> compute
                                    edge.m_vertexIndex = vertexBuffer.size(); // New created vertex
                                    const std::array<size_t, 3>& locationFirstPoint = cellPointsGridLocation[cellEdgesPointIndices[cellEdgeIndex][0]]; // Location of first point linked to edge in global grid
                                    const std::array<size_t, 3>& locationSecondPoint = cellPointsGridLocation[cellEdgesPointIndices[cellEdgeIndex][1]]; // Location of second point linked to edge in global grid
                                    const double& densityFirstPoint = densityGrid(locationFirstPoint);
                                    const double& densitySecondPoint = densityGrid(locationSecondPoint);
                                    const double x = (densityThreshold - densityFirstPoint) / (densitySecondPoint - densityFirstPoint); // Interpolation variable

                                    // Compute vertex (linear interpolation between both points linked to the edge)
                                    const Eigen::Vector3d positionFirstPoint(locationFirstPoint[0] * cellDimension(0), locationFirstPoint[1] * cellDimension(1), locationFirstPoint[2] * cellDimension(2));
                                    const Eigen::Vector3d positionSecondPoint(locationSecondPoint[0] * cellDimension(0), locationSecondPoint[1] * cellDimension(1), locationSecondPoint[2] * cellDimension(2));
                                    if (fabs(densityThreshold - densityFirstPoint) < 1e-6)
                                        vertexBuffer.push_back(positionFirstPoint);
                                    else if (fabs(densityThreshold - densitySecondPoint) < 1e-6)
                                        vertexBuffer.push_back(positionSecondPoint);
                                    else if (fabs(densityFirstPoint - densitySecondPoint) < 1e-6)
                                        vertexBuffer.push_back(positionFirstPoint);
                                    else
                                        vertexBuffer.push_back(positionFirstPoint + x * (positionSecondPoint - positionFirstPoint));

                                    // Compute normal (linear interpolation between density gradients of both points linked to the edge)
                                    if (withNormals == true) {
                                        DensityGradient& densityGradientFirstPoint = densityGradientBuffer(locationFirstPoint); // Density gradients of first point
                                        DensityGradient& densityGradientSecondPoint = densityGradientBuffer(locationSecondPoint); // Density gradients of second point

                                        // Computation of the density gradient
                                        auto computeGradient = [&](const std::array<size_t, 3>& gradientLocation, Eigen::Vector3d& gradient) {
                                            for (Eigen::Index d = 0; d < 3; d++) { // Iterate over dimensions
                                                // Check type of gradient
                                                double stepSize = cellDimension(d);
                                                std::array<size_t, 3> leftLocation = gradientLocation;
                                                std::array<size_t, 3> rightLocation = gradientLocation;
                                                if (gradientLocation[d] == 0)
                                                    rightLocation[d]++; // "right" gradient
                                                else if (gradientLocation[d] + 1 == gridSize[d])
                                                    leftLocation[d]--; // "left" gradient
                                                else {
                                                    // "centered" gradient
                                                    leftLocation[d]--;
                                                    rightLocation[d]++;
                                                    stepSize *= 2.0;
                                                }
                                                gradient(d) = (densityGrid(rightLocation) - densityGrid(leftLocation)) / stepSize;
                                            }
                                        };
                                        if (densityGradientFirstPoint.m_processed == false) {
                                            computeGradient(locationFirstPoint, densityGradientFirstPoint.m_gradient);
                                            densityGradientFirstPoint.m_processed = true;
                                        }
                                        if (densityGradientSecondPoint.m_processed == false) {
                                            computeGradient(locationSecondPoint, densityGradientSecondPoint.m_gradient);
                                            densityGradientSecondPoint.m_processed = true;
                                        }

                                        // Compute normal from gradient
                                        if (fabs(densityThreshold - densityFirstPoint) < 1e-6)
                                            normalBuffer.push_back(-densityGradientFirstPoint.m_gradient.normalized());
                                        else if (fabs(densityThreshold - densitySecondPoint) < 1e-6)
                                            normalBuffer.push_back(-densityGradientSecondPoint.m_gradient.normalized());
                                        else if (fabs(densityFirstPoint - densitySecondPoint) < 1e-6)
                                            normalBuffer.push_back(-densityGradientFirstPoint.m_gradient.normalized());
                                        else
                                            normalBuffer.push_back((x * (densityGradientFirstPoint.m_gradient - densityGradientSecondPoint.m_gradient) - densityGradientFirstPoint.m_gradient).normalized());
                                    }

                                    // Compute color
                                    if (withColors == true) {
                                        if (interpolateColors == false) {
                                            // No interpolation -> always use point with higher density i.e. the "occupied" grid point
                                            if (densityFirstPoint > densitySecondPoint)
                                                colorBuffer.push_back(colorGrid(locationFirstPoint));
                                            else
                                                colorBuffer.push_back(colorGrid(locationSecondPoint));
                                        } else {
                                            // Interpolation -> compute new color
                                            const CGMesh::ColorType& colorFirstPoint = colorGrid(locationFirstPoint);
                                            const CGMesh::ColorType& colorSecondPoint = colorGrid(locationSecondPoint);
                                            colorBuffer.resize(colorBuffer.size() + 1);
                                            for (Eigen::Index c = 0; c < colorFirstPoint.size(); c++) {
                                                const double firstColorChannel = colorFirstPoint(c);
                                                const double secondColorChannel = colorSecondPoint(c);
                                                colorBuffer.back()(c) = std::round(firstColorChannel + x * (secondColorChannel - firstColorChannel));
                                            }
                                        }
                                    }

                                    // Tag this edge as processed
                                    edge.m_processed = true;
                                }

                                // Use vertex linke to edge for triangle
                                indexBuffer.back()(2 - e) = edge.m_vertexIndex;
                            }
                        }
                    }
                }
            }

            // Setup output mesh
            // -----------------
            CGMesh mesh;

            // Copy vertex buffer
            mesh.m_vertexBuffer.resize(Eigen::NoChange, vertexBuffer.size());
            for (size_t i = 0; i < vertexBuffer.size(); i++)
                mesh.m_vertexBuffer.col(i) = vertexBuffer[i];

            // Copy normal buffer
            mesh.m_normalBuffer.resize(Eigen::NoChange, normalBuffer.size());
            for (size_t i = 0; i < normalBuffer.size(); i++)
                mesh.m_normalBuffer.col(i) = normalBuffer[i];

            // Copy color buffer
            mesh.m_colorBuffer.resize(Eigen::NoChange, colorBuffer.size());
            for (size_t i = 0; i < colorBuffer.size(); i++)
                mesh.m_colorBuffer.col(i) = colorBuffer[i];

            // Copy index buffer
            mesh.m_indexBuffer.resize(3, indexBuffer.size());
            for (size_t i = 0; i < indexBuffer.size(); i++)
                mesh.m_indexBuffer.col(i) = indexBuffer[i];

            // Pass back mesh
            return mesh;
        }

        // Helpers
        // -------
    protected:
        //! Adds the given vertex and normal to the specified mesh
        /*!
         * \param [in,out] mesh The mesh to which the data should be added to
         * \param [in,out] currentVertexIndex The index of the current vertex in the vertex buffer (will be incremented)
         * \param [in] vertex The vertex to add
         * \param [in] normal The per-vertex normal for this vertex
         * \param [in] flipNormal If `true`, the vertex normal is flipped (reverse direction)
         */
        static inline void addVertexWithNormal(CGMesh& mesh, IndexType& currentVertexIndex, const Eigen::Vector3d& vertex, const Eigen::Vector3d& normal, const bool& flipNormal = false)
        {
            mesh.m_vertexBuffer.col(currentVertexIndex) = vertex;
            if (flipNormal == false)
                mesh.m_normalBuffer.col(currentVertexIndex) = normal;
            else
                mesh.m_normalBuffer.col(currentVertexIndex) = -normal;
            currentVertexIndex++;
        };

        //! Adds the given triangle to the specified mesh
        /*!
         * \param [in,out] mesh The mesh to which the data should be added to
         * \param [in,out] currentTriangleIndex The index of the current triangle in the index buffer (will be incremented)
         * \param [in] index0 The vertex index of vertex "0"
         * \param [in] index1 The vertex index of vertex "1"
         * \param [in] index2 The vertex index of vertex "2"
         * \param [in] flipNormal If `true`, the triangle normal is flipped (reverse index order)
         */
        static inline void addTriangle(CGMesh& mesh, IndexType& currentTriangleIndex, const IndexType& index0, const IndexType& index1, const IndexType& index2, const bool& flipNormal = false)
        {
            if (flipNormal == false)
                mesh.m_indexBuffer.col(currentTriangleIndex++) = Triangle(index0, index1, index2);
            else
                mesh.m_indexBuffer.col(currentTriangleIndex++) = Triangle(index0, index2, index1);
        };

        //! Adds the given quad (2 triangles) to the specified mesh
        /*!
         * \param [in,out] mesh The mesh to which the data should be added to
         * \param [in,out] currentTriangleIndex The index of the current triangle in the index buffer (will be incremented)
         * \param [in] index0 The vertex index of vertex "0"
         * \param [in] index1 The vertex index of vertex "1"
         * \param [in] index2 The vertex index of vertex "2"
         * \param [in] index3 The vertex index of vertex "3"
         * \param [in] flipNormal If `true`, the triangle normal is flipped (reverse index order)
         */
        static inline void addQuad(CGMesh& mesh, IndexType& currentTriangleIndex, const IndexType& index0, const IndexType& index1, const IndexType& index2, const IndexType& index3, const bool& flipNormal = false)
        {
            addTriangle(mesh, currentTriangleIndex, index0, index1, index2, flipNormal);
            addTriangle(mesh, currentTriangleIndex, index0, index2, index3, flipNormal);
        };
    };

    //! \}
} // namespace geometry
} // namespace broccoli

#endif // HAVE_EIGEN3
