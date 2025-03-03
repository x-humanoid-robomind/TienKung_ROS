/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "CGMesh.hpp"

namespace broccoli {
namespace geometry {
    /*!
     * \addtogroup broccoli_geometry
     * \{
     */

    //! Collection of helpful tools for dealing with \ref broccoli::geometry::CGMesh objects
    class CGMeshTools {
    public:
        // Type definitions
        using IndexType = CGMesh::IndexType; //!< \copydoc broccoli::geometry::CGMesh::IndexType
        using ColorChannelType = CGMesh::ColorChannelType; //!< \copydoc broccoli::geometry::CGMesh::ColorChannelType

        // Mesh operations
        // ---------------
        //! Generates a default index buffer (each triple in the vertex buffer defines a triangle)
        /*!
         * (Re-)creates the \ref broccoli::geometry::CGMesh::m_indexBuffer from the \ref broccoli::geometry::CGMesh::m_vertexBuffer. Each triple of vertices in the \ref broccoli::geometry::CGMesh::m_vertexBuffer is interpreted as a
         * triangle (disconnected from all other triangles). One might specify a \p materialIndex \f$>0\f$ which is set for **all** triangles.
         *
         * \warning The column-count of the \ref broccoli::geometry::CGMesh::m_vertexBuffer has to be a multiple of 3!
         *
         * \param [in,out] mesh The mesh to be processed (will be overwritten!)
         * \param [in] materialIndex Material index \f$\geq 0\f$ for **all** triangles (use \f$-1\f$ for no material indices at all)
         * \return `True` on success, `false` otherwise (if the size of the vertex buffer is not a multiple of 3)
         */
        static inline bool generateDefaultIndexBuffer(CGMesh& mesh, const int32_t& materialIndex = -1)
        {
            // Check if mesh is valid
            assert(mesh.isValid());

            // Check, if size of vertex buffer is valid
            if (mesh.m_vertexBuffer.cols() % 3 != 0)
                return false;

            // Re-allocate index buffer
            if (materialIndex < 0)
                mesh.m_indexBuffer.resize(3, mesh.m_vertexBuffer.cols() / 3);
            else
                mesh.m_indexBuffer.resize(4, mesh.m_vertexBuffer.cols() / 3);

            // Check, if there are any triangles
            if (mesh.m_vertexBuffer.cols() == 0)
                return true; // We are done

            // Generate vertex indices for all triangles
            IndexType currentVertexIndex = 0;
            for (Eigen::Index j = 0; j < mesh.m_indexBuffer.cols(); j++)
                for (Eigen::Index i = 0; i < 3; i++)
                    mesh.m_indexBuffer(i, j) = currentVertexIndex++;

            // Generate material indices for all triangles
            if (materialIndex >= 0)
                mesh.m_indexBuffer.row(3).setConstant((IndexType)materialIndex);

            // Success
            return true;
        }

        //! Computes the \ref broccoli::geometry::CGMesh::m_colorBuffer (vertex colors) from the linked materials
        /*!
         * This method runs through all triangles of the mesh and sets the vertex color according to the linked material.
         * Uses \ref broccoli::geometry::CGMaterial::m_diffuseColor and \ref broccoli::geometry::CGMaterial::m_alpha as vertex color (red, green, blue, alpha)).
         *
         * \warning Overwrites existing vertex colors. If a vertex belongs to multiple triangles with different materials,
         * the vertex color is set to the material color of the last triangle in the index buffer.
         *
         * \param [in,out] mesh The mesh to be processed (will be overwritten!)
         * \param [in] defaultVertexColor Default vertex color to use in case no material information is available
         */
        static inline void computeVertexColorFromMaterial(CGMesh& mesh, const Eigen::Matrix<ColorChannelType, 4, 1>& defaultVertexColor = (Eigen::Matrix<ColorChannelType, 4, 1>() << 255, 255, 255, 255).finished())
        {
            // Check if mesh is valid
            assert(mesh.isValid());

            // Setup color buffer, if it not specified yet
            if (mesh.m_colorBuffer.cols() != mesh.m_vertexBuffer.cols()) {
                mesh.m_colorBuffer.resize(Eigen::NoChange, mesh.m_vertexBuffer.cols());
                for (Eigen::Index i = 0; i < mesh.m_colorBuffer.cols(); i++)
                    mesh.m_colorBuffer.col(i) = defaultVertexColor;
            }

            // Check, if materials are specified
            if (mesh.m_indexBuffer.rows() == 3) {
                // No materials specified -> nothing to do...
            } else {
                // Materials specified -> iterate through all triangles
                for (Eigen::Index j = 0; j < mesh.m_indexBuffer.cols(); j++) {
                    // Get material index
                    const IndexType& materialIndex = mesh.m_indexBuffer(3, j);

                    // Check, if material index is valid
                    if (materialIndex < mesh.m_materialBuffer.size()) {
                        // Pass through vertices of triangle
                        for (Eigen::Index i = 0; i < 3; i++) {
                            // Get vertex index
                            const IndexType& vertexIndex = mesh.m_indexBuffer(i, j);

                            // Check, if vertex index is valid
                            if (vertexIndex < mesh.m_vertexBuffer.cols()) {
                                if (mesh.m_materialBuffer[materialIndex].m_diffuseColorFromVertexColor == false)
                                    mesh.m_colorBuffer.block<3, 1>(0, vertexIndex) = Eigen::Map<Eigen::Matrix<ColorChannelType, 3, 1, Eigen::ColMajor>>(mesh.m_materialBuffer[materialIndex].m_diffuseColor.data());
                                if (mesh.m_materialBuffer[materialIndex].m_alphaFromVertexColor == false)
                                    mesh.m_colorBuffer(3, vertexIndex) = mesh.m_materialBuffer[materialIndex].m_alpha;
                            }
                            // else: invalid index -> do nothing
                        }
                    }
                    // else: invalid index -> do nothing
                }
            }
        }

        //! Converts all linked materials to corresponding vertex colors of the mesh (calls \ref broccoli::geometry::CGMeshTools::computeVertexColorFromMaterial())
        /*!
         * This method performs three steps:
         * * compute vertex colors form the linked materials (diffuse color and alpha)
         * * remove all linked materials
         * * add a single material to all vertices which uses the vertex color to specify diffuse color and alpha value
         *
         * \param [in,out] mesh The mesh to be processed (will be overwritten!)
         * \param [in] assignedVertexColorMaterialName Name of the new material (alias vertex color material)
         * \param [in] defaultVertexColor Default vertex color for triangles without assigned material
         */
        static inline void convertMaterialsToVertexColor(CGMesh& mesh, const std::string& assignedVertexColorMaterialName = "", const Eigen::Matrix<ColorChannelType, 4, 1>& defaultVertexColor = (Eigen::Matrix<ColorChannelType, 4, 1>() << 255, 255, 255, 255).finished())
        {
            // Check if mesh is valid
            assert(mesh.isValid());

            // Step 1: re-compute vertex colors from materials
            computeVertexColorFromMaterial(mesh, defaultVertexColor);

            // Step 2: Remove all linked materials
            mesh.m_materialBuffer.clear();

            // Step 3: create material which uses the vertex color
            mesh.m_materialBuffer.resize(1);
            mesh.m_materialBuffer[0].m_name = assignedVertexColorMaterialName;
            mesh.m_materialBuffer[0].m_diffuseColorFromVertexColor = true; // Use vertex color as diffuse color
            mesh.m_materialBuffer[0].m_alphaFromVertexColor = true; // Use vertex alpha for material alpha
            mesh.m_indexBuffer.conservativeResize(4, Eigen::NoChange);
            mesh.m_indexBuffer.row(3).setConstant(0);
        }

        //! Discards any unneccessary elements in the vertex-, normal-, color- and material-buffer which do not contribute to the mesh
        /*!
         * May be used to get a "minimal representation" of the mesh (without unneccessary data).
         *
         * \warning The complete mesh is rewritten, thus this is an expensive call!
         *
         * \param [in,out] mesh The mesh to be processed (will be overwritten!)
         *
         * \return Array [count-of-removed-vertices, count-of-removed-materials]
         */
        static inline std::array<IndexType, 2> cleanup(CGMesh& mesh)
        {
            // Check if mesh is valid
            assert(mesh.isValid());

            // Initialize return value
            std::array<IndexType, 2> returnValue{ 0, 0 };

            // Create memory for new ("cleaned") mesh
            CGMesh newMesh;
            newMesh.m_indexBuffer.resize(mesh.m_indexBuffer.rows(), mesh.m_indexBuffer.cols()); // New mesh has same count of triangles

            // Allocate mapping between old and new buffers
            std::vector<IndexType> vertexIndexMap(mesh.m_vertexBuffer.cols()); // Stores index of (old) vertex/normal/color in new mesh
            std::vector<bool> oldVertexUsedInNewMesh(mesh.m_vertexBuffer.cols(), false); // Flag indicating, if the corresponding vertex in the old mesh is used in the new mesh
            uint64_t newVertexCount = 0; // Count of vertices in new mesh
            std::vector<IndexType> materialIndexMap(mesh.m_materialBuffer.size()); // Stores index of (old) material in new mesh
            std::vector<bool> oldMaterialUsedInNewMesh(mesh.m_materialBuffer.size(), false); // Flag indicating, if the corresponding material in the old mesh is used in the new mesh
            IndexType newMaterialCount = 0; // Count of materials in new mesh

            // Process vertex indices in index buffer
            // --------------------------------------
            // Run through all triangles of the "old" mesh
            for (Eigen::Index j = 0; j < mesh.m_indexBuffer.cols(); j++) {
                // Run through vertices of the triangle
                for (Eigen::Index i = 0; i < 3; i++) {
                    // Get old vertex index
                    const IndexType& oldVertexIndex = mesh.m_indexBuffer(i, j);

                    // Check, if vertex index has already occured
                    if (oldVertexUsedInNewMesh[oldVertexIndex] == true) {
                        // Already occured -> reuse existing vertex
                        newMesh.m_indexBuffer(i, j) = vertexIndexMap[oldVertexIndex];
                    } else {
                        // Vertex has not occured before -> store it in new mesh
                        newMesh.m_indexBuffer(i, j) = newVertexCount;
                        vertexIndexMap[oldVertexIndex] = newVertexCount;
                        oldVertexUsedInNewMesh[oldVertexIndex] = true;
                        newVertexCount++;
                    }
                }
            }

            // Process material indices in index buffer
            // ----------------------------------------
            if (mesh.m_indexBuffer.rows() == 4) {
                // Run through all triangles of the "old" mesh
                for (Eigen::Index j = 0; j < mesh.m_indexBuffer.cols(); j++) {
                    // Get old material index
                    const IndexType& oldMaterialIndex = mesh.m_indexBuffer(3, j);

                    // Check, if material index has already occured
                    if (oldMaterialUsedInNewMesh[oldMaterialIndex] == true) {
                        // Already occured -> reuse existing material
                        newMesh.m_indexBuffer(3, j) = materialIndexMap[oldMaterialIndex];
                    } else {
                        // Material has not occured before -> store it in new mesh
                        newMesh.m_indexBuffer(3, j) = newMaterialCount;
                        materialIndexMap[oldMaterialIndex] = newMaterialCount;
                        oldMaterialUsedInNewMesh[oldMaterialIndex] = true;
                        newMaterialCount++;
                    }
                }
            }

            // Re-assemble vertex-, normal- and color-buffers
            // ----------------------------------------------
            if (newVertexCount > 0) {
                newMesh.m_vertexBuffer.resize(Eigen::NoChange, newVertexCount);
                if (mesh.m_normalBuffer.cols() > 0)
                    newMesh.m_normalBuffer.resize(Eigen::NoChange, newVertexCount);
                if (mesh.m_colorBuffer.cols() > 0)
                    newMesh.m_colorBuffer.resize(Eigen::NoChange, newVertexCount);
                for (size_t oldVertexIndex = 0; oldVertexIndex < vertexIndexMap.size(); oldVertexIndex++) {
                    if (oldVertexUsedInNewMesh[oldVertexIndex] == true) {
                        const IndexType& newVertexIndex = vertexIndexMap[oldVertexIndex];
                        newMesh.m_vertexBuffer.col(newVertexIndex) = mesh.m_vertexBuffer.col(oldVertexIndex);
                        if (mesh.m_normalBuffer.cols() > 0)
                            newMesh.m_normalBuffer.col(newVertexIndex) = mesh.m_normalBuffer.col(oldVertexIndex);
                        if (mesh.m_colorBuffer.cols() > 0)
                            newMesh.m_colorBuffer.col(newVertexIndex) = mesh.m_colorBuffer.col(oldVertexIndex);
                    }
                }
            }

            // Re-assemble material buffer
            // ---------------------------
            if (newMaterialCount > 0) {
                newMesh.m_materialBuffer.resize(newMaterialCount);
                for (size_t oldMaterialIndex = 0; oldMaterialIndex < materialIndexMap.size(); oldMaterialIndex++) {
                    if (oldMaterialUsedInNewMesh[oldMaterialIndex] == true) {
                        const IndexType& newMaterialIndex = materialIndexMap[oldMaterialIndex];
                        newMesh.m_materialBuffer[newMaterialIndex] = mesh.m_materialBuffer[oldMaterialIndex];
                    }
                }
            }

            // Postprocessing
            // --------------
            // Set return value
            returnValue[0] = mesh.m_vertexBuffer.cols() - newVertexCount;
            returnValue[1] = mesh.m_materialBuffer.size() - newMaterialCount;

            // Swap memory of old mesh and new mesh
            mesh = newMesh;

            // Pass back return value
            return returnValue;
        }

        //! Creates a new CGMesh by merging the given CGMeshes together
        /*!
         * \param [in] inputMeshes List of pointers to input meshes
         * \param [in] defaultVertexColor Default vertex color to use in case no color information is available
         * \return Output mesh
         */
        static inline CGMesh merge(const std::vector<CGMesh const*>& inputMeshes, const Eigen::Matrix<ColorChannelType, 4, 1>& defaultVertexColor = (Eigen::Matrix<ColorChannelType, 4, 1>() << 255, 255, 255, 255).finished())
        {
            // Initialize output mesh
            CGMesh outputMesh;

            // Check input meshes
            IndexType vertexBufferSize = 0;
            bool normalBufferAvaiable = false;
            bool colorBufferAvailable = false;
            IndexType materialBufferSize = 0;
            bool materialIndicesAvailable = false;
            IndexType indexBufferSize = 0;
            for (size_t i = 0; i < inputMeshes.size(); i++) {
                // Check validity of input mesh
                assert(inputMeshes[i]->isValid());

                // Process dimensions
                vertexBufferSize += inputMeshes[i]->m_vertexBuffer.cols();
                if (inputMeshes[i]->m_normalBuffer.cols() > 0)
                    normalBufferAvaiable = true;
                if (inputMeshes[i]->m_colorBuffer.cols() > 0)
                    colorBufferAvailable = true;
                materialBufferSize += inputMeshes[i]->m_materialBuffer.size();
                if (inputMeshes[i]->m_indexBuffer.rows() == 4)
                    materialIndicesAvailable = true;
                indexBufferSize += inputMeshes[i]->m_indexBuffer.cols();
            }

            // Allocate buffers
            outputMesh.m_vertexBuffer.resize(Eigen::NoChange, vertexBufferSize);
            if (normalBufferAvaiable == true)
                outputMesh.m_normalBuffer.resize(Eigen::NoChange, vertexBufferSize);
            if (colorBufferAvailable == true)
                outputMesh.m_colorBuffer.resize(Eigen::NoChange, vertexBufferSize);
            outputMesh.m_materialBuffer.reserve(materialBufferSize + 1); // +1 since it might be necessary to add a default material
            if (materialIndicesAvailable == false)
                outputMesh.m_indexBuffer.resize(3, indexBufferSize);
            else
                outputMesh.m_indexBuffer.resize(4, indexBufferSize);

            // Copy data of buffers
            IndexType currentVertexBufferPosition = 0;
            IndexType currentMaterialBufferPosition = 0;
            IndexType currentIndexBufferPosition = 0;
            bool addDefaultMaterial = false; // Flag indicating, if a default material should be appended to the list of materials (assigned to triangles without dedicated material info)
            for (size_t i = 0; i < inputMeshes.size(); i++) {
                // Copy vertex buffer
                if (inputMeshes[i]->m_vertexBuffer.cols() > 0)
                    outputMesh.m_vertexBuffer.block(0, currentVertexBufferPosition, 3, inputMeshes[i]->m_vertexBuffer.cols()) = inputMeshes[i]->m_vertexBuffer;

                // Copy normal buffer
                if (inputMeshes[i]->m_normalBuffer.cols() > 0)
                    outputMesh.m_normalBuffer.block(0, currentVertexBufferPosition, 3, inputMeshes[i]->m_normalBuffer.cols()) = inputMeshes[i]->m_normalBuffer;
                else if (normalBufferAvaiable == true && inputMeshes[i]->m_vertexBuffer.cols() > 0) {
                    // No normals available in input mesh, but output mesh requires normals -> compute per-vertex normals from per-triangle normals
                    CGMesh inputMeshWithNormals = *inputMeshes[i];
                    inputMeshWithNormals.recomputeNormals();
                    outputMesh.m_normalBuffer.block(0, currentVertexBufferPosition, 3, inputMeshWithNormals.m_normalBuffer.cols()) = inputMeshWithNormals.m_normalBuffer;
                }

                // Copy color buffer
                if (inputMeshes[i]->m_colorBuffer.cols() > 0)
                    outputMesh.m_colorBuffer.block(0, currentVertexBufferPosition, 4, inputMeshes[i]->m_colorBuffer.cols()) = inputMeshes[i]->m_colorBuffer;
                else if (colorBufferAvailable == true && inputMeshes[i]->m_vertexBuffer.cols() > 0) {
                    // No colors available in input mesh, but output mesh requires colors -> fill with default color
                    for (Eigen::Index j = currentVertexBufferPosition; j < currentVertexBufferPosition + inputMeshes[i]->m_vertexBuffer.cols(); j++)
                        outputMesh.m_colorBuffer.col(j) = defaultVertexColor;
                }

                // Copy material buffer
                if (inputMeshes[i]->m_materialBuffer.size() > 0)
                    outputMesh.m_materialBuffer.insert(outputMesh.m_materialBuffer.end(), inputMeshes[i]->m_materialBuffer.begin(), inputMeshes[i]->m_materialBuffer.end());

                // Copy index buffer
                if (inputMeshes[i]->m_indexBuffer.cols() > 0) {
                    // Copy vertex indices
                    Eigen::Matrix<IndexType, 3, Eigen::Dynamic> vertexIndexShift;
                    vertexIndexShift.resize(Eigen::NoChange, inputMeshes[i]->m_indexBuffer.cols());
                    vertexIndexShift.setConstant(currentVertexBufferPosition);
                    outputMesh.m_indexBuffer.block(0, currentIndexBufferPosition, 3, inputMeshes[i]->m_indexBuffer.cols()) = inputMeshes[i]->m_indexBuffer.block(0, 0, 3, inputMeshes[i]->m_indexBuffer.cols()) + vertexIndexShift;

                    // Copy material indices
                    if (materialIndicesAvailable == true) {
                        if (inputMeshes[i]->m_indexBuffer.rows() == 4) {
                            Eigen::Matrix<IndexType, 1, Eigen::Dynamic> materialIndexShift;
                            materialIndexShift.resize(Eigen::NoChange, inputMeshes[i]->m_indexBuffer.cols());
                            materialIndexShift.setConstant(currentMaterialBufferPosition);
                            outputMesh.m_indexBuffer.block(3, currentIndexBufferPosition, 1, inputMeshes[i]->m_indexBuffer.cols()) = inputMeshes[i]->m_indexBuffer.block(3, 0, 1, inputMeshes[i]->m_indexBuffer.cols()) + materialIndexShift;
                        } else {
                            // No material index data available in input mesh, but output mesh requires material index data -> fill with default material
                            addDefaultMaterial = true;
                            Eigen::Matrix<IndexType, 1, Eigen::Dynamic> defaultMaterialBlock;
                            defaultMaterialBlock.resize(Eigen::NoChange, inputMeshes[i]->m_indexBuffer.cols());
                            defaultMaterialBlock.setConstant(materialBufferSize);
                            outputMesh.m_indexBuffer.block(3, currentIndexBufferPosition, 1, inputMeshes[i]->m_indexBuffer.cols()) = defaultMaterialBlock;
                        }
                    }
                }

                // Update current positions
                currentVertexBufferPosition += inputMeshes[i]->m_vertexBuffer.cols();
                currentMaterialBufferPosition += inputMeshes[i]->m_materialBuffer.size();
                currentIndexBufferPosition += inputMeshes[i]->m_indexBuffer.cols();
            }

            // Add default material
            if (addDefaultMaterial == true) {
                CGMaterial defaultMaterial;
                if (colorBufferAvailable == true) {
                    // Use vertex colors as diffuse colors and alpha values
                    defaultMaterial.m_diffuseColorFromVertexColor = true;
                    defaultMaterial.m_alphaFromVertexColor = true;
                }
                outputMesh.m_materialBuffer.push_back(defaultMaterial);
            }

            // Pass back output mesh
            return outputMesh;
        }

        //! Creates a new CGMesh by merging the given two CGMeshes together
        /*!
         * \param [in] firstMesh First mesh to merge
         * \param [in] secondMesh Second mesh to merge
         * \param [in] defaultVertexColor Default vertex color to use in case no color information is available
         * \return Output mesh
         */
        static inline CGMesh merge(const CGMesh& firstMesh, const CGMesh& secondMesh, const Eigen::Matrix<ColorChannelType, 4, 1>& defaultVertexColor = (Eigen::Matrix<ColorChannelType, 4, 1>() << 255, 255, 255, 255).finished()) { return merge(std::vector<CGMesh const*>{ &firstMesh, &secondMesh }, defaultVertexColor); }
    };

    //! \}
} // namespace geometry
} // namespace broccoli

#endif // HAVE_EIGEN3
