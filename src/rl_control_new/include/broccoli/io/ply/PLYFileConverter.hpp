/*
 * This file is part of broccoli.
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "../../geometry/CGMesh.hpp"
#include "PLYFile.hpp"

namespace broccoli {
namespace io {
    /*!
     * \addtogroup broccoli_io_ply
     * \{
     */

    //! Interface class to convert a ply file to and from other data structures
    class PLYFileConverter {
    public:
        // Conversion to/from CGMesh
        // -------------------------
        //! Converts the given mesh to a corresponding PLY data structure
        /*!
         * \param [in] input The triangle mesh as input
         * \param [out] output The ply file as output
         * \param [out] plyResult Pointer to flag in which the result related to the ply format should be stored
         * \param [out] meshResult Pointer to flag in which the result related to the mesh should be stored
         * \return `true` on success, `false` otherwise
         */
        static inline bool convert(const geometry::CGMesh& input, PLYFile& output, PLYResult* const plyResult = nullptr, geometry::CGMeshResult* const meshResult = nullptr)
        {
            // Check, validity of mesh (important to avoid seg-faults in case of an invalid mesh)
            if (input.isValid(meshResult) == false) {
                if (plyResult != nullptr)
                    *plyResult = PLYResult::ERROR_MESH_INVALID;
                assert(false);
                return false;
            }

            // Clear data structure
            output.m_elementTypes.clear();
            output.m_elementTypes.reserve(3); // element "vertex", "face", "material"

            // Create element type "vertex"
            if (input.m_vertexBuffer.cols() > 0) {
                output.m_elementTypes.resize(output.m_elementTypes.size() + 1);
                PLYElementType& vertexElementType = output.m_elementTypes.back();
                vertexElementType.m_name = "vertex";
                vertexElementType.m_elementCount = input.m_vertexBuffer.cols();
                vertexElementType.m_propertyTypes.reserve(10); // property "x", "y", "z", "nx", "ny", "nz", "red", "green", "blue", "alpha"

                // Create property "x", "y", "z"
                for (int i = 0; i < 3; i++) {
                    vertexElementType.m_propertyTypes.resize(vertexElementType.m_propertyTypes.size() + 1);
                    PLYPropertyType& currentPropertyType = vertexElementType.m_propertyTypes.back();
                    if (i == 0)
                        currentPropertyType.m_name = "x";
                    else if (i == 1)
                        currentPropertyType.m_name = "y";
                    else
                        currentPropertyType.m_name = "z";
                    currentPropertyType.m_isListProperty = false;
                    currentPropertyType.m_valueType = PLYScalar::Type::FLOAT;
                }

                // Create property "nx", "ny", "nz"
                if (input.m_normalBuffer.cols() == input.m_vertexBuffer.cols()) {
                    for (int i = 0; i < 3; i++) {
                        vertexElementType.m_propertyTypes.resize(vertexElementType.m_propertyTypes.size() + 1);
                        PLYPropertyType& currentPropertyType = vertexElementType.m_propertyTypes.back();
                        if (i == 0)
                            currentPropertyType.m_name = "nx";
                        else if (i == 1)
                            currentPropertyType.m_name = "ny";
                        else
                            currentPropertyType.m_name = "nz";
                        currentPropertyType.m_isListProperty = false;
                        currentPropertyType.m_valueType = PLYScalar::Type::FLOAT;
                    }
                }

                // Create property "red", "green", "blue", "alpha"
                if (input.m_colorBuffer.cols() == input.m_vertexBuffer.cols()) {
                    for (int i = 0; i < 4; i++) {
                        vertexElementType.m_propertyTypes.resize(vertexElementType.m_propertyTypes.size() + 1);
                        PLYPropertyType& currentPropertyType = vertexElementType.m_propertyTypes.back();
                        if (i == 0)
                            currentPropertyType.m_name = "red";
                        else if (i == 1)
                            currentPropertyType.m_name = "green";
                        else if (i == 2)
                            currentPropertyType.m_name = "blue";
                        else
                            currentPropertyType.m_name = "alpha";
                        currentPropertyType.m_isListProperty = false;
                        currentPropertyType.m_valueType = PLYScalar::Type::UCHAR;
                    }
                }

                // Fill buffer of element "vertex"
                vertexElementType.m_data.resize(input.m_vertexBuffer.cols());
                for (size_t i = 0; i < vertexElementType.m_data.size(); i++) {
                    vertexElementType.m_data[i].resize(vertexElementType.m_propertyTypes.size());

                    // Write property "x", "y", "z"
                    for (int j = 0; j < 3; j++)
                        vertexElementType.m_data[i][j].setScalar((FLOAT_t)input.m_vertexBuffer(j, i));

                    // Create property "nx", "ny", "nz"
                    int propertyStartIndex = 3;
                    if (input.m_normalBuffer.cols() == input.m_vertexBuffer.cols()) {
                        for (int j = 0; j < 3; j++)
                            vertexElementType.m_data[i][propertyStartIndex + j].setScalar((FLOAT_t)input.m_normalBuffer(j, i));
                        propertyStartIndex += 3;
                    }

                    // Create property "red", "green", "blue", "alpha"
                    if (input.m_colorBuffer.cols() == input.m_vertexBuffer.cols()) {
                        for (int j = 0; j < 4; j++)
                            vertexElementType.m_data[i][propertyStartIndex + j].setScalar((UCHAR_t)input.m_colorBuffer(j, i));
                    }
                }
            }

            // Create element type "face"
            if (input.m_indexBuffer.cols() > 0) {
                output.m_elementTypes.resize(output.m_elementTypes.size() + 1);
                PLYElementType& faceElementType = output.m_elementTypes.back();
                faceElementType.m_name = "face";
                faceElementType.m_elementCount = input.m_indexBuffer.cols();
                faceElementType.m_propertyTypes.reserve(2); // property "vertex_indices", "material_index"

                // Create property "vertex_indices"
                faceElementType.m_propertyTypes.resize(faceElementType.m_propertyTypes.size() + 1);
                PLYPropertyType& vertexIndicesType = faceElementType.m_propertyTypes.back();
                vertexIndicesType.m_name = "vertex_indices";
                vertexIndicesType.m_isListProperty = true;
                vertexIndicesType.m_counterType = PLYScalar::Type::UCHAR;
                vertexIndicesType.m_valueType = PLYScalar::Type::UINT;

                // Create property "material_index"
                if (input.m_indexBuffer.rows() == 4) {
                    faceElementType.m_propertyTypes.resize(faceElementType.m_propertyTypes.size() + 1);
                    PLYPropertyType& materialIndexPropertyType = faceElementType.m_propertyTypes.back();
                    materialIndexPropertyType.m_name = "material_index";
                    materialIndexPropertyType.m_isListProperty = false;
                    materialIndexPropertyType.m_valueType = PLYScalar::Type::UINT;
                }

                // Fill buffer of element "face"
                faceElementType.m_data.resize(input.m_indexBuffer.cols());
                for (size_t i = 0; i < faceElementType.m_data.size(); i++) {
                    faceElementType.m_data[i].resize(faceElementType.m_propertyTypes.size());

                    // Write property "vertex_indices"
                    faceElementType.m_data[i][0].setList(std::vector<UINT_t>{ input.m_indexBuffer(0, i), input.m_indexBuffer(1, i), input.m_indexBuffer(2, i) });

                    // Write property "material_index"
                    if (input.m_indexBuffer.rows() == 4)
                        faceElementType.m_data[i][1].setScalar((UINT_t)input.m_indexBuffer(3, i));
                }
            }

            // Create element type "material"
            if (input.m_materialBuffer.size() > 0) {
                output.m_elementTypes.resize(output.m_elementTypes.size() + 1);
                PLYElementType& materialElementType = output.m_elementTypes.back();
                materialElementType.m_name = "material";
                materialElementType.m_elementCount = input.m_materialBuffer.size();
                materialElementType.m_propertyTypes.resize(19);

                // Iterate through properties
                for (size_t i = 0; i < materialElementType.m_propertyTypes.size(); i++)
                    materialElementType.m_propertyTypes[i].m_isListProperty = false;
                materialElementType.m_propertyTypes[0].m_name = "ambient_red";
                materialElementType.m_propertyTypes[0].m_valueType = PLYScalar::Type::UCHAR;
                materialElementType.m_propertyTypes[1].m_name = "ambient_green";
                materialElementType.m_propertyTypes[1].m_valueType = PLYScalar::Type::UCHAR;
                materialElementType.m_propertyTypes[2].m_name = "ambient_blue";
                materialElementType.m_propertyTypes[2].m_valueType = PLYScalar::Type::UCHAR;
                materialElementType.m_propertyTypes[3].m_name = "ambient_from_vertex_color";
                materialElementType.m_propertyTypes[3].m_valueType = PLYScalar::Type::UCHAR;
                materialElementType.m_propertyTypes[4].m_name = "ambient_coeff";
                materialElementType.m_propertyTypes[4].m_valueType = PLYScalar::Type::FLOAT;
                materialElementType.m_propertyTypes[5].m_name = "diffuse_red";
                materialElementType.m_propertyTypes[5].m_valueType = PLYScalar::Type::UCHAR;
                materialElementType.m_propertyTypes[6].m_name = "diffuse_green";
                materialElementType.m_propertyTypes[6].m_valueType = PLYScalar::Type::UCHAR;
                materialElementType.m_propertyTypes[7].m_name = "diffuse_blue";
                materialElementType.m_propertyTypes[7].m_valueType = PLYScalar::Type::UCHAR;
                materialElementType.m_propertyTypes[8].m_name = "diffuse_from_vertex_color";
                materialElementType.m_propertyTypes[8].m_valueType = PLYScalar::Type::UCHAR;
                materialElementType.m_propertyTypes[9].m_name = "diffuse_coeff";
                materialElementType.m_propertyTypes[9].m_valueType = PLYScalar::Type::FLOAT;
                materialElementType.m_propertyTypes[10].m_name = "specular_red";
                materialElementType.m_propertyTypes[10].m_valueType = PLYScalar::Type::UCHAR;
                materialElementType.m_propertyTypes[11].m_name = "specular_green";
                materialElementType.m_propertyTypes[11].m_valueType = PLYScalar::Type::UCHAR;
                materialElementType.m_propertyTypes[12].m_name = "specular_blue";
                materialElementType.m_propertyTypes[12].m_valueType = PLYScalar::Type::UCHAR;
                materialElementType.m_propertyTypes[13].m_name = "specular_from_vertex_color";
                materialElementType.m_propertyTypes[13].m_valueType = PLYScalar::Type::UCHAR;
                materialElementType.m_propertyTypes[14].m_name = "specular_coeff";
                materialElementType.m_propertyTypes[14].m_valueType = PLYScalar::Type::FLOAT;
                materialElementType.m_propertyTypes[15].m_name = "specular_power";
                materialElementType.m_propertyTypes[15].m_valueType = PLYScalar::Type::FLOAT;
                materialElementType.m_propertyTypes[16].m_name = "alpha";
                materialElementType.m_propertyTypes[16].m_valueType = PLYScalar::Type::UCHAR;
                materialElementType.m_propertyTypes[17].m_name = "alpha_from_vertex_color";
                materialElementType.m_propertyTypes[17].m_valueType = PLYScalar::Type::UCHAR;
                materialElementType.m_propertyTypes[18].m_name = "shadeless";
                materialElementType.m_propertyTypes[18].m_valueType = PLYScalar::Type::UCHAR;

                // Fill buffer of element "face"
                materialElementType.m_data.resize(input.m_materialBuffer.size());
                for (size_t i = 0; i < materialElementType.m_data.size(); i++) {
                    materialElementType.m_data[i].resize(materialElementType.m_propertyTypes.size());

                    // Write properties
                    for (int j = 0; j < 3; j++) {
                        materialElementType.m_data[i][j].setScalar((UCHAR_t)input.m_materialBuffer[i].m_ambientColor[j]);
                        materialElementType.m_data[i][5 + j].setScalar((UCHAR_t)input.m_materialBuffer[i].m_diffuseColor[j]);
                        materialElementType.m_data[i][10 + j].setScalar((UCHAR_t)input.m_materialBuffer[i].m_specularColor[j]);
                    }
                    materialElementType.m_data[i][3].setScalar((UCHAR_t)input.m_materialBuffer[i].m_ambientColorFromVertexColor);
                    materialElementType.m_data[i][4].setScalar((FLOAT_t)input.m_materialBuffer[i].m_ambientIntensity);
                    materialElementType.m_data[i][8].setScalar((UCHAR_t)input.m_materialBuffer[i].m_diffuseColorFromVertexColor);
                    materialElementType.m_data[i][9].setScalar((FLOAT_t)input.m_materialBuffer[i].m_diffuseIntensity);
                    materialElementType.m_data[i][13].setScalar((UCHAR_t)input.m_materialBuffer[i].m_specularColorFromVertexColor);
                    materialElementType.m_data[i][14].setScalar((FLOAT_t)input.m_materialBuffer[i].m_specularIntensity);
                    materialElementType.m_data[i][15].setScalar((FLOAT_t)input.m_materialBuffer[i].m_shininess);
                    materialElementType.m_data[i][16].setScalar((UCHAR_t)(input.m_materialBuffer[i].m_alpha));
                    materialElementType.m_data[i][17].setScalar((UCHAR_t)input.m_materialBuffer[i].m_alphaFromVertexColor);
                    materialElementType.m_data[i][18].setScalar((UCHAR_t)input.m_materialBuffer[i].m_shadeless);
                }
            }

            // Success
            if (meshResult != nullptr)
                *meshResult = geometry::CGMeshResult::SUCCESS;
            if (plyResult != nullptr)
                *plyResult = PLYResult::SUCCESS;
            return true;
        }

        //! Converts the current PLY data structure to a mesh
        /*!
         * \param [in] input The ply file as input
         * \param [out] output The triangle mesh as output
         * \param [out] plyResult Pointer to flag in which the result related to the ply format should be stored
         * \return `true` on success, `false` otherwise
         */
        static inline bool convert(const PLYFile& input, geometry::CGMesh& output, PLYResult* const plyResult = nullptr)
        {
            // Initialize helpers
            std::vector<int32_t> vertexIndicesList;

            // First get element type indices
            auto vertexElementTypeIndex = input.getElementTypeIndexByName("vertex");
            auto faceElementTypeIndex = input.getElementTypeIndexByName("face");
            auto materialElementTypeIndex = input.getElementTypeIndexByName("material");

            // Process vertex, normal and color buffer
            // ---------------------------------------
            if (vertexElementTypeIndex >= 0) {
                const PLYElementType& vertexElementType = input.m_elementTypes[vertexElementTypeIndex];

                // Check, if element buffer is valid
                if (vertexElementType.bufferMatchesSpecification(plyResult) == false) {
                    assert(false);
                    return false;
                }

                // Process vertex buffer
                std::array<int64_t, 3> idx_xyz; // Index of "x", "y", and "z" properties
                idx_xyz[0] = vertexElementType.getPropertyTypeIndexByName("x");
                idx_xyz[1] = vertexElementType.getPropertyTypeIndexByName("y");
                idx_xyz[2] = vertexElementType.getPropertyTypeIndexByName("z");
                if (idx_xyz[0] >= 0 && idx_xyz[1] >= 0 && idx_xyz[2] >= 0) {
                    // Setup vertex buffer
                    output.m_vertexBuffer.resize(Eigen::NoChange, vertexElementType.m_elementCount);
                    for (size_t i = 0; i < vertexElementType.m_elementCount; i++) {
                        for (int j = 0; j < 3; j++) {
                            if (vertexElementType.m_data[i][idx_xyz[j]].scalarConverted(vertexElementType.m_propertyTypes[idx_xyz[j]].m_valueType, output.m_vertexBuffer(j, i)) == false) {
                                if (plyResult != nullptr)
                                    *plyResult = PLYResult::ERROR_INVALID_BUFFER_PROPERTYVALUE;
                                assert(false);
                                return false;
                            }
                        }
                    }
                } else {
                    if (plyResult != nullptr)
                        *plyResult = PLYResult::ERROR_INVALID_VERTEX_ELEMENT_TYPE;
                    assert(false);
                    return false;
                }

                // Process normal buffer
                std::array<int64_t, 3> idx_nxnynz; // Index of "nx", "ny" and "nz" properties
                idx_nxnynz[0] = vertexElementType.getPropertyTypeIndexByName("nx");
                idx_nxnynz[1] = vertexElementType.getPropertyTypeIndexByName("ny");
                idx_nxnynz[2] = vertexElementType.getPropertyTypeIndexByName("nz");
                if (idx_nxnynz[0] >= 0 && idx_nxnynz[1] >= 0 && idx_nxnynz[2] >= 0) {
                    // Setup normal buffer
                    output.m_normalBuffer.resize(Eigen::NoChange, vertexElementType.m_elementCount);
                    for (size_t i = 0; i < vertexElementType.m_elementCount; i++) {
                        for (int j = 0; j < 3; j++) {
                            if (vertexElementType.m_data[i][idx_nxnynz[j]].scalarConverted(vertexElementType.m_propertyTypes[idx_nxnynz[j]].m_valueType, output.m_normalBuffer(j, i)) == false) {
                                if (plyResult != nullptr)
                                    *plyResult = PLYResult::ERROR_INVALID_BUFFER_PROPERTYVALUE;
                                assert(false);
                                return false;
                            }
                        }
                    }
                } else
                    output.m_normalBuffer.resize(Eigen::NoChange, 0);

                // Process color buffer
                std::array<int64_t, 4> idx_rgba; // Index of "red", "green", "blue" and "alpha" properties
                idx_rgba[0] = vertexElementType.getPropertyTypeIndexByName("red");
                idx_rgba[1] = vertexElementType.getPropertyTypeIndexByName("green");
                idx_rgba[2] = vertexElementType.getPropertyTypeIndexByName("blue");
                idx_rgba[3] = vertexElementType.getPropertyTypeIndexByName("alpha");
                if (idx_rgba[0] >= 0 && idx_rgba[1] >= 0 && idx_rgba[2] >= 0) {
                    // Setup color buffer
                    output.m_colorBuffer.resize(Eigen::NoChange, vertexElementType.m_elementCount);
                    for (size_t i = 0; i < vertexElementType.m_elementCount; i++) {
                        // Process red, green and blue channel
                        for (int j = 0; j < 3; j++) {
                            if (vertexElementType.m_data[i][idx_rgba[j]].scalarConverted(vertexElementType.m_propertyTypes[idx_rgba[j]].m_valueType, output.m_colorBuffer(j, i)) == false) {
                                if (plyResult != nullptr)
                                    *plyResult = PLYResult::ERROR_INVALID_BUFFER_PROPERTYVALUE;
                                assert(false);
                                return false;
                            }
                        }

                        // Process alpha channel (optional)
                        if (idx_rgba[3] >= 0) {
                            if (vertexElementType.m_data[i][idx_rgba[3]].scalarConverted(vertexElementType.m_propertyTypes[idx_rgba[3]].m_valueType, output.m_colorBuffer(3, i)) == false) {
                                if (plyResult != nullptr)
                                    *plyResult = PLYResult::ERROR_INVALID_BUFFER_PROPERTYVALUE;
                                assert(false);
                                return false;
                            }
                        } else
                            output.m_colorBuffer(3, i) = 255;
                    }
                } else
                    output.m_colorBuffer.resize(Eigen::NoChange, 0);
            } else {
                output.m_vertexBuffer.resize(Eigen::NoChange, 0);
                output.m_normalBuffer.resize(Eigen::NoChange, 0);
                output.m_colorBuffer.resize(Eigen::NoChange, 0);
            }

            // Process index buffer
            // --------------------
            if (faceElementTypeIndex >= 0) {
                const PLYElementType& faceElementType = input.m_elementTypes[faceElementTypeIndex];

                // Check, if element buffer is valid
                if (faceElementType.bufferMatchesSpecification(plyResult) == false) {
                    assert(false);
                    return false;
                }

                // Process index buffer
                const int64_t idx_vertexIndices = faceElementType.getPropertyTypeIndexByName("vertex_indices"); // Index of "vertex_indices" property
                const int64_t idx_materialIndices = faceElementType.getPropertyTypeIndexByName("material_index"); // Index of "material_index" property
                if (idx_vertexIndices >= 0) {
                    // Setup index buffer
                    if (idx_materialIndices >= 0)
                        output.m_indexBuffer.resize(4, faceElementType.m_elementCount);
                    else
                        output.m_indexBuffer.resize(3, faceElementType.m_elementCount);
                    for (size_t i = 0; i < faceElementType.m_elementCount; i++) {
                        // Grab list of vertex indices
                        vertexIndicesList.clear();
                        if (faceElementType.m_data[i][idx_vertexIndices].listConverted(faceElementType.m_propertyTypes[idx_vertexIndices].m_valueType, vertexIndicesList) == false) {
                            if (plyResult != nullptr)
                                *plyResult = PLYResult::ERROR_INVALID_BUFFER_PROPERTYVALUE;
                            assert(false);
                            return false;
                        } else {
                            if (vertexIndicesList.size() != 3) {
                                if (plyResult != nullptr)
                                    *plyResult = PLYResult::ERROR_INVALID_FACE_VERTEX_COUNT;
                                assert(false);
                                return false;
                            } else {
                                for (size_t j = 0; j < 3; j++)
                                    output.m_indexBuffer(j, i) = vertexIndicesList[j];
                            }
                        }

                        // Grab material index (if available)
                        if (idx_materialIndices >= 0) {
                            if (faceElementType.m_data[i][idx_materialIndices].scalarConverted(faceElementType.m_propertyTypes[idx_materialIndices].m_valueType, output.m_indexBuffer(3, i)) == false) {
                                if (plyResult != nullptr)
                                    *plyResult = PLYResult::ERROR_INVALID_BUFFER_PROPERTYVALUE;
                                assert(false);
                                return false;
                            }
                        }
                    }
                } else {
                    if (plyResult != nullptr)
                        *plyResult = PLYResult::ERROR_INVALID_FACE_ELEMENT_TYPE;
                    assert(false);
                    return false;
                }
            } else
                output.m_indexBuffer.resize(0, 0);

            // Process material buffer
            // -----------------------
            if (materialElementTypeIndex >= 0) {
                const PLYElementType& materialElementType = input.m_elementTypes[materialElementTypeIndex];

                // Check, if element buffer is valid
                if (materialElementType.bufferMatchesSpecification(plyResult) == false) {
                    assert(false);
                    return false;
                }

                // Process index buffer
                std::array<int64_t, 3> idx_ambientRGB; // Index of "ambient_red", "ambient_green" and "ambient_blue" property
                idx_ambientRGB[0] = materialElementType.getPropertyTypeIndexByName("ambient_red");
                idx_ambientRGB[1] = materialElementType.getPropertyTypeIndexByName("ambient_green");
                idx_ambientRGB[2] = materialElementType.getPropertyTypeIndexByName("ambient_blue");
                const int64_t idx_ambientFromVertexColor = materialElementType.getPropertyTypeIndexByName("ambient_from_vertex_color"); // Index of "ambient_from_vertex_color" property
                const int64_t idx_ambientCoeff = materialElementType.getPropertyTypeIndexByName("ambient_coeff"); // Index of "ambient_coeff" property
                std::array<int64_t, 3> idx_diffuseRGB; // Index of "diffuse_red", "diffuse_green" and "diffuse_blue" property
                idx_diffuseRGB[0] = materialElementType.getPropertyTypeIndexByName("diffuse_red");
                idx_diffuseRGB[1] = materialElementType.getPropertyTypeIndexByName("diffuse_green");
                idx_diffuseRGB[2] = materialElementType.getPropertyTypeIndexByName("diffuse_blue");
                const int64_t idx_diffuseFromVertexColor = materialElementType.getPropertyTypeIndexByName("diffuse_from_vertex_color"); // Index of "diffuse_from_vertex_color" property
                const int64_t idx_diffuseCoeff = materialElementType.getPropertyTypeIndexByName("diffuse_coeff"); // Index of "diffuse_coeff" property
                std::array<int64_t, 3> idx_specularRGB; // Index of "specular_red", "specular_green" and property
                idx_specularRGB[0] = materialElementType.getPropertyTypeIndexByName("specular_red");
                idx_specularRGB[1] = materialElementType.getPropertyTypeIndexByName("specular_green");
                idx_specularRGB[2] = materialElementType.getPropertyTypeIndexByName("specular_blue");
                const int64_t idx_specularFromVertexColor = materialElementType.getPropertyTypeIndexByName("specular_from_vertex_color"); // Index of "specular_from_vertex_color" property
                const int64_t idx_specularCoeff = materialElementType.getPropertyTypeIndexByName("specular_coeff"); // Index of "specular_coeff" property
                const int64_t idx_specularPower = materialElementType.getPropertyTypeIndexByName("specular_power"); // Index of "specular_coeff" property
                const int64_t idx_alpha = materialElementType.getPropertyTypeIndexByName("alpha"); // Index of "alpha" property
                const int64_t idx_alphaFromVertexColor = materialElementType.getPropertyTypeIndexByName("alpha_from_vertex_color"); // Index of "alpha" property
                const int64_t idx_shadeless = materialElementType.getPropertyTypeIndexByName("shadeless"); // Index of "alpha" property

                // Setup material buffer
                output.m_materialBuffer.clear();
                output.m_materialBuffer.resize(materialElementType.m_elementCount);
                bool errorOccured = false; // Flag indicating, if an error occured
                for (size_t i = 0; i < materialElementType.m_elementCount; i++) {
                    // Iterate through color channels
                    for (int j = 0; j < 3; j++) {
                        if (idx_ambientRGB[j] >= 0 && materialElementType.m_data[i][idx_ambientRGB[j]].scalarConverted(materialElementType.m_propertyTypes[idx_ambientRGB[j]].m_valueType, output.m_materialBuffer[i].m_ambientColor[j]) == false)
                            errorOccured = true;
                        if (idx_diffuseRGB[j] >= 0 && materialElementType.m_data[i][idx_diffuseRGB[j]].scalarConverted(materialElementType.m_propertyTypes[idx_diffuseRGB[j]].m_valueType, output.m_materialBuffer[i].m_diffuseColor[j]) == false)
                            errorOccured = true;
                        if (idx_specularRGB[j] >= 0 && materialElementType.m_data[i][idx_specularRGB[j]].scalarConverted(materialElementType.m_propertyTypes[idx_specularRGB[j]].m_valueType, output.m_materialBuffer[i].m_specularColor[j]) == false)
                            errorOccured = true;
                    }

                    // Process "ambient_from_vertex_color"
                    if (idx_ambientFromVertexColor >= 0 && materialElementType.m_data[i][idx_ambientFromVertexColor].scalarConverted(materialElementType.m_propertyTypes[idx_ambientFromVertexColor].m_valueType, output.m_materialBuffer[i].m_ambientColorFromVertexColor) == false)
                        errorOccured = true;

                    // Process "ambient_coeff"
                    if (idx_ambientCoeff >= 0 && materialElementType.m_data[i][idx_ambientCoeff].scalarConverted(materialElementType.m_propertyTypes[idx_ambientCoeff].m_valueType, output.m_materialBuffer[i].m_ambientIntensity) == false)
                        errorOccured = true;

                    // Process "diffuse_from_vertex_color"
                    if (idx_diffuseFromVertexColor >= 0 && materialElementType.m_data[i][idx_diffuseFromVertexColor].scalarConverted(materialElementType.m_propertyTypes[idx_diffuseFromVertexColor].m_valueType, output.m_materialBuffer[i].m_diffuseColorFromVertexColor) == false)
                        errorOccured = true;

                    // Process "diffuse_coeff"
                    if (idx_diffuseCoeff >= 0 && materialElementType.m_data[i][idx_diffuseCoeff].scalarConverted(materialElementType.m_propertyTypes[idx_diffuseCoeff].m_valueType, output.m_materialBuffer[i].m_diffuseIntensity) == false)
                        errorOccured = true;

                    // Process "specular_from_vertex_color"
                    if (idx_specularFromVertexColor >= 0 && materialElementType.m_data[i][idx_specularFromVertexColor].scalarConverted(materialElementType.m_propertyTypes[idx_specularFromVertexColor].m_valueType, output.m_materialBuffer[i].m_specularColorFromVertexColor) == false)
                        errorOccured = true;

                    // Process "specular_coeff"
                    if (idx_specularCoeff >= 0 && materialElementType.m_data[i][idx_specularCoeff].scalarConverted(materialElementType.m_propertyTypes[idx_specularCoeff].m_valueType, output.m_materialBuffer[i].m_specularIntensity) == false)
                        errorOccured = true;

                    // Process "specular_power"
                    if (idx_specularPower >= 0 && materialElementType.m_data[i][idx_specularPower].scalarConverted(materialElementType.m_propertyTypes[idx_specularPower].m_valueType, output.m_materialBuffer[i].m_shininess) == false)
                        errorOccured = true;

                    // Process "alpha"
                    if (idx_alpha >= 0 && materialElementType.m_data[i][idx_alpha].scalarConverted(materialElementType.m_propertyTypes[idx_alpha].m_valueType, output.m_materialBuffer[i].m_alpha) == false)
                        errorOccured = true;

                    // Process "alpha_from_vertex_color"
                    if (idx_alphaFromVertexColor >= 0 && materialElementType.m_data[i][idx_alphaFromVertexColor].scalarConverted(materialElementType.m_propertyTypes[idx_alphaFromVertexColor].m_valueType, output.m_materialBuffer[i].m_alphaFromVertexColor) == false)
                        errorOccured = true;

                    // Process "shadeless"
                    if (idx_shadeless >= 0 && materialElementType.m_data[i][idx_shadeless].scalarConverted(materialElementType.m_propertyTypes[idx_shadeless].m_valueType, output.m_materialBuffer[i].m_shadeless) == false)
                        errorOccured = true;
                }

                // Handle errors
                if (errorOccured == true) {
                    if (plyResult != nullptr)
                        *plyResult = PLYResult::ERROR_INVALID_BUFFER_PROPERTYVALUE;
                    assert(false);
                    return false;
                }
            } else
                output.m_materialBuffer.clear();

            // Success
            if (plyResult != nullptr)
                *plyResult = PLYResult::SUCCESS;
            return true;
        }
    };
    //! \}
} // namespace io
} // namespace broccoli
