/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "../io/serialization/serialization.hpp"
#include <array>
#include <assert.h>
#include <stdint.h>
#include <string>

namespace broccoli {
namespace geometry {
    /*!
     * \addtogroup broccoli_geometry
     * \{
     */

    //! Generic material model used in <b>C</b>omputer <b>G</b>raphics
    /*!
     * Contains common properties of various material models. Inspired by the PLY format and (basic) Blender material models.
     */
    class CGMaterial : public io::serialization::SerializableData {
    public:
        //! Constructor
        CGMaterial()
            : m_name("")
            , m_ambientColor{ { 255, 255, 255 } }
            , m_ambientColorFromVertexColor(false)
            , m_ambientIntensity(0.0)
            , m_diffuseColor{ { 255, 255, 255 } }
            , m_diffuseColorFromVertexColor(false)
            , m_diffuseIntensity(1.0)
            , m_specularColor{ { 255, 255, 255 } }
            , m_specularColorFromVertexColor(false)
            , m_specularIntensity(0.0)
            , m_shininess(0.0)
            , m_alpha(255)
            , m_alphaFromVertexColor(false)
            , m_shadeless(false)
        {
        }

        //! Destructor
        virtual ~CGMaterial()
        {
        }

        //! Comparison operator: **equality**
        bool operator==(const CGMaterial& reference) const
        {
            // Compare members
            if (m_name != reference.m_name)
                return false;
            if (m_ambientColor != reference.m_ambientColor)
                return false;
            if (m_ambientColorFromVertexColor != reference.m_ambientColorFromVertexColor)
                return false;
            if (m_ambientIntensity != reference.m_ambientIntensity)
                return false;
            if (m_diffuseColor != reference.m_diffuseColor)
                return false;
            if (m_diffuseColorFromVertexColor != reference.m_diffuseColorFromVertexColor)
                return false;
            if (m_diffuseIntensity != reference.m_diffuseIntensity)
                return false;
            if (m_specularColor != reference.m_specularColor)
                return false;
            if (m_specularColorFromVertexColor != reference.m_specularColorFromVertexColor)
                return false;
            if (m_specularIntensity != reference.m_specularIntensity)
                return false;
            if (m_shininess != reference.m_shininess)
                return false;
            if (m_alpha != reference.m_alpha)
                return false;
            if (m_alphaFromVertexColor != reference.m_alphaFromVertexColor)
                return false;
            if (m_shadeless != reference.m_shadeless)
                return false;

            // All members are equal -> equal
            return true;
        }

        //! Comparison operator: **inequality**
        bool operator!=(const CGMaterial& reference) const { return !(*this == reference); }

        // Members
        std::string m_name; //!< Name of the material as string
        std::array<uint8_t, 3> m_ambientColor; //!< Ambient color [red=0...255, green, blue]
        bool m_ambientColorFromVertexColor; //!< If `true`, the ambient color is extracted from the vertex color (RGB channel)
        float m_ambientIntensity; //!< Intensity of ambient color
        std::array<uint8_t, 3> m_diffuseColor; //!< Diffuse color [red=0...255, green, blue]
        bool m_diffuseColorFromVertexColor; //!< If `true`, the diffuse color is extracted from the vertex color (RGB channel)
        float m_diffuseIntensity; //!< Intensity of diffuse color
        std::array<uint8_t, 3> m_specularColor; //!< Specular color [red=0...255, green, blue]
        bool m_specularColorFromVertexColor; //!< If `true`, the specular color is extracted from the vertex color (RGB channel)
        float m_specularIntensity; //!< Intensity of specular color
        float m_shininess; //!< Shininess (phong power)
        uint8_t m_alpha; //!< Alpha (0...255) (transparency)
        bool m_alphaFromVertexColor; //!< If `true`, the alpha value is extracted from the vertex color (alpha channel)
        bool m_shadeless; //!< If `true` the material is rendered shadeless

    protected:
        // Serialization
        // -------------
        // Serialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize serializePayload(io::serialization::BinaryStream& stream, const io::serialization::Endianness& endianness) const
        {
            return io::serialization::serialize(stream, endianness, //
                m_name, //
                m_ambientColor, m_ambientColorFromVertexColor, m_ambientIntensity, //
                m_diffuseColor, m_diffuseColorFromVertexColor, m_diffuseIntensity, //
                m_specularColor, m_specularColorFromVertexColor, m_specularIntensity, //
                m_shininess, //
                m_alpha, m_alphaFromVertexColor, //
                m_shadeless);
        }

        // Deserialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize deSerializePayload(const io::serialization::BinaryStream& stream, const io::serialization::BinaryStreamSize& index, const io::serialization::BinaryStreamSize& payloadSize, const io::serialization::Endianness& endianness)
        {
            (void)payloadSize; // unused
            return io::serialization::deSerialize(stream, index, endianness, //
                m_name, //
                m_ambientColor, m_ambientColorFromVertexColor, m_ambientIntensity, //
                m_diffuseColor, m_diffuseColorFromVertexColor, m_diffuseIntensity, //
                m_specularColor, m_specularColorFromVertexColor, m_specularIntensity, //
                m_shininess, //
                m_alpha, m_alphaFromVertexColor, //
                m_shadeless);
        }
    };

    //! \}
} // namespace geometry
} // namespace broccoli
