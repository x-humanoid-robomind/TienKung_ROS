/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../../geometry/CGMeshFactory.hpp"
#include "../../parallel/ThreadSafeContainer.hpp"

namespace broccoli {
namespace analysis {
    //! Bounding box covering evaluated taskspace samples
    /*! \ingroup broccoli_analysis_taskspace */
    class TaskSpaceBoundingBox : protected parallel::ThreadSafeContainer, public io::serialization::SerializableData {
    public:
        //! Default constructor
        TaskSpaceBoundingBox()
            : parallel::ThreadSafeContainer()
            , io::serialization::SerializableData()
        {
        }

    protected:
        //! Copy constructor (internal)
        /*!
         * \warning **Not thread-safe** -> should only be called by the thread-safe wrapper!
         *
         * \param [in] original Reference to original object.
         */
        TaskSpaceBoundingBox(const TaskSpaceBoundingBox& original, const int& /* <- trick used for locking mutex */)
            : ThreadSafeContainer(original)
            , m_samples(original.m_samples)
            , m_minimum(original.m_minimum)
            , m_maximum(original.m_maximum)
        {
        }

    public:
        //! Copy constructor (wrapper) (thread-safe)
        /*!
         * Locks the mutex of the original object for reading before calling the internal (protected) constructor. Unlocks original object afterwards.
         * \param [in] original Reference to original object.
         */
        TaskSpaceBoundingBox(const TaskSpaceBoundingBox& original)
            : TaskSpaceBoundingBox(original, original.lockForRead() /* <- lock mutex of original object first (for reading since also using "const") */)
        {
            original.unlock(); // Unlock mutex of original object after internal (protected) constructor has been called
        }

        //! Copy assignment operator (thread-safe)
        /*!
         * Uses own mutex and mutex of the reference object to guarantee thread-safe copying of members.
         *
         * \param [in] reference Reference to reference object.
         * \return Pointer to this instance.
         */
        TaskSpaceBoundingBox& operator=(const TaskSpaceBoundingBox& reference)
        {
            // Avoid self-assignment
            if (this == &reference)
                return *this;

            // Try to lock ourselves and reference (while avoiding deadlocks)
            while (true) { // Spinning
                lockForWrite(); // Lock ourselves for writing (blocking)
                if (reference.tryLockForRead() == true) // Try to lock reference for reading
                    break; // ...success -> stop spinning
                else
                    unlock(); // ...fail -> unlock ourselves to allow other threads to access us and thus resolve possible deadlocks
            }

            // Copy data
            ThreadSafeContainer::operator=(reference);
            m_samples = reference.m_samples;
            m_minimum = reference.m_minimum;
            m_maximum = reference.m_maximum;

            // Unlock reference object and ourselves
            reference.unlock();
            unlock();

            return *this;
        }

    protected:
        // Members
        // -------
        // Start protected data
        uint64_t m_samples = 0; //!< Total count of collected samples
        Eigen::Vector3d m_minimum = Eigen::Vector3d::Zero(); //!< Minimum (x, y, z) of the bounding box [m]
        Eigen::Vector3d m_maximum = Eigen::Vector3d::Zero(); //!< Maximum (x, y, z) of the bounding box [m]
        // End protected data

        // Operators
        // ---------
    public:
        //! Comparison operator: **equality** (**thread-safe**)
        bool operator==(const TaskSpaceBoundingBox& reference) const
        {
            bool equality = true;

            // Try to lock ourselves and reference (while avoiding deadlocks)
            while (true) { // Spinning
                lockForRead(); // Lock ourselves for reading (blocking)
                if (reference.tryLockForRead() == true) // Try to lock reference for reading
                    break; // ...success -> stop spinning
                else
                    unlock(); // ...fail -> unlock ourselves to allow other threads to access us and thus resolve possible deadlocks
            }

            // Compare members
            if (m_samples != reference.m_samples || //
                m_minimum != reference.m_minimum || //
                m_maximum != reference.m_maximum)
                equality = false;

            // Unlock reference object and ourselves
            reference.unlock();
            unlock();

            return equality;
        }

        //! Comparison operator: **inequality** (**thread-safe**)
        inline bool operator!=(const TaskSpaceBoundingBox& reference) const { return !(*this == reference); }

        // Setters (thread-safe)
        // ---------------------
    public:
        //! Resets the bounding box (**thread-safe**)
        void clear()
        {
            lockForWrite();
            m_samples = 0;
            m_minimum = Eigen::Vector3d::Zero();
            m_maximum = Eigen::Vector3d::Zero();
            unlock();
        }

        //! Adds the given sample to the bounding box (**thread-safe**)
        void addSample(const Eigen::Vector3d& samplePosition)
        {
            lockForWrite();

            // Check, if there have been samples already
            if (m_samples == 0) {
                // No samples so far -> use sample to initialize minimum and maximum
                m_minimum = samplePosition;
                m_maximum = samplePosition;
            } else {
                // There have been samples already -> update bounding box
                for (int i = 0; i < 3; i++) {
                    // Update minimum
                    if (m_minimum(i) > samplePosition(i))
                        m_minimum(i) = samplePosition(i);

                    // Update maximum
                    if (m_maximum(i) < samplePosition(i))
                        m_maximum(i) = samplePosition(i);
                }
            }

            // Update sample counter
            m_samples++;

            unlock();
        }

        // Getters (thread-safe)
        // ---------------------
    public:
        //! Counter of samples used to compute the bounding box \details **Thread-safe getter**
        inline uint64_t samples() const { return getProtectedData(m_samples); }
        //! Minimum (x,y,z) of the bounding box \details **Thread-safe getter**
        inline Eigen::Vector3d minimum() const { return getProtectedData(m_minimum); }
        //! Maximum (x,y,z) of the bounding box \details **Thread-safe getter**
        inline Eigen::Vector3d maximum() const { return getProtectedData(m_maximum); }

        // Postprocessing (thread-safe)
        // ----------------------------
    public:
        //! Creates a bounding box mesh for visualization (**thread-safe**)
        /*! \param [in] withColors If `true`, the mesh is colored according to the face normal */
        geometry::CGMesh createMesh(const bool& withColors) const
        {
            // Get parameters
            lockForRead();
            Eigen::Vector3d dimension = m_maximum - m_minimum;
            const Eigen::Vector3d center = 0.5 * (m_minimum + m_maximum);
            unlock();
            for (int i = 0; i < 3; i++)
                if (dimension(i) <= 0)
                    dimension(i) = 1e-6; // Avoid zero-dimension

            // Create mesh
            geometry::CGMesh mesh = geometry::CGMeshFactory::createBox(dimension(0), dimension(1), dimension(2));
            mesh.translate(center);

            // Setup color buffer
            if (withColors == true) {
                mesh.m_colorBuffer.resize(Eigen::NoChange, mesh.m_normalBuffer.cols());
                for (Eigen::Index j = 0; j < mesh.m_colorBuffer.cols(); j++)
                    for (int i = 0; i < 3; i++) // Iterate over all color channels (except alpha)
                        mesh.m_colorBuffer(i, j) = fabs(mesh.m_normalBuffer(i, j)) * 255.0;
            }

            // Pass back mesh
            return mesh;
        }

        // Serialization (not thread-safe)
        // -------------
    public:
        //! Compute serialized size of this object (in bytes)
        io::serialization::BinaryStreamSize computeBinaryStreamSize() const
        {
            io::serialization::BinaryStreamSize totalSize = sizeof(io::serialization::BinaryStreamSize); // Own header

            // Contribution of members
            totalSize += sizeof(m_samples); // Contribution of m_samples
            totalSize += 2 * sizeof(io::serialization::BinaryStreamSize) + 3 * sizeof(double); // Contribution of m_minimum
            totalSize += 2 * sizeof(io::serialization::BinaryStreamSize) + 3 * sizeof(double); // Contribution of m_maximum

            // Pass back total size of the stream
            return totalSize;
        }

    protected:
        // Serialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize serializePayload(io::serialization::BinaryStream& stream, const io::serialization::Endianness& endianness) const
        {
            stream.reserve(stream.size() + computeBinaryStreamSize());
            return io::serialization::serialize(stream, endianness, //
                m_samples, m_minimum, m_maximum);
        }

        // Deserialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize deSerializePayload(const io::serialization::BinaryStream& stream, const io::serialization::BinaryStreamSize& index, const io::serialization::BinaryStreamSize& payloadSize, const io::serialization::Endianness& endianness)
        {
            (void)payloadSize; // Not needed
            return io::serialization::deSerialize(stream, index, endianness, //
                m_samples, m_minimum, m_maximum);
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace analysis
} // namespace broccoli

#endif // HAVE_EIGEN3
