/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "SSVDistanceEvaluator.hpp"
#include "SSVSegment.hpp"
#include "SSVSegmentDistance.hpp"
#include <Eigen/StdVector>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

namespace broccoli {
namespace geometry {
    //! Database of a SSV **scene**
    /*!
     * \ingroup broccoli_geometry_ssv
     * This database contains all relevant (managed) input- and output data of a \ref SSVScene.
     *
     * \note
     * Original implementation by Poscher, Reinhold, "Effiziente Abstandsberechnungen mit Swept-Sphere Volumen für Echtzeit Kollisionsvermeidung in der Robotik", Technical University of Munich, 2020, Bachelor's thesis, https://mediatum.ub.tum.de/1580089
     */
    class SSVSceneData {
        // Custom hash functions
        // ---------------------
    private:
        //! Hasher for segment container
        struct SegmentContainerHash {
            size_t operator()(const SSVSegment::ID& id) const { return id; }
        };

        //! Computes a unique hash for a segment pair from the segment IDs
        /*!
         * \attention Flipping the IDs (first-segment <-> second-segment) results in the same hash! This prevents unnecessary duplicate computations.
         *
         * \param [in] firstSegmentID ID of the first segment
         * \param [in] secondSegmentID ID of the second segment
         * \return Unique hash for this pair
         */
        static inline uint64_t computePairHash(const SSVSegment::ID& firstSegmentID, const SSVSegment::ID& secondSegmentID)
        {
            // Force the smaller ID to be stored first
            if (firstSegmentID < secondSegmentID)
                return ((((uint64_t)firstSegmentID) << 32) | ((uint64_t)secondSegmentID));
            else
                return ((((uint64_t)secondSegmentID) << 32) | ((uint64_t)firstSegmentID));
        }

        //! Hasher for pair container
        struct PairContainerHash {
            size_t operator()(const std::pair<SSVSegment::ID, SSVSegment::ID>& pair) const
            {
                return computePairHash(pair.first, pair.second);
            }
        };

        //! Equality checker for pair container
        struct PairContainerEqualityChecker {
            bool operator()(const std::pair<SSVSegment::ID, SSVSegment::ID>& firstPair, const std::pair<SSVSegment::ID, SSVSegment::ID>& secondPair) const
            {
                return (firstPair.first == secondPair.first && firstPair.second == secondPair.second) || (firstPair.first == secondPair.second && firstPair.second == secondPair.first);
            }
        };

        // Type definitions
        // ----------------
    public:
        // Input
        using SegmentContainer = std::unordered_map<SSVSegment::ID, SSVSegment, SegmentContainerHash, std::equal_to<SSVSegment::ID>, Eigen::aligned_allocator<std::pair<const SSVSegment::ID, SSVSegment>>>; //!< Container for segments in the scene
        using PairContainer = std::unordered_set<std::pair<SSVSegment::ID, SSVSegment::ID>, PairContainerHash, PairContainerEqualityChecker>; //!< Container for segment pairs to evaluate
        // Output
        using DistanceList = std::vector<SSVSegmentDistance, Eigen::aligned_allocator<SSVSegmentDistance>>; //!< List of evaluated distances

        //! Container for a segment to update
        class SegmentToUpdate {
            // Construction
            // ------------
        public:
            SegmentToUpdate(SSVSegment& segment)
                : m_segment(segment)
                , m_expectedCost(segment.estimateUpdateCost())
            {
            }

            // Members
            // -------
        protected:
            // Input
            SSVSegment& m_segment; //!< Segment to be updated
            // Intermediate data
            double m_expectedCost = 0.0; //!< \copybrief expectedCost()

            // Getters
            // -------
        public:
            //! Expected cost for this operation
            inline const double& expectedCost() const { return m_expectedCost; }

            // Main Interface
            // --------------
        public:
            //! Main processing function for this operation
            inline void process() { m_segment.update(); }
        };
        using SegmentToUpdateList = std::vector<SegmentToUpdate>; //!< List of segments to update

        //! Container for a segment pair to evaluate
        class PairToEvaluate {
            // Construction
            // ------------
        public:
            PairToEvaluate(const SSVSegment& firstSegment, const SSVSegment& secondSegment, const double& maximumBoundingBoxDistanceSquared)
                : m_firstSegment(firstSegment)
                , m_secondSegment(secondSegment)
                , m_maximumBoundingBoxDistanceSquared(maximumBoundingBoxDistanceSquared)
                , m_expectedCost(SSVDistanceEvaluator::estimateEvaluateCost(firstSegment, secondSegment))
            {
            }

            // Members
            // -------
        protected:
            // Input
            const SSVSegment& m_firstSegment; //!< First segment in pair to evaluate
            const SSVSegment& m_secondSegment; //!< Second segment in pair to evaluate
            double m_maximumBoundingBoxDistanceSquared = -1.0; //!< Maximum (squared) distance of bounding boxes (used for acceleration, a value <0 indicates, that bounding box acceleration is disabled)
            // Intermediate data
            double m_expectedCost = 0.0; //!< \copybrief expectedCost()
            // Output
            size_t m_evaluatedElementPairs = 0; //!< \copybrief evaluatedElementPairs()
            SSVSegmentDistance m_distance; //!< \copybrief distance()

            // Getters
            // -------
        public:
            //! Expected cost for this operation
            inline const double& expectedCost() const { return m_expectedCost; }

            //! Total count of evaluated element pairs for this operation
            /*!
             * If 0 either
             *  * this operation has not been processed yet (call \ref process()), or
             *  * one of the segments does not contain any elements, or
             *  * the bounding boxes have a distance greater than the maximum.
             */
            inline const size_t& evaluatedElementPairs() const { return m_evaluatedElementPairs; }

            //! The evaluated distance (**Attention**: only valid if \ref evaluatedElementPairs() is >0)
            inline const SSVSegmentDistance& distance() const { return m_distance; }

            // Main Interface
            // --------------
        public:
            //! Main processing function for this operation
            inline void process() { m_evaluatedElementPairs = SSVDistanceEvaluator::evaluate(m_firstSegment, m_secondSegment, m_distance, m_maximumBoundingBoxDistanceSquared); }

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
        };
        using PairToEvaluateList = std::vector<PairToEvaluate, Eigen::aligned_allocator<PairToEvaluate>>; //!< List of pairs to evaluate

        // Construction
        // ------------
    public:
        //! Default constructor
        SSVSceneData() = default;

        //! Copy constructor
        SSVSceneData(const SSVSceneData& original)
            : m_segments(original.m_segments)
            , m_pairs(original.m_pairs)
            , m_segmentsToUpdateValid(false)
            , m_pairsToEvaluateValid(false)
            , m_maximumDistance(original.m_maximumDistance)
            , m_maximumBoundingBoxDistanceSquared(original.m_maximumBoundingBoxDistanceSquared)
            , m_distancesValid(original.m_distancesValid)
            , m_distances(original.m_distances)
            , m_minimumDistanceIndex(original.m_minimumDistanceIndex)
        {
        }

        //! Copy assignment operator
        SSVSceneData& operator=(const SSVSceneData& reference)
        {
            m_segments = reference.m_segments;
            m_pairs = reference.m_pairs;
            m_segmentsToUpdateValid = false;
            m_segmentsToUpdate.clear();
            m_pairsToEvaluateValid = false;
            m_pairsToEvaluate.clear();
            m_maximumDistance = reference.m_maximumDistance;
            m_maximumBoundingBoxDistanceSquared = reference.m_maximumBoundingBoxDistanceSquared;
            m_distancesValid = reference.m_distancesValid;
            m_distances = reference.m_distances;
            m_minimumDistanceIndex = reference.m_minimumDistanceIndex;
            return *this;
        }

        // Members
        // -------
    private:
        // Input
        SegmentContainer m_segments; //!< \copybrief segments()
        PairContainer m_pairs; //!< \copybrief pairs()
        // Intermediate data
        bool m_segmentsToUpdateValid = true; //!< Flag indicating, if \ref segmentsToUpdate() is valid (`true`) or has to be recomputed (`false`)
        SegmentToUpdateList m_segmentsToUpdate; //!< \copybrief segmentsToUpdate()
        bool m_pairsToEvaluateValid = true; //!< Flag indicating, if \ref pairsToEvaluate() is valid (`true`) or has to be recomputed (`false`)
        PairToEvaluateList m_pairsToEvaluate; //!< \copybrief pairsToEvaluate()
        // Acceleration
        double m_maximumDistance = -1.0; //!< \copybrief maximumDistance()
        double m_maximumBoundingBoxDistanceSquared = -1.0; //!< Maximum (squared) distance of bounding boxes (used for acceleration, a value <0 indicates, that bounding box acceleration is disabled)
        // Output
        bool m_distancesValid = true; //!< \copybrief distancesValid()
        DistanceList m_distances; //!< \copybrief distances()
        size_t m_minimumDistanceIndex = 0; //!< \copybrief minimumDistanceIndex()

        // Getters (segments)
        // -------
    public:
        //! Container for all segments in this scene
        inline const SegmentContainer& segments() const { return m_segments; }

        //! Checks, if the specified segment exists in the scene
        /*!
         * \param [in] segmentID The ID of the segment in consideration
         * \return `true`, if the segment exists in the scene, `false` otherwise
         */
        inline bool segmentExists(const SSVSegment::ID& segmentID) const { return (m_segments.find(segmentID) != m_segments.end()); }

        //! Returns a reference to the specified segment
        /*!
         * \warning Throws an exception if the specified segment does not exist in the scene (may be checked in advance with \ref segmentExists())
         *
         * \param [in] segmentID The ID of the segment in consideration
         * \return Reference to the segment
         */
        inline const SSVSegment& segment(const SSVSegment::ID& segmentID) const
        {
            const auto searchResult = m_segments.find(segmentID);
            if (searchResult == m_segments.end())
                throw std::runtime_error("SSVSceneData: Could not find segment with id '" + std::to_string(segmentID) + "'!");
            return searchResult->second;
        }

        //! Returns a list of all contained segment IDs
        /*!
         * \param [in] sort If `true` the list of IDs will be sorted in ascending order.
         * \return List of IDs of all segments currently contained in this scene
         */
        inline std::vector<SSVSegment::ID> segmentIDList(const bool& sort = false) const
        {
            std::vector<SSVSegment::ID> list;
            list.reserve(m_segments.size());
            for (auto iterator = m_segments.begin(); iterator != m_segments.end(); ++iterator)
                list.push_back(iterator->first);
            if (sort == true)
                std::sort(list.begin(), list.end());
            return list;
        }

        // Getters (pairs)
        // -------
    public:
        //! Container for segment pairs to evaluate
        inline const PairContainer& pairs() const { return m_pairs; }

        //! Checks, if the specified segment pair is registered for evaluation
        /*!
         * \param [in] firstSegmentID The ID of the first segment
         * \param [in] secondSegmentID The ID of the second segment
         * \return `true`, if the pair will be evaluated, `false` otherwise
         */
        inline bool pairExists(const SSVSegment::ID& firstSegmentID, const SSVSegment::ID& secondSegmentID) const { return (m_pairs.find({ firstSegmentID, secondSegmentID }) != m_pairs.end()); }

        // Getters (acceleration)
        // -------
    public:
        //! Maximum allowed distance (all greater distances will be discarded; used for acceleration; set to a value <0 to disable maximum)
        inline const double& maximumDistance() const { return m_maximumDistance; }

        // Getters (intermediate data)
        // -------
    protected:
        //! (Internal) list of segments for which an update is required
        inline SegmentToUpdateList& segmentsToUpdate() { return m_segmentsToUpdate; }

        //! (Internal) list of pairs to be evaluated
        inline PairToEvaluateList& pairsToEvaluate() { return m_pairsToEvaluate; }

        // Getters (distances)
        // -------
    public:
        //! Flag indicating, if \ref distances() is valid (`true`) or has to be recomputed (`false`)
        inline const bool& distancesValid() const { return m_distancesValid; }

        //! List of evaluated distances
        inline const DistanceList& distances() const
        {
            assert(distancesValid());
            return m_distances;
        }

        //! Index of element in \ref distances() with minimum distance value
        inline const size_t& minimumDistanceIndex() const
        {
            assert(distancesValid());
            return m_minimumDistanceIndex;
        }

        //! Reference to evaluated distance with minimum distance value
        /*! \warning The list of distances must be valid (see \ref distancesValid()) and contain at least one element) */
        inline const SSVSegmentDistance& minimumDistance() const
        {
            assert(distancesValid());
            if (m_minimumDistanceIndex >= m_distances.size())
                throw std::out_of_range("SSVSceneData: Could not obtain minimum distance! The list of distances does not contain enough distances!");
            return m_distances[m_minimumDistanceIndex];
        }

        // Setters (generic)
        // -------
    public:
        //! Clears all data of the scene
        inline void clear()
        {
            m_segments.clear();
            m_pairs.clear();
            m_segmentsToUpdateValid = true;
            m_segmentsToUpdate.clear();
            m_pairsToEvaluateValid = true;
            m_pairsToEvaluate.clear();
            m_distancesValid = true;
            m_distances.clear();
            m_minimumDistanceIndex = 0;
        }

        // Setters (segments)
        // -------
    private:
        //! Returns a **writable** reference to the specified segment
        /*!
         * \warning Throws an exception if the specified segment does not exist in the scene (may be checked in advance with \ref segmentExists())
         *
         * \param [in] segmentID The ID of the segment in consideration
         * \return Reference to the segment
         */
        inline SSVSegment& setSegment(const SSVSegment::ID& segmentID) { return const_cast<SSVSegment&>(static_cast<const SSVSceneData&>(*this).segment(segmentID)); }

    public:
        //! Adds the given segment to the scene
        /*!
         * \attention Skips, if a segment with the same ID is already in the scene
         *
         * \param [in] segment The segment to be added
         * \return `true` if the segment was added to the scene, `false` if a segment with the same ID already exists
         */
        inline bool addSegment(const SSVSegment& segment)
        {
            assert(segment.isValid());
            const auto insertResult = m_segments.insert({ segment.id(), segment });
            if (insertResult.second == true) {
                m_segmentsToUpdateValid = false;
                m_pairsToEvaluateValid = false;
                m_distancesValid = false;
            }
            return insertResult.second;
        }

        //! Sets the position of a specific segment in the scene
        /*!
         * \param [in] segmentID ID of the segment
         * \param [in] position The new position of the segment (see SSVSegment::position())
         *
         * \attention Throws an exception if the specified segment does not exist in the scene (may be checked in advance with \ref segmentExists())
         */
        inline void setSegmentPosition(const SSVSegment::ID& segmentID, const Eigen::Vector3d& position)
        {
            setSegment(segmentID).setPosition(position);
            m_segmentsToUpdateValid = false;
            m_distancesValid = false;
        }

        //! Sets the orientation of a specific segment in the scene
        /*!
         * \param [in] segmentID ID of the segment
         * \param [in] orientation The new orientation of the segment (see SSVSegment::orientation())
         *
         * \attention Throws an exception if the specified segment does not exist in the scene (may be checked in advance with \ref segmentExists())
         */
        inline void setSegmentOrientation(const SSVSegment::ID& segmentID, const Eigen::Matrix3d& orientation)
        {
            setSegment(segmentID).setOrientation(orientation);
            m_segmentsToUpdateValid = false;
            m_distancesValid = false;
        }

        //! Removes the specified segment from the scene
        /*!
         * \param [in] segmentID ID of the segment
         * \return `true` if the segment was removed, `false` if it did not exist
         */
        inline bool removeSegment(const SSVSegment::ID& segmentID)
        {
            if (m_segments.erase(segmentID) == 1) {
                m_segmentsToUpdateValid = false;
                m_pairsToEvaluateValid = false;
                m_distancesValid = false;
                return true;
            }
            return false;
        }

        // Setters (pairs)
        // -------
    public:
        //! Clears the list of pairs
        inline void clearPairs()
        {
            m_pairs.clear();
            m_pairsToEvaluateValid = true;
            m_pairsToEvaluate.clear();
            m_distancesValid = true;
            m_distances.clear();
            m_minimumDistanceIndex = 0;
        }

        //! Registers the given segment pair for evaluation
        /*!
         * \param [in] firstSegmentID ID of the first segment
         * \param [in] secondSegmentID ID of the second segment
         * \return `true` if the segment pair was added, `false` if the pair was already registered (or the IDs are the same which is not permitted)
         */
        inline bool addPair(const SSVSegment::ID& firstSegmentID, const SSVSegment::ID& secondSegmentID)
        {
            if (firstSegmentID == secondSegmentID) {
                assert(false);
                return false;
            }
            const auto insertResult = m_pairs.insert({ firstSegmentID, secondSegmentID });
            if (insertResult.second == true) {
                m_pairsToEvaluateValid = false;
                m_distancesValid = false;
            }
            return insertResult.second;
        }

        //! Deregisters the given segment pair from the list of segment pairs to evaluate
        /*!
         * \param [in] firstSegmentID ID of the first segment
         * \param [in] secondSegmentID ID of the second segment
         * \return `true` if the segment pair was removed, `false` if the pair was not registered
         */
        inline bool removePair(const SSVSegment::ID& firstSegmentID, const SSVSegment::ID& secondSegmentID)
        {
            if (m_pairs.erase({ firstSegmentID, secondSegmentID }) == 1) {
                m_pairsToEvaluateValid = false;
                m_distancesValid = false;
                return true;
            }
            return false;
        }

        //! Generates a pair list where each segment is compared with all other segments
        inline void setPairsAll()
        {
            m_pairs.clear();
            m_pairsToEvaluateValid = false;
            m_distancesValid = false;
            if (m_segments.size() < 2)
                return;
            for (auto firstSegment = m_segments.begin(); firstSegment != m_segments.end(); ++firstSegment) {
                for (auto secondSegment = std::next(firstSegment); secondSegment != m_segments.end(); ++secondSegment) {
                    m_pairs.insert({ firstSegment->first, secondSegment->first });
                }
            }
        }

        // Setters (acceleration)
        // -------
    public:
        //! Setter for \ref maximumDistance() - \copybrief maximumDistance()
        inline void setMaximumDistance(const double& maximumDistance)
        {
            if (maximumDistance < 0.0) {
                // Bounding box acceleration should be disabled
                if (m_maximumDistance >= 0.0) {
                    m_pairsToEvaluateValid = false;
                    m_distancesValid = false;
                }
                m_maximumDistance = maximumDistance;
                m_maximumBoundingBoxDistanceSquared = -1.0;
            } else {
                // Bounding box acceleration should be enabled
                if (maximumDistance != m_maximumDistance) {
                    m_pairsToEvaluateValid = false;
                    m_distancesValid = false;
                    m_maximumDistance = maximumDistance;
                    m_maximumBoundingBoxDistanceSquared = m_maximumDistance * m_maximumDistance;
                }
            }
        }

        // Setters (intermediate data)
        // -------
    protected:
        //! Re-computes list of segments to update
        /*! \return `true` if the list was recomputed, `false` otherwise (if the list was already up to date) */
        inline bool recomputeSegmentsToUpdate()
        {
            // Skip, if no update is required
            if (m_segmentsToUpdateValid == true)
                return false;

            // Collect segments which require an update
            m_segmentsToUpdate.clear();
            m_segmentsToUpdate.reserve(m_segments.size());
            for (auto iterator = m_segments.begin(); iterator != m_segments.end(); ++iterator) {
                if (iterator->second.updateRequired() == true)
                    m_segmentsToUpdate.push_back(iterator->second);
            }

            // Reset flag
            m_segmentsToUpdateValid = true;
            return true;
        }

        //! Re-computes list of pairs to evaluate
        /*! \return `true` if the list was recomputed, `false` otherwise (if the list was already up to date) */
        inline bool recomputePairsToEvaluate()
        {
            // Skip, if no update is required
            if (m_pairsToEvaluateValid == true)
                return false;

            // Collect pairs to evaluate (skip pairs with invalid segment indices)
            m_pairsToEvaluate.clear();
            m_pairsToEvaluate.reserve(m_pairs.size());
            for (auto iterator = m_pairs.begin(); iterator != m_pairs.end(); ++iterator) {
                // Search first segment
                const auto firstSegmentSearchResult = m_segments.find(iterator->first);
                if (firstSegmentSearchResult == m_segments.end())
                    assert(false); // First segment ID could not be found! Skip this pair...
                else {
                    // Search second segment
                    const auto secondSegmentSearchResult = m_segments.find(iterator->second);
                    if (secondSegmentSearchResult == m_segments.end())
                        assert(false); // Second segment ID could not be found! Skip this pair...
                    else
                        m_pairsToEvaluate.push_back(PairToEvaluate(firstSegmentSearchResult->second, secondSegmentSearchResult->second, m_maximumBoundingBoxDistanceSquared));
                }
            }

            // Reset flag
            m_pairsToEvaluateValid = true;
            return true;
        }

        // Setters (distances)
        // -------
    protected:
        //! Re-initializes distance list
        inline void clearDistances()
        {
            m_distancesValid = false;
            m_distances.clear();
            m_minimumDistanceIndex = 0;
        }

        //! Reserves memory in distance list
        inline void reserveDistances(const size_t& count) { m_distances.reserve(count); }

        //! Adds the given distance to the list of distances
        inline void addDistance(const SSVSegmentDistance& distance) { m_distances.push_back(distance); }

        //! Finalizes list of distances
        inline void finalizeDistances()
        {
            // Get element with minimum distance value
            m_minimumDistanceIndex = 0;
            if (m_distances.size() > 1) {
                for (size_t i = 1; i < m_distances.size(); i++) {
                    if (m_distances[i].distance() < m_distances[m_minimumDistanceIndex].distance())
                        m_minimumDistanceIndex = i;
                }
            }

            // Finalize list
            m_distancesValid = true;
        }

        // Helpers (generic)
        // -------
    public:
        //! Checks, if the scene (input) data is properly defined
        /*! \return `true`, if the scene data is valid, `false` otherwise. */
        inline bool isValid() const
        {
            // Iterate through all segments
            for (auto iterator = m_segments.begin(); iterator != m_segments.end(); ++iterator) {
                // Check, if segment is properly defined
                if (iterator->second.isValid() == false)
                    return false;
            }

            // Iterate through all pairs
            for (auto iterator = m_pairs.begin(); iterator != m_pairs.end(); ++iterator) {
                // Check, if both segments exist
                if (segmentExists(iterator->first) == false || segmentExists(iterator->second) == false)
                    return false;
            }

            // No errors found -> valid
            return true;
        }

        //! Removes all unused segments (which do not occur in the pair list) and invalid pairs (with non-existing segment IDs) from the scene
        /*! \return The total count of removed segments (first entry) and pairs (second entry) */
        inline std::pair<size_t, size_t> cleanUp()
        {
            // Initialize helpers
            std::unordered_map<SSVSegment::ID, bool, SegmentContainerHash> segments; // Map of segments [key=ID, value={false=unused, true=used}]
            for (auto iterator = m_segments.begin(); iterator != m_segments.end(); ++iterator)
                segments.insert({ iterator->first, false });
            std::vector<std::pair<SSVSegment::ID, SSVSegment::ID>> invalidPairs; // List of invalid pairs (to be removed)
            invalidPairs.reserve(m_pairs.size());

            // Run through all pairs
            for (auto iterator = m_pairs.begin(); iterator != m_pairs.end(); ++iterator) {
                // Check, if both segments exist
                if (segmentExists(iterator->first) == true && segmentExists(iterator->second) == true) {
                    // Pair is valid -> both segments are used
                    segments[iterator->first] = true;
                    segments[iterator->second] = true;
                } else // Pair is invalid
                    invalidPairs.push_back(*iterator);
            }

            // Remove unused segments
            size_t removedSegmentCount = 0;
            for (auto iterator = segments.begin(); iterator != segments.end(); ++iterator) {
                if (iterator->second == false) {
                    removeSegment(iterator->first);
                    removedSegmentCount++;
                }
            }

            // Remove invalid pairs
            for (size_t i = 0; i < invalidPairs.size(); i++)
                removePair(invalidPairs[i].first, invalidPairs[i].second);

            // Pass back total count of removed segments and pairs
            return std::pair<size_t, size_t>{ removedSegmentCount, invalidPairs.size() };
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };
} // namespace geometry
} // namespace broccoli

#endif // HAVE_EIGEN3
