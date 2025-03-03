/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "SSVLineElement.hpp"
#include "SSVPointElement.hpp"
#include "SSVSegment.hpp"
#include "SSVSegmentDistance.hpp"
#include "SSVTriangleElement.hpp"

namespace broccoli {
namespace geometry {
    //! Collection of methods to evaluate the shortest distance between two SSVElement%s or SSVSegment%s
    /*!
     * \ingroup broccoli_geometry_ssv
     * This class contains static functions to evaluate every possible combination of two SSVElement%s as well as two SSVSegment%s.\n
     * The public evaluate() functions are calling protected core functions for the given types.
     * These five core functions contain the actual evaluation algorithms.
     *
     * \note
     * Original implementation by Poscher, Reinhold, "Effiziente Abstandsberechnungen mit Swept-Sphere Volumen für Echtzeit Kollisionsvermeidung in der Robotik", Technical University of Munich, 2020, Bachelor's thesis, https://mediatum.ub.tum.de/1580089
     */
    class SSVDistanceEvaluator {
    public:
        //! Computes the distance between two SSVPointElement%s (from first element to second element)
        /*!
         * \param [in] firstElement First element
         * \param [in] secondElement Second element
         * \param [out] distance Resulting minimum distance
         */
        static inline void evaluate(const SSVPointElement& firstElement, const SSVPointElement& secondElement, SSVElementDistance& distance)
        {
            evaluatePointToPoint(firstElement.point0(), firstElement.radius(), secondElement.point0(), secondElement.radius(), distance, false);
        }

        //! Computes the distance between a SSVPointElement and a SSVLineElement (from first element to second element)
        /*!
         * \param [in] firstElement First element
         * \param [in] secondElement Second element
         * \param [out] distance Resulting minimum distance
         */
        static inline void evaluate(const SSVPointElement& firstElement, const SSVLineElement& secondElement, SSVElementDistance& distance)
        {
            evaluatePointToLine(firstElement.point0(), firstElement.radius(), secondElement.point0(), secondElement.point1(), secondElement.edge0(), secondElement.radius(), distance, false);
        }

        //! Computes the distance between a SSVLineElement and a SSVPointElement (from first element to second element)
        /*!
         * \param [in] firstElement First element
         * \param [in] secondElement Second element
         * \param [out] distance Resulting minimum distance
         */
        static inline void evaluate(const SSVLineElement& firstElement, const SSVPointElement& secondElement, SSVElementDistance& distance)
        {
            evaluatePointToLine(secondElement.point0(), secondElement.radius(), firstElement.point0(), firstElement.point1(), firstElement.edge0(), firstElement.radius(), distance, true);
        }

        //! Computes the distance between a SSVPointElement and a SSVTriangleElement (from first element to second element)
        /*!
         * \param [in] firstElement First element
         * \param [in] secondElement Second element
         * \param [out] distance Resulting minimum distance
         */
        static inline void evaluate(const SSVPointElement& firstElement, const SSVTriangleElement& secondElement, SSVElementDistance& distance)
        {
            evaluatePointToTriangle(firstElement.point0(), firstElement.radius(), secondElement, distance, false);
        }

        //! Computes the distance between a SSVTriangleElement and a SSVPointElement (from first element to second element)
        /*!
         * \param [in] firstElement First element
         * \param [in] secondElement Second element
         * \param [out] distance Resulting minimum distance
         */
        static inline void evaluate(const SSVTriangleElement& firstElement, const SSVPointElement& secondElement, SSVElementDistance& distance)
        {
            evaluatePointToTriangle(secondElement.point0(), secondElement.radius(), firstElement, distance, true);
        }

        //! Computes the distance between two SSVLineElement%s (from first element to second element)
        /*!
         * \param [in] firstElement First element
         * \param [in] secondElement Second element
         * \param [out] distance Resulting minimum distance
         */
        static inline void evaluate(const SSVLineElement& firstElement, const SSVLineElement& secondElement, SSVElementDistance& distance)
        {
            evaluateLineToLine(firstElement.point0(), firstElement.edge0(), firstElement.radius(), secondElement.point0(), secondElement.edge0(), secondElement.radius(), distance, false);
        }

        //! Computes the distance between a SSVLineElement and a SSVTriangleElement (from first element to second element)
        /*!
         * \param [in] firstElement First element
         * \param [in] secondElement Second element
         * \param [out] distance Resulting minimum distance
         */
        static inline void evaluate(const SSVLineElement& firstElement, const SSVTriangleElement& secondElement, SSVElementDistance& distance)
        {
            evaluateLineToTriangle(firstElement.point0(), firstElement.point1(), firstElement.edge0(), firstElement.radius(), secondElement, distance, false);
        }

        //! Computes the distance between a SSVTriangleElement and a SSVLineElement (from first element to second element)
        /*!
         * \param [in] firstElement First element
         * \param [in] secondElement Second element
         * \param [out] distance Resulting minimum distance
         */
        static inline void evaluate(const SSVTriangleElement& firstElement, const SSVLineElement& secondElement, SSVElementDistance& distance)
        {
            evaluateLineToTriangle(secondElement.point0(), secondElement.point1(), secondElement.edge0(), secondElement.radius(), firstElement, distance, true);
        }

        //! Computes the distance between two SSVTriangleElement%s (from first element to second element)
        /*!
         * \param [in] firstElement First element
         * \param [in] secondElement Second element
         * \param [out] distance Resulting minimum distance
         */
        static inline void evaluate(const SSVTriangleElement& firstElement, const SSVTriangleElement& secondElement, SSVElementDistance& distance)
        {
            // Initialize helpers
            SSVElementDistance currentDistance;

            // Split the first triangle into three lines and compute the distance to the second triangle
            // Note: The order of the vertices and edges of the passed line element is crucial for correct evaluation!
            evaluateLineToTriangle(firstElement.point0(), firstElement.point1(), firstElement.edge0(), firstElement.radius(), secondElement, distance, false);
            evaluateLineToTriangle(firstElement.point1(), firstElement.point2(), firstElement.edge2(), firstElement.radius(), secondElement, currentDistance, false);
            distance.minimum(currentDistance);
            evaluateLineToTriangle(firstElement.point2(), firstElement.point0(), -firstElement.edge1(), firstElement.radius(), secondElement, currentDistance, false);
            distance.minimum(currentDistance);

            // Split the second triangle into three lines and compute the distance to the first triangle
            // Note: The order of the vertices and edges of the passed line element is crucial for correct evaluation!
            evaluateLineToTriangle(secondElement.point0(), secondElement.point1(), secondElement.edge0(), secondElement.radius(), firstElement, currentDistance, true);
            distance.minimum(currentDistance);
            evaluateLineToTriangle(secondElement.point1(), secondElement.point2(), secondElement.edge2(), secondElement.radius(), firstElement, currentDistance, true);
            distance.minimum(currentDistance);
            evaluateLineToTriangle(secondElement.point2(), secondElement.point0(), -secondElement.edge1(), secondElement.radius(), firstElement, currentDistance, true);
            distance.minimum(currentDistance);
        }

        //! Computes the distance between two SSVSegment%s (from first segment to second segment)
        /*!
         * This function computes the minimum distance between two SSV segments by evaluating the distances between all elements of both segments. The algorithm can use
         * bounding boxes of the segments to accelerate the distance computation. For this the user has to specify a maximum allowed (squared) bounding box distance
         * \p maximumBoundingBoxDistanceSquared. If this parameter is set, a full distance evaluation is skipped if the distance between the bounding boxes exceeds the
         * maximum. In this case the output parameter \p distance is not set and a total count of evaluated element pairs of 0 is returned.
         *
         * \param [in] firstSegment First segment
         * \param [in] secondSegment Second segment
         * \param [out] distance Resulting minimum distance
         * \param [in] maximumBoundingBoxDistanceSquared The maximum allowed (squared) distance between two bounding boxes which triggers a full evaluation (use a value < 0 to disable bounding box acceleration)
         * \return The total count of evaluated element pairs of the two SSVSegment%s. If 0, either one of the segments does not have any elements, or the bounding boxes have a distance greater than the specified maximum.
         *
         * \warning The output parameter \p distance is invalid if the total count of evaluated element pairs is 0!
         */
        static inline size_t evaluate(const SSVSegment& firstSegment, const SSVSegment& secondSegment, SSVSegmentDistance& distance, const double& maximumBoundingBoxDistanceSquared = -1.0)
        {
            // Check, if both segments are up to date
            assert(!firstSegment.updateRequired());
            assert(!secondSegment.updateRequired());

            // Step 1: compute distance between bounding boxes
            // -----------------------------------------------
            if (maximumBoundingBoxDistanceSquared >= 0) {
                // Compute (squared) distance between axis aligned bounding boxes
                double boundingBoxDistanceSquared = 0;
                for (Eigen::Index i = 0; i < 3; i++) {
                    if (secondSegment.globalBoundingBoxMinimum()(i) > firstSegment.globalBoundingBoxMaximum()(i)) {
                        const double x = secondSegment.globalBoundingBoxMinimum()(i) - firstSegment.globalBoundingBoxMaximum()(i);
                        boundingBoxDistanceSquared += (x * x);
                    } else if (firstSegment.globalBoundingBoxMinimum()(i) > secondSegment.globalBoundingBoxMaximum()(i)) {
                        const double x = firstSegment.globalBoundingBoxMinimum()(i) - secondSegment.globalBoundingBoxMaximum()(i);
                        boundingBoxDistanceSquared += (x * x);
                    }
                    // else -> intersection in this axis -> add 0
                }

                // Check, if a full evaluation can be skipped
                if (boundingBoxDistanceSquared > maximumBoundingBoxDistanceSquared)
                    return 0; // Distance between bounding boxes is greater than the specified maximum -> skip full evaluation
            }

            // Step 2: full evaluation
            // -----------------------
            // Initialize helpers
            SSVElementDistance currentDistance;
            size_t currentEvaluatedElementPairs = 0;
            size_t totalEvaluatedElementPairs = 0;

            // 9 different evaluations between two SSV segments
            for (uint8_t i = 0; i < 9; i++) {
                if (i == 0) // Point <-> Point
                    currentEvaluatedElementPairs = evaluateElementLists(firstSegment.globalPointElements(), secondSegment.globalPointElements(), currentDistance);
                else if (i == 1) // Point <-> Line
                    currentEvaluatedElementPairs = evaluateElementLists(firstSegment.globalPointElements(), secondSegment.globalLineElements(), currentDistance);
                else if (i == 2) // Point <-> Triangle
                    currentEvaluatedElementPairs = evaluateElementLists(firstSegment.globalPointElements(), secondSegment.globalTriangleElements(), currentDistance);
                else if (i == 3) // Line <-> Point
                    currentEvaluatedElementPairs = evaluateElementLists(firstSegment.globalLineElements(), secondSegment.globalPointElements(), currentDistance);
                else if (i == 4) // Line <-> Line
                    currentEvaluatedElementPairs = evaluateElementLists(firstSegment.globalLineElements(), secondSegment.globalLineElements(), currentDistance);
                else if (i == 5) // Line <-> Triangle
                    currentEvaluatedElementPairs = evaluateElementLists(firstSegment.globalLineElements(), secondSegment.globalTriangleElements(), currentDistance);
                else if (i == 6) // Triangle <-> Point
                    currentEvaluatedElementPairs = evaluateElementLists(firstSegment.globalTriangleElements(), secondSegment.globalPointElements(), currentDistance);
                else if (i == 7) // Triangle <-> Line
                    currentEvaluatedElementPairs = evaluateElementLists(firstSegment.globalTriangleElements(), secondSegment.globalLineElements(), currentDistance);
                else if (i == 8) // Triangle <-> Triangle
                    currentEvaluatedElementPairs = evaluateElementLists(firstSegment.globalTriangleElements(), secondSegment.globalTriangleElements(), currentDistance);

                // If at least one pair of elements got evaluated...
                if (currentEvaluatedElementPairs > 0) {
                    if (totalEvaluatedElementPairs == 0 || currentDistance.distance() < distance.distance())
                        distance = currentDistance;
                    totalEvaluatedElementPairs += currentEvaluatedElementPairs;
                }
            }

            // Set IDs
            distance.setFirstID(firstSegment.id());
            distance.setSecondID(secondSegment.id());

            // Return the total number of evaluated element pairs
            return totalEvaluatedElementPairs;
        }

        //! Computes an estimation of the cost of \ref evaluate() for two given segments
        /*!
         * The estimation is based on a benchmark of direct element evaluations (nanoseconds). The estimation does **not** consider acceleration through bounding boxes, since this
         * depends on the particular pose of the segments.
         *
         * \param [in] firstSegment The first segment in the pair
         * \param [in] secondSegment The second segment in the pair
         * \return An estimated cost value for this segment pair
         */
        static inline double estimateEvaluateCost(const SSVSegment& firstSegment, const SSVSegment& secondSegment)
        {
            return 2.3 * (firstSegment.localPointElements().size() * secondSegment.localPointElements().size()) + // <-- point <-> point
                4.4 * (firstSegment.localPointElements().size() * secondSegment.localLineElements().size() + firstSegment.localLineElements().size() * secondSegment.localPointElements().size()) + // <-- point <-> line and line <-> point
                13.4 * (firstSegment.localPointElements().size() * secondSegment.localTriangleElements().size() + firstSegment.localTriangleElements().size() * secondSegment.localPointElements().size()) + // <-- point <-> triangle and triangle <-> point
                17.0 * (firstSegment.localLineElements().size() * secondSegment.localLineElements().size()) + // <-- line <-> line
                42.0 * (firstSegment.localLineElements().size() * secondSegment.localTriangleElements().size() + firstSegment.localTriangleElements().size() * secondSegment.localLineElements().size()) + // <-- line <-> triangle and triangle <-> line
                261.0 * (firstSegment.localTriangleElements().size() * secondSegment.localTriangleElements().size()); // <-- triangle <-> triangle
        }

        // Core functions
        // --------------
    protected:
        //! Computes the distance between two point elements (from first point element to second point element)
        /*!
         * \f[ c = p_{1} - p_{0}\f]
         *
         * \param [in] firstElementPoint Point of first element
         * \param [in] firstElementRadius Radius of first element
         * \param [in] secondElementPoint Point of second element
         * \param [in] secondElementRadius Radius of second element
         * \param [out] distance Resulting minimum distance
         * \param [in] flip If `true`, the direction of the distance is flipped (swapping first and second element)
         */
        static inline void evaluatePointToPoint(const Eigen::Vector3d& firstElementPoint, const double& firstElementRadius, const Eigen::Vector3d& secondElementPoint, const double& secondElementRadius, SSVElementDistance& distance, const bool& flip)
        {
            distance.set(firstElementPoint, secondElementPoint, firstElementRadius, secondElementRadius, flip);
        }

        //! Computes the distance between a point element and a line element (from point element to line element)
        /*!
         * The connection of the two elements is given by
         * \f[ c = (v_{0} + s \cdot e) - p \f]
         * with \f$ s \in [0, 1] \f$ as a parameter of the line.
         *
         * Region definitions for a line (r0, r1 & r2):
         * --------------------------------------------
         * \verbatim
         +--------------------+
         |     |        |     |
         |  r0 |   r1   | r2  |
         |     |        |     |
         | ---v0---e----v1--- |
         |     |        |     |
         +--------------------+
         \endverbatim
         *
         * Condition                    | \f$ s \f$                   | Region
         * ---------                    | ---------                   | ------
         * \f$ w^{T} e \leq 0\f$        | 0                           | \f$ r0 \f$
         * \f$ w^{T} e \geq e^{T} e\f$  | 1                           | \f$ r2 \f$
         * else                         | \f$ w^{T} e / e^{T} e\f$    | \f$ r1 \f$
         *
         * with \f$ w = p - v_{0} \f$.
         *
         * \param [in] firstElementPoint Point of first element
         * \param [in] firstElementRadius Radius of first element
         * \param [in] secondElementFirstPoint First point of second element
         * \param [in] secondElementSecondPoint Second point of second element
         * \param [in] secondElementEdge Edge of second element
         * \param [in] secondElementRadius Radius of second element
         * \param [out] distance Resulting minimum distance
         * \param [in] flip If `true`, the direction of the distance is flipped (swapping first and second element)
         */
        static inline void evaluatePointToLine(const Eigen::Vector3d& firstElementPoint, const double& firstElementRadius, const Eigen::Vector3d& secondElementFirstPoint, const Eigen::Vector3d& secondElementSecondPoint, const Eigen::Vector3d& secondElementEdge, const double& secondElementRadius, SSVElementDistance& distance, const bool& flip)
        {
            const Eigen::Vector3d w = firstElementPoint - secondElementFirstPoint;
            const double w_dot_edge = w.dot(secondElementEdge);
            if (w_dot_edge < 0.0) {
                // Region 0
                distance.set(firstElementPoint, secondElementFirstPoint, firstElementRadius, secondElementRadius, flip);
            } else {
                // Region 1 or 2
                const double edge_dot_edge = secondElementEdge.dot(secondElementEdge);
                if (w_dot_edge > edge_dot_edge) {
                    // Region 2
                    distance.set(firstElementPoint, secondElementSecondPoint, firstElementRadius, secondElementRadius, flip);
                } else {
                    // Region 1
                    const double ratio = w_dot_edge / edge_dot_edge;
                    distance.set(firstElementPoint, secondElementFirstPoint + ratio * secondElementEdge, firstElementRadius, secondElementRadius, flip);
                }
            }
        }

        //! Computes the distance between a point element and a triangle element (from point element to triangle element)
        /*!
         * The connection of the two elements is given by
         * \f[ c = v_{0} + e_{0} \cdot s_{0} + e_{1} \cdot s_{1} - p \f]
         * with \f$ s_{0} \mbox{ and } s_{1} \in [0, 1]\f$ as parameters of the triangle.
         *
         * Region definitions for a triangle (r0..r6):
         * ------------------------------------------
         * \verbatim
         +---------------------+
         |       \ r5 /        |
         |        \  /         |
         |         v2          |
         |         /\          |
         |   r6  e1  e2   r4   |
         |       / r0 \        |
         | ----v0--e0--v1----- |
         |     /        \      |
         | r1 /    r2    \ r3  |
         |   /            \    |
         +---------------------+
         \endverbatim
         *
         * Region        | Function
         * ------        | --------
         * \f$ r0 \f$    | evaluatePointToPoint()
         * \f$ r1 \f$    | see *table for sections*
         * \f$ r2 \f$    | evaluatePointToLine()
         * \f$ r3 \f$    | see *table for sections*
         * \f$ r4 \f$    | evaluatePointToLine()
         * \f$ r5 \f$    | see *table for sections*
         * \f$ r6 \f$    | evaluatePointToLine()
         *
         * In case the closest point lies in region \f$ r1, r3 \mbox{ or } r5\f$, a second case distinction has to be made.
         *
         * Section definitions for a triangle (s0, s1 & s2):
         * ------------------------------------------------
         * \verbatim
         +---------------------+
         |      \       /      |
         |       \ s0  e1      |
         |   s1   \   /        |
         |         \ /         |
         | ---------v          |
         |          | triangle |
         |    s2    |   area   |
         |          e0         |
         |          |          |
         +---------------------+
         \endverbatim
         *
         * Section       | Function
         * -------       | --------
         * \f$ s0 \f$    | evaluatePointToLine()
         * \f$ s1 \f$    | evaluatePointToPoint()
         * \f$ s2 \f$    | evaluatePointToLine()
         *
         * \param [in] firstElementPoint Point of first element
         * \param [in] firstElementRadius Radius of first element
         * \param [in] secondElement Second element
         * \param [out] distance Resulting minimum distance
         * \param [in] flip If `true`, the direction of the distance is flipped (swapping first and second element)
         */
        static inline void evaluatePointToTriangle(const Eigen::Vector3d& firstElementPoint, const double& firstElementRadius, const SSVTriangleElement& secondElement, SSVElementDistance& distance, const bool& flip)
        {
            // Compute the region and section
            // ------------------------------
            uint8_t region = 0;
            uint8_t section = 0;
            Eigen::Vector3d connection = firstElementPoint - secondElement.point0();
            const Eigen::Vector2d identifier = secondElement.matrixS() * connection;
            if (identifier(0) >= 0.0) {
                // Region 0, 2, 3, 4
                if (identifier(1) >= 0.0) {
                    // Region 0, 4
                    if ((identifier(0) + identifier(1)) < 1.0)
                        region = 0;
                    else
                        region = 4;
                } else {
                    // Region 2, 3
                    if ((identifier(0) + identifier(1)) < 1.0)
                        region = 2;
                    else {
                        region = 3;
                        connection = firstElementPoint - secondElement.point1(); // Redefining the connection vector
                        const double edge0_dot_edge2 = -secondElement.edge0().dot(secondElement.edge2()); // Dot product of the joining edges
                        if (edge0_dot_edge2 > 0.0) // If angle between the joining edges is less than 90 degrees -> acute triangle, no case distinction required
                            section = 1;
                        else {
                            // Case distinction required
                            const double connection_dot_edge0 = -connection.dot(secondElement.edge0()); // Dot product of the first edge and the connection
                            if (connection_dot_edge0 > 0.0)
                                section = 0;
                            else {
                                // Section 1, 2
                                const double connection_dot_edge2 = connection.dot(secondElement.edge2()); // Dot product of the second edge and the connection
                                if (connection_dot_edge2 > 0.0)
                                    section = 2;
                                else
                                    section = 1;
                            }
                        }
                    }
                }
            } else {
                // Region 1, 5, 6
                if (identifier(1) >= 0.0) {
                    // Region 5, 6
                    if ((identifier(0) + identifier(1)) < 1.0)
                        region = 6;
                    else {
                        region = 5;
                        connection = firstElementPoint - secondElement.point2(); // Redefining the connection vector
                        const double edge1_dot_edge2 = secondElement.edge1().dot(secondElement.edge2()); // Dot product of the joining edges
                        if (edge1_dot_edge2 > 0.0) // If angle between the joining edges is less than 90 degrees -> acute triangle, no case distinction required
                            section = 1;
                        else {
                            // Case distinction required
                            const double connection_dot_edge2 = -connection.dot(secondElement.edge2()); // Dot product of the first edge and the connection
                            if (connection_dot_edge2 > 0.0)
                                section = 0;
                            else {
                                // Section 1, 2
                                const double connection_dot_edge1 = -connection.dot(secondElement.edge1()); // Dot product of the second edge and the connection
                                if (connection_dot_edge1 > 0.0)
                                    section = 2;
                                else
                                    section = 1;
                            }
                        }
                    }
                } else {
                    region = 1;
                    // (redefining the connection vector is not required)
                    const double edge0_dot_edge1 = secondElement.edge0().dot(secondElement.edge1()); // Dot product of the joining edges
                    if (edge0_dot_edge1 > 0.0) // If angle between the joining edges is less than 90 degrees -> acute triangle, no case distinction required
                        section = 1;
                    else {
                        // Case distinction required
                        const double connection_dot_edge1 = connection.dot(secondElement.edge1()); // Dot product of the first edge and the connection
                        if (connection_dot_edge1 > 0.0)
                            section = 0;
                        else {
                            // Section 1, 2
                            const double connection_dot_edge0 = connection.dot(secondElement.edge0()); // Dot product of the second edge and the connection
                            if (connection_dot_edge0 > 0.0)
                                section = 2;
                            else
                                section = 1;
                        }
                    }
                }
            }

            // Depending on the region and section the appropriate evaluate function gets called
            // ---------------------------------------------------------------------------------
            if (region == 0) {
                // Point inside the triangle
                const Eigen::Vector3d pointOnTriangle = secondElement.point0() + identifier(0) * secondElement.edge0() + identifier(1) * secondElement.edge1(); // Position vector of the point inside the triangle
                evaluatePointToPoint(firstElementPoint, firstElementRadius, pointOnTriangle, secondElement.radius(), distance, flip);
            } else if (region == 1) {
                if (section == 0) // Point to edge1
                    evaluatePointToLine(firstElementPoint, firstElementRadius, secondElement.point0(), secondElement.point2(), secondElement.edge1(), secondElement.radius(), distance, flip);
                else if (section == 1) // Point to vertex0
                    evaluatePointToPoint(firstElementPoint, firstElementRadius, secondElement.point0(), secondElement.radius(), distance, flip);
                else // section = 2 -> point to edge0
                    evaluatePointToLine(firstElementPoint, firstElementRadius, secondElement.point0(), secondElement.point1(), secondElement.edge0(), secondElement.radius(), distance, flip);
            } else if (region == 2) {
                // Point to edge0
                evaluatePointToLine(firstElementPoint, firstElementRadius, secondElement.point0(), secondElement.point1(), secondElement.edge0(), secondElement.radius(), distance, flip);
            } else if (region == 3) {
                if (section == 0) // Point to edge0
                    evaluatePointToLine(firstElementPoint, firstElementRadius, secondElement.point0(), secondElement.point1(), secondElement.edge0(), secondElement.radius(), distance, flip);
                else if (section == 1) // Point to vertex1
                    evaluatePointToPoint(firstElementPoint, firstElementRadius, secondElement.point1(), secondElement.radius(), distance, flip);
                else // section = 2 -> point to edge2
                    evaluatePointToLine(firstElementPoint, firstElementRadius, secondElement.point1(), secondElement.point2(), secondElement.edge2(), secondElement.radius(), distance, flip);
            } else if (region == 4) {
                // Point to edge2
                evaluatePointToLine(firstElementPoint, firstElementRadius, secondElement.point1(), secondElement.point2(), secondElement.edge2(), secondElement.radius(), distance, flip);
            } else if (region == 5) {
                if (section == 0) // Point to edge2
                    evaluatePointToLine(firstElementPoint, firstElementRadius, secondElement.point1(), secondElement.point2(), secondElement.edge2(), secondElement.radius(), distance, flip);
                else if (section == 1) // Point to vertex2
                    evaluatePointToPoint(firstElementPoint, firstElementRadius, secondElement.point2(), secondElement.radius(), distance, flip);
                else // section = 2 -> point to edge1
                    evaluatePointToLine(firstElementPoint, firstElementRadius, secondElement.point0(), secondElement.point2(), secondElement.edge1(), secondElement.radius(), distance, flip);
            } else // region = 6 -> point to edge1
                evaluatePointToLine(firstElementPoint, firstElementRadius, secondElement.point0(), secondElement.point2(), secondElement.edge1(), secondElement.radius(), distance, flip);
        }

        //! Computes the distance between two line elements (from first line element to second line element)
        /*!
         * The connection of the two elements is given by
         * \f[ c = w - s \cdot u + t \cdot v \f]
         * with:\n
         * \f$ u = p_{0,1} - p_{0,0} \mbox{ and } s \in [0, 1]\f$ for the first line element,\n
         * \f$ v = p_{1,1} - p_{1,0} \mbox{ and } t \in [0, 1]\f$ for the second line element and\n
         * \f$ w = p_{1,0} - p_{0,0} \f$ connecting the base points of the lines.
         *
         * \verbatim
         +---------------------+
         |           p11       |
         |           /         |
         |          /          |
         |         v           |
         |        /            |
         |       /             |
         |     p10             |
         |                     |
         |  p00-----u-----p01  |
         |                     |
         +---------------------+
         \endverbatim
         *
         * \param [in] firstElementFirstPoint First point of first element
         * \param [in] firstElementEdge Edge of first element
         * \param [in] firstElementRadius Radius of first element
         * \param [in] secondElementFirstPoint First point of second element
         * \param [in] secondElementEdge Edge of second element
         * \param [in] secondElementRadius Radius of second element
         * \param [out] distance Resulting minimum distance
         * \param [in] flip If `true`, the direction of the distance is flipped (swapping first and second element)
         */
        static inline void evaluateLineToLine(const Eigen::Vector3d& firstElementFirstPoint, const Eigen::Vector3d& firstElementEdge, const double& firstElementRadius, const Eigen::Vector3d& secondElementFirstPoint, const Eigen::Vector3d& secondElementEdge, const double& secondElementRadius, SSVElementDistance& distance, const bool& flip)
        {
            // Parameters of the two line elements
            double s = 0.0;
            double t = 0.0;

            // Connection vector of the base points
            const Eigen::Vector3d w = secondElementFirstPoint - firstElementFirstPoint;

            // All necessary dot products of both edges and the connection vector
            // u - edge of the first line
            // v - edge of the second line
            // q - connection vector
            const double uu = firstElementEdge.dot(firstElementEdge);
            const double vv = secondElementEdge.dot(secondElementEdge);
            const double uv = firstElementEdge.dot(secondElementEdge);
            const double uw = firstElementEdge.dot(w);
            const double vw = secondElementEdge.dot(w);

            // Denominator of the optimization problem
            const double N = uu * vv - uv * uv;

            // NOT parallel case
            // -----------------
            if (core::isZero(N / (uu * vv)) == false) { // N / (uu * vv) numerical more stable
                // Auxiliary numbers
                // xZ...enumerator, xN...denominator
                double sZ = vv * uw - uv * vw;
                double tZ = uv * uw - uu * vw;
                double sN = N;
                double tN = N;

                // First: evaluation based on second line parameter tZ
                // ---------------------------------------------------
                if (tZ < 0.0) {
                    sZ = uw;
                    tZ = 0.0;
                    sN = uu;
                } else if (tZ > tN) {
                    sZ = uv + uw;
                    tZ = tN;
                    sN = uu;
                }
                // else -> do nothing

                // Second: evaluation based on first line parameter sZ
                // ---------------------------------------------------
                if (sZ < 0.0) {
                    sZ = 0.0;
                    if (vw > 0.0)
                        tZ = 0.0;
                    else if (-vw > vv)
                        tZ = tN;
                    else {
                        tZ = -vw;
                        tN = vv;
                    }
                } else if (sZ > sN) {
                    sZ = sN;
                    if ((uv - vw) < 0.0)
                        tZ = 0.0;
                    else if ((uv - vw) > vv)
                        tZ = tN;
                    else {
                        tZ = uv - vw;
                        tN = vv;
                    }
                } // else -> do nothing

                // Calculation of both line parameters
                s = sZ / sN;
                t = tZ / tN;
            }

            // Parallel case
            // -------------
            else {
                // Auxiliary variable
                const double temp = uv + uw;

                if (uw < 0.0) { // p10 left of p00
                    if (temp < 0.0) { // p11 left of p00
                        s = 0.0;
                        if (uv < 0.0)
                            t = 0.0;
                        else
                            t = 1.0;
                    } else if (temp > uu) { // p11 right of p01
                        s = 0.5;
                        t = (uu - 2 * uw) / (2 * uv);
                    } else { // p11 over u
                        s = (vv + vw) / (2 * uv);
                        t = (uv - uw) / (2 * uv);
                    }
                } else if (uw > uu) { // p10 right if p01
                    if (temp < 0.0) { // p11 left of p00
                        s = 0.5;
                        t = (uu - 2 * uw) / (2 * uv);
                    } else if (temp > uu) { // p11 right of p01
                        s = 1.0;
                        if (uv < 0.0)
                            t = 1.0;
                        else
                            t = 0.0;
                    } else { // p11 over u
                        s = (uv + vv + vw) / (2 * uv);
                        t = (uv + uu - uw) / (2 * uv);
                    }
                } else { // p10 over u
                    if (temp < 0.0) { // p11 left of p00
                        s = vw / (2 * uv);
                        t = -uw / (2 * uv);
                    } else if (temp > uu) { // p11 right of p01
                        s = (uv + vw) / (2 * uv);
                        t = (uu - uw) / (2 * uv);
                    } else { // p11 over u
                        s = (vv + 2 * vw) / (2 * uv);
                        t = 0.5;
                    }
                }
            }

            // Setting the closest points in results
            distance.set(firstElementFirstPoint + s * firstElementEdge, secondElementFirstPoint + t * secondElementEdge, firstElementRadius, secondElementRadius, flip);
        }

        //! Computes the distance between a line element and a triangle element (from line element to triangle element)
        /*!
         * The connection of the two elements is given by
         * \f[ c = w + D \cdot x\f]
         * with:\n
         * \f$ w = v_{t0} - v_{l0}\f$ connecting the base points,\n\n
         * \f$ x = \left[\begin{array}{c} s_{0} \\ s_{1} \\ t\end{array}\right] \f$ vector of parameters for the triangle \f$(s)\f$ and the line \f$(t)\f$\n\n
         * \f$ D = \left[\begin{array}{ccc} e_{t0} & e_{t1} & -e_{l}\end{array}\right] \in \mathbb{R}^{3\times 3} \f$.
         *
         * Region definitions for a triangle (r0..r6):
         * ------------------------------------------
         * \verbatim
         +---------------------+
         |       \ r5 /        |
         |        \  /         |
         |         v2          |
         |         /\          |
         |   r6  e1  e2   r4   |
         |       / r0 \        |
         | ----v0--e0--v1----- |
         |     /        \      |
         | r1 /    r2    \ r3  |
         |   /            \    |
         +---------------------+
         \endverbatim
         *
         * \param [in] firstElementFirstPoint First point of first element
         * \param [in] firstElementSecondPoint Second point of first element
         * \param [in] firstElementEdge Edge of first element
         * \param [in] firstElementRadius Radius of first element
         * \param [in] secondElement Second element
         * \param [out] distance Resulting minimum distance
         * \param [in] flip If `true`, the direction of the distance is flipped (swapping first and second element)
         */
        static inline void evaluateLineToTriangle(const Eigen::Vector3d& firstElementFirstPoint, const Eigen::Vector3d& firstElementSecondPoint, const Eigen::Vector3d& firstElementEdge, const double& firstElementRadius, const SSVTriangleElement& secondElement, SSVElementDistance& distance, const bool& flip)
        {
            // Denominator of the optimization problem
            // If the dot product of the normal of the triangle and the edge of the line is 0, the elements are parallel
            const Eigen::Vector3d lineEdgeNormalized = firstElementEdge.normalized();
            const double N = fabs(secondElement.normal().dot(lineEdgeNormalized));

            // Parallel case
            if (core::isZero(N) == true) {
                SSVElementDistance currentDistance;

                // Evaluate the line element with every edge of the triangle
                evaluateLineToLine(firstElementFirstPoint, firstElementEdge, firstElementRadius, secondElement.point0(), secondElement.edge0(), secondElement.radius(), distance, flip);
                evaluateLineToLine(firstElementFirstPoint, firstElementEdge, firstElementRadius, secondElement.point0(), secondElement.edge1(), secondElement.radius(), currentDistance, flip);
                distance.minimum(currentDistance);
                evaluateLineToLine(firstElementFirstPoint, firstElementEdge, firstElementRadius, secondElement.point1(), secondElement.edge2(), secondElement.radius(), currentDistance, flip);
                distance.minimum(currentDistance);

                // Evaluate one vertex of the line with the triangle element
                evaluatePointToTriangle(firstElementFirstPoint, firstElementRadius, secondElement, currentDistance, flip);
                distance.minimum(currentDistance);
            }
            // NOT parallel case
            else {
                // Connection vector of the base points
                const Eigen::Vector3d w = secondElement.point0() - firstElementFirstPoint;

                // Parameters of the line and the triangle element
                double t = secondElement.normal().dot(w) / secondElement.normal().dot(firstElementEdge);
                Eigen::Vector2d s(0.0, 0.0);

                // First: evaluation based on line parameter t
                // -------------------------------------------
                if (t < 0.0) {
                    // t of line out of feasible region [0, 1] - set to 0
                    t = 0.0;
                    // Calculating parameter vector s for the triangle
                    s = secondElement.matrixS() * (firstElementFirstPoint - secondElement.point0());
                } else if (t > 1.0) {
                    // t of line out of feasible region [0, 1] - set to 1
                    t = 1.0;
                    // Calculating parameter vector s for the triangle
                    s = secondElement.matrixS() * (firstElementSecondPoint - secondElement.point0());
                } else {
                    // Auxillary matrix
                    Eigen::Matrix<double, 2, 3> crossEdgeMatrix;
                    crossEdgeMatrix.row(0) = firstElementEdge.cross(secondElement.edge1());
                    crossEdgeMatrix.row(1) = secondElement.edge0().cross(firstElementEdge);

                    // Calculating parameter vector s for the triangle
                    s = (crossEdgeMatrix * w) / (secondElement.normal().dot(firstElementEdge));
                }

                // Second: evaluation based on triangle parameter vector s
                // -------------------------------------------------------
                uint8_t region = 0;
                if (s(0) >= 0.0) { // Region 0, 2, 3, 4
                    if (s(1) >= 0.0) { // Region 0, 4
                        if ((s(0) + s(1)) < 1.0)
                            region = 0;
                        else
                            region = 4;
                    } else { // Region 2, 3
                        if ((s(0) + s(1)) < 1.0)
                            region = 2;
                        else
                            region = 3;
                    }
                } else { // Region 1, 5, 6
                    if (s(1) < 0.0)
                        region = 1;
                    else { // Region 5, 6
                        if ((s(0) + s(1)) >= 1.0)
                            region = 5;
                        else
                            region = 6;
                    }
                }

                // Depending on the region the appropriate evaluate functions get called
                // ---------------------------------------------------------------------
                if (region == 0) {
                    // Closest point lies inside the triangle
                    const Eigen::Vector3d pointOnLine = firstElementFirstPoint + t * firstElementEdge;
                    const Eigen::Vector3d pointOnTriangle = secondElement.point0() + s(0) * secondElement.edge0() + s(1) * secondElement.edge1();
                    evaluatePointToPoint(pointOnLine, firstElementRadius, pointOnTriangle, secondElement.radius(), distance, flip);
                } else if (region == 1) {
                    // Line to edge0 and edge 1
                    SSVElementDistance currentDistance;
                    evaluateLineToLine(firstElementFirstPoint, firstElementEdge, firstElementRadius, secondElement.point0(), secondElement.edge0(), secondElement.radius(), distance, flip);
                    evaluateLineToLine(firstElementFirstPoint, firstElementEdge, firstElementRadius, secondElement.point0(), secondElement.edge1(), secondElement.radius(), currentDistance, flip);
                    distance.minimum(currentDistance);
                } else if (region == 2) // Line to edge0
                    evaluateLineToLine(firstElementFirstPoint, firstElementEdge, firstElementRadius, secondElement.point0(), secondElement.edge0(), secondElement.radius(), distance, flip);
                else if (region == 3) {
                    // Line to edge0 and edge2
                    SSVElementDistance currentDistance;
                    evaluateLineToLine(firstElementFirstPoint, firstElementEdge, firstElementRadius, secondElement.point0(), secondElement.edge0(), secondElement.radius(), distance, flip);
                    evaluateLineToLine(firstElementFirstPoint, firstElementEdge, firstElementRadius, secondElement.point1(), secondElement.edge2(), secondElement.radius(), currentDistance, flip);
                    distance.minimum(currentDistance);
                } else if (region == 4) // Line to edge2
                    evaluateLineToLine(firstElementFirstPoint, firstElementEdge, firstElementRadius, secondElement.point1(), secondElement.edge2(), secondElement.radius(), distance, flip);
                else if (region == 5) {
                    // Line to edge1 and edge2
                    SSVElementDistance currentDistance;
                    evaluateLineToLine(firstElementFirstPoint, firstElementEdge, firstElementRadius, secondElement.point0(), secondElement.edge1(), secondElement.radius(), distance, flip);
                    evaluateLineToLine(firstElementFirstPoint, firstElementEdge, firstElementRadius, secondElement.point1(), secondElement.edge2(), secondElement.radius(), currentDistance, flip);
                    distance.minimum(currentDistance);
                } else // region = 6 -> line to edge1
                    evaluateLineToLine(firstElementFirstPoint, firstElementEdge, firstElementRadius, secondElement.point0(), secondElement.edge1(), secondElement.radius(), distance, flip);
            }
        }

        //! Template function for the distance computation of two lists of SSV elements
        /*!
         * This function iterates through both lists and evaluates any possible pair of SSV elements. The result is the smallest distance between one pair of these lists.
         * If \f$ n \f$ is the size of the first list and \f$ m \f$ of the second, the total number of evaluations is \f$ n * m \f$.
         *
         * \param [in] firstList First list of SSV elements
         * \param [in] secondList Second list of SSV elements
         * \param [out] distance Resulting minimum distance
         * \return The total number of evaluations \f$ n * m \f$
         */
        template <class FirstList, class SecondList>
        static inline size_t evaluateElementLists(const FirstList& firstList, const SecondList& secondList, SSVElementDistance& distance)
        {
            // Initialize helpers
            SSVElementDistance currentDistance;

            // Iteration through the first list
            for (size_t i = 0; i < firstList.size(); i++) {
                // Iteration through the second list
                for (size_t j = 0; j < secondList.size(); j++) {
                    // Calling the matching evaluate() function
                    evaluate(firstList[i], secondList[j], currentDistance);
                    // The first evaluated pair gets assigned to results, as well as every evaluated pair with a smaller distance
                    if (i == 0 && j == 0)
                        distance = currentDistance;
                    else
                        distance.minimum(currentDistance);
                }
            }

            // Total number of evaluated elements
            return firstList.size() * secondList.size();
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };
} // namespace geometry
} // namespace broccoli

#endif // HAVE_EIGEN3
