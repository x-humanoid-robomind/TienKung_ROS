/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../io/encoding.hpp"
#include "../memory/SmartVector.hpp"
#include <Eigen/Dense>
#include <Eigen/StdList>
#include <assert.h>
#include <list>
#include <math.h>
#include <stdint.h>

namespace broccoli {
namespace geometry {
    /*!
     * \addtogroup broccoli_geometry
     * \{
     */

    //! Definition of vertex type for polygons (for convenience)
    using Polygon2DVertex = Eigen::Matrix<double, 2, 1>;

    //! Definition of aligned vertex allocator for polygons
    using Polygon2DVertexAllocator = Eigen::aligned_allocator<Polygon2DVertex>;

    //! Specification of result types for polygon algorithms
    enum class Polygon2DResult : uint8_t {
        UNKNOWN = 0, //!< Unknown result (this should **never** happen)
        SUCCESS, //!< Algorithm was successful
        ERROR_INVALID_POLYGON_MIN_THREE_POINTS, //!< An **error** occured: polygon is invalid (it has to contain at least 3 points)
        ERROR_INVALID_POLYGON_MIN_THREE_NONCOINCIDING_POINTS, //!< An **error** occured: polygon is invalid (it has to contain at least 3 non-coinciding points)
        ERROR_INVALID_POLYGON_MIN_THREE_NONCOINCIDING_POINTS_NOT_ON_LINE, //!< An **error** occured: polygon is invalid (it has to contain at least 3 non-coinciding points which do not lie on one line)
        ERROR_INVALID_POLYGON_NONZERO_AREA, //!< An **error** occured: polygon is invalid (area has to be non-zero)
        ERROR_INVALID_RAY_DIRECTION, //!< An **error** occured: ray direction is invalid
        ERROR_INVALID_LAMBDA_BOUNDARIES, //!< An **error** occured: boundaries for lambda are invalid
        RESULT_COUNT //!< Total count of elements
    };

    //! Returns the string representation of the given result type
    static inline std::string polygon2DResultString(const Polygon2DResult& result)
    {
        // Check result
        switch (result) {
        case Polygon2DResult::UNKNOWN:
            return "UNKNOWN";
        case Polygon2DResult::SUCCESS:
            return "SUCCESS";
        case Polygon2DResult::ERROR_INVALID_POLYGON_MIN_THREE_POINTS:
            return "ERROR_INVALID_POLYGON_MIN_THREE_POINTS";
        case Polygon2DResult::ERROR_INVALID_POLYGON_MIN_THREE_NONCOINCIDING_POINTS:
            return "ERROR_INVALID_POLYGON_MIN_THREE_NONCOINCIDING_POINTS";
        case Polygon2DResult::ERROR_INVALID_POLYGON_MIN_THREE_NONCOINCIDING_POINTS_NOT_ON_LINE:
            return "ERROR_INVALID_POLYGON_MIN_THREE_NONCOINCIDING_POINTS_NOT_ON_LINE";
        case Polygon2DResult::ERROR_INVALID_POLYGON_NONZERO_AREA:
            return "ERROR_INVALID_POLYGON_NONZERO_AREA";
        case Polygon2DResult::ERROR_INVALID_RAY_DIRECTION:
            return "ERROR_INVALID_RAY_DIRECTION";
        case Polygon2DResult::ERROR_INVALID_LAMBDA_BOUNDARIES:
            return "ERROR_INVALID_LAMBDA_BOUNDARIES";
        default: {
            assert(false);
            return "UNKNOWN";
        }
        }
    }

    //! Checks, if the given polygons are equivalent
    /*!
     * Compares the vertices of the first polygon with the vertices of the second polygon. If each vertex of the frist
     * polygon has an equivalent counterpart in the second polygon the polygons are considered as "equivalent". Note that
     * the order of vertices may differ in the first and second polygon (equivalence is not equality!).
     *
     * \warning Assumes, that both, the first and second polygon, do not have coinciding vertices!
     *
     * \tparam FirstPolygonType Type of the first polygon
     * \tparam SecondPolygonType Type of the second polygon
     *
     * \param [in] firstPolygon The first polygon to compare agains the second polygon.
     * \param [in] secondPolygon The second polygon to compare agains the first polygon.
     * \param [in] threshold The allowed threshold (norm of distance vector between two vertices)
     * \return `true`, if polygons are equivalent, `false` otherwise
     */
    template <class FirstPolygonType, class SecondPolygonType>
    static inline bool polygon2DIsEquivalent(const FirstPolygonType& firstPolygon, const SecondPolygonType& secondPolygon, const double& threshold = 1e-6)
    {
        // Check, if vertex count matches
        if (firstPolygon.m_vertices.size() != secondPolygon.m_vertices.size())
            return false;

        // Check, if there are any vertices to compare
        if (firstPolygon.m_vertices.size() == 0)
            return true; // No vertices -> equivalent!

        // Try to find "start-vertex" in second polygon
        int secondPolygonStartIndex = -1;
        for (size_t i = 0; i < secondPolygon.m_vertices.size(); i++) {
            if ((firstPolygon.m_vertices[0] - secondPolygon.m_vertices[i]).norm() <= threshold) {
                secondPolygonStartIndex = i;
                break;
            }
        }
        if (secondPolygonStartIndex == -1) {
            // Could not find a vertex in the second polygon which is equivalent to the first vertex in the first polygon
            return false;
        }

        // Iterate through vertices of first polygon (but skip first one - already checked!)
        for (size_t i = 1; i < firstPolygon.m_vertices.size(); i++) {
            // Get index of expected vertex in the second polygon
            int secondPolygonIndex = secondPolygonStartIndex + i;
            while (secondPolygonIndex >= (int)secondPolygon.m_vertices.size())
                secondPolygonIndex -= secondPolygon.m_vertices.size();

            // Compare vertices
            if ((firstPolygon.m_vertices[i] - secondPolygon.m_vertices[secondPolygonIndex]).norm() > threshold)
                return false;
        }

        // All vertices match -> equivalent
        return true;
    }

    //! Abstract representation of (simple) two-dimensional polygons
    /*!
     * The polygon is described as a (single) chain of vertices which form the border (counter-clockwise).
     * Holes in the polygon can not be represented by this class (restriction to "simple" polygons).
     * Class implements various geometric algorithms related to two-dimensional polygons.
     *
     * \tparam StaticVertexCount Count of statically allocated vertices (for best runtime preformance)
     */
    template <size_t StaticVertexCount = 3>
    class Polygon2D {
    public:
        //! Default constructor (reserves memory for the expected count of vertices)
        /*!
         * \param [in] expectedVertexCount Expected count of vertices used for allocating memory right at construction.
         */
        Polygon2D(const size_t& expectedVertexCount = StaticVertexCount)
        {
            // Allocate memory for the expected amount of vertices
            m_vertices.reserve(expectedVertexCount);
        }

        //! Comparison operator: **equality**
        bool operator==(const Polygon2D& reference) const
        {
            // Compare vertices (with tolerance)
            if (m_vertices.size() != reference.m_vertices.size())
                return false;
            for (size_t i = 0; i < m_vertices.size(); i++)
                if ((m_vertices[i] - reference.m_vertices[i]).squaredNorm() > 1e-6)
                    return false;

            // Otherwise -> equality
            return true;
        }

        //! Comparison operator: **inequality**
        inline bool operator!=(const Polygon2D& reference) const { return !(*this == reference); }

        //! Checks, if **this** polygon is equivalent to the given other polygon
        /*!
         * \tparam OtherPolygonType Type of the other polygon
         *
         * \param [in] other The other polygon to compare against this polygon
         * \param [in] threshold The allowed threshold (norm of distance vector between two vertices)
         * \return `true`, if polygons are equivalent, `false` otherwise
         */
        template <class OtherPolygonType>
        bool isEquivalent(const OtherPolygonType& other, const double& threshold = 1e-6) const { return polygon2DIsEquivalent<Polygon2D<StaticVertexCount>, OtherPolygonType>(*this, other, threshold); }

        // Members
        // -------
        memory::SmartVector<Polygon2DVertex, StaticVertexCount, Polygon2DVertexAllocator> m_vertices; //!< Vector of vertices describing the border (counter-clockwise chain)

        // Polygon algorithms
        // ------------------
        //! Compute perimeter
        /*!
         * Computes the perimeter (=length of edges)
         *
         * \par Assumptions/Requirements:
         * - contains at least 3 vertices
         *
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return Perimeter of the polygon
         */
        double perimeter(Polygon2DResult* const result = nullptr) const
        {
            // Check, if point set contains at least three points
            if (m_vertices.size() < 3) {
                // Error -> invalid polygon!
                if (result != nullptr)
                    *result = Polygon2DResult::ERROR_INVALID_POLYGON_MIN_THREE_POINTS;
                assert(false);
                return 0;
            }

            // Compute perimeter by summing up the length of all edges
            double perimeter = 0;
            for (size_t i = 0; i < m_vertices.size(); i++) {
                // Get index of next vertex (next neighbor)
                size_t nextVertexIndex = i + 1;
                if (i == m_vertices.size() - 1) // Connect last vertex with first vertex (close loop)
                    nextVertexIndex = 0;

                // Using "good old" Pythagoras
                double deltaX = m_vertices[nextVertexIndex](0) - m_vertices[i](0);
                double deltaY = m_vertices[nextVertexIndex](1) - m_vertices[i](1);
                perimeter += sqrt(deltaX * deltaX + deltaY * deltaY);
            }

            // Success
            if (result != nullptr)
                *result = Polygon2DResult::SUCCESS;

            // Pass back perimeter
            return perimeter;
        }

        //! Compute enclosed area (signed)
        /*!
         * Computes the enclosed area of the polygon. The sign indicates the ordering of the vertices:
         * - positive sign: counter-clockwise around positive "z-axis"
         * - negative sign: clockwise around positive "z-axis"
         *
         * \par Assumptions/Requirements:
         * - contains at least 3 vertices
         * - non-intersecting polygon
         *
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return Enclosed area (signed)
         */
        double areaSigned(Polygon2DResult* const result = nullptr) const
        {
            // Check, if point set contains at least three points
            if (m_vertices.size() < 3) {
                // Error -> invalid polygon!
                if (result != nullptr)
                    *result = Polygon2DResult::ERROR_INVALID_POLYGON_MIN_THREE_POINTS;
                assert(false);
                return 0;
            }

            // Compute area of 2D polygon according to http://paulbourke.net/geometry/polygonmesh/ (accessed on 08.09.2018 15:34:38)
            double area = 0;
            for (size_t i = 0; i < m_vertices.size(); i++) {
                // Get index of next vertex (next neighbor)
                size_t nextVertexIndex = i + 1;
                if (i == m_vertices.size() - 1) // Connect last vertex with first vertex (close loop)
                    nextVertexIndex = 0;

                // Compute contribution of "segement" to overall area
                area += m_vertices[i](0) * m_vertices[nextVertexIndex](1) - m_vertices[nextVertexIndex](0) * m_vertices[i](1);
            }

            // Success
            if (result != nullptr)
                *result = Polygon2DResult::SUCCESS;

            // Return signed area
            return (double)0.5 * area;
        }

        //! Compute enclosed area (unsigned)
        /*!
         * Same as \ref areaSigned() but with a forced positive result
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return Enclosed area (always positive)
         */
        double area(Polygon2DResult* const result = nullptr) const
        {
            // Pass through positive signed area
            return fabs(areaSigned(result));
        }

        //! Compute the centroid
        /*!
         * Computes the centroid of the polygon.
         *
         * \par Assumptions/Requirements:
         * - contains at least 3 vertices
         * - non-intersecting polygon
         *
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return The centroid of the polygon as vertex or (0, 0) in case of an error
         */
        Polygon2DVertex centroid(Polygon2DResult* const result = nullptr) const
        {
            // Initialize result
            Polygon2DVertex centroid;
            centroid << 0, 0;

            // Check, if point set contains at least three points
            if (m_vertices.size() < 3) {
                // Error -> invalid polygon!
                if (result != nullptr)
                    *result = Polygon2DResult::ERROR_INVALID_POLYGON_MIN_THREE_POINTS;
                assert(false);
                return centroid;
            }

            // Get signed area as part of the computation
            double area = areaSigned();
            if (area == 0) {
                // Error -> invalid polygon!
                if (result != nullptr)
                    *result = Polygon2DResult::ERROR_INVALID_POLYGON_NONZERO_AREA;
                return centroid;
            }

            // Compute components according to http://paulbourke.net/geometry/polygonmesh/ (accessed on 08.09.2018 15:34:38)
            for (size_t i = 0; i < m_vertices.size(); i++) {
                // Get index of next vertex (next neighbor)
                size_t nextVertexIndex = i + 1;
                if (i == m_vertices.size() - 1) // Connect last vertex with first vertex (close loop)
                    nextVertexIndex = 0;

                // Compute contribution of "segment" to overall centroid
                double segmentArea = m_vertices[i](0) * m_vertices[nextVertexIndex](1) - m_vertices[nextVertexIndex](0) * m_vertices[i](1);
                centroid += (m_vertices[i] + m_vertices[nextVertexIndex]) * segmentArea;
            }

            // Success
            if (result != nullptr)
                *result = Polygon2DResult::SUCCESS;

            // Post-multiply to get final result
            return (double)1.0 / (6.0 * area) * centroid;
        }

        //! Recompute vertices of polygon as convex hull of the vertices before calling the function
        /*!
         * Uses Graham's scan algorithm (see below for details). If the computed vertex count is zero, the input was invalid.
         *
         * \par Assumptions/Requirements:
         * - given point set consists of at least three points which do not lie on one line
         *
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         */
        bool reComputeConvexHull(Polygon2DResult* const result = nullptr)
        {
            // Check, if point set contains at least three points
            if (m_vertices.size() < 3) {
                // Error -> invalid input set!
                if (result != nullptr)
                    *result = Polygon2DResult::ERROR_INVALID_POLYGON_MIN_THREE_POINTS;
                assert(false);
                m_vertices.clear();
                return false;
            }

            /*! Graham's Scan
             *  -------------
             * (*according to Cormen et. al, "Introduction to Algorithms", 3rd edition, 2009, The MIT press, Cambridge, Massachusetts, p. 1030, ISBN 978-0-262-03384-8*)
             *
             * \par Assumptions:
             * -# at least 3 points,
             * -# points do not lie all on one line
             *
             * **Note:** coinciding points are removed automatically
             */

            // Initialize helpers
            double epsilon = 1e-6; // Tolerance for dealing with imperfect machine precision

            // Create input point-set (take all vertices, which are in the polygon right now)
            std::list<Polygon2DVertex, Polygon2DVertexAllocator> inputPointSet;
            for (size_t i = 0; i < m_vertices.size(); i++)
                inputPointSet.push_back(m_vertices[i]);

            // Step 1: find point with the minimum y-coordinate (and minimum x-coordinate, if there is more than one)
            // ------------------------------------------------
            auto rootPointIterator = inputPointSet.begin(); // Assume first point to be the root point, then compare to others
            auto inputPointIterator = inputPointSet.begin();
            ++inputPointIterator; // Start with second element in set
            while (inputPointIterator != inputPointSet.end()) {
                // Compare y-coordinate
                if ((*inputPointIterator)(1) < (*rootPointIterator)(1) - epsilon) {
                    // Considererd point is lower -> set this as new root point
                    rootPointIterator = inputPointIterator;

                    // Switch to next candidate
                    ++inputPointIterator;
                } else if (fabs((*inputPointIterator)(1) - (*rootPointIterator)(1)) <= epsilon) {
                    // Points are (approximately) on the same height -> take the one with minimum x-coordinate
                    if ((*inputPointIterator)(0) < (*rootPointIterator)(0) - epsilon) {
                        // Considererd point is on the left side -> set this as new root point
                        rootPointIterator = inputPointIterator;

                        // Switch to next candidate
                        ++inputPointIterator;
                    } else if (fabs((*inputPointIterator)(0) - (*rootPointIterator)(0)) <= epsilon) {
                        // Considered point coincides (approximately) with root point -> remove from list and switch to next candidate
                        inputPointSet.erase(inputPointIterator++);
                    } else {
                        // ...considered point is on the right side -> switch to next candidate
                        ++inputPointIterator;
                    }
                } else {
                    // ...considered point is higher -> switch to next candidate
                    ++inputPointIterator;
                }
            }

            // Check, if input point set still contains at least three points
            // (some points may have been removed due to coincidence (multiple root point candidates))
            if (inputPointSet.size() < 3) {
                // Error -> empty convex hull
                if (result != nullptr)
                    *result = Polygon2DResult::ERROR_INVALID_POLYGON_MIN_THREE_NONCOINCIDING_POINTS;
                m_vertices.clear();
                return false;
            }

            // Step 2: sort non-root points by polar angle in counterclockwise order around root point
            // ---------------------------------------------------------------------------------------
            // Note: if two points have (approximately) the same angle, take the one with larger radius
            std::list<Eigen::Matrix<double, 4, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 4, 1>>> pointFan; // Sorted point "fan"
            Eigen::Matrix<double, 4, 1> newFanPoint; // New point to be added to the point fan (helper variable)
            // Indices:
            // [0] ... x-coordinate (cartesian)
            // [1] ... y-coordiante (cartesian)
            // [2] ... radius^2 (polar relative to root point)
            // [3] ... angle (polar relative to root point)

            // Pass through input set and sort by polar angle relative to root point (counterclockwise)
            inputPointIterator = inputPointSet.begin();
            while (inputPointIterator != inputPointSet.end()) {
                // Copy cartesian coordinates
                newFanPoint(0) = (*inputPointIterator)(0);
                newFanPoint(1) = (*inputPointIterator)(1);

                // Check point type
                if (inputPointIterator == rootPointIterator) {
                    // ...root point -> add to beginning of list
                    newFanPoint(2) = 0; // Zero radius for root element (no distance to itself)
                    newFanPoint(3) = -1; // Negative angle for comparisons (keeps root point at the beginning of the list, since all other points have positive angle)
                    pointFan.push_front(newFanPoint);
                } else {
                    // ...not-root point -> compute polar coordinates to root point
                    double deltaX = (*inputPointIterator)(0) - (*rootPointIterator)(0);
                    double deltaY = (*inputPointIterator)(1) - (*rootPointIterator)(1);
                    newFanPoint(2) = deltaX * deltaX + deltaY * deltaY; // Radius^2
                    newFanPoint(3) = atan2(deltaY, deltaX); // Angle

                    // Project angles: as the root point is the lowest (or lowest leftmost) point, the angle is expected to be in the upper half plane
                    // relative to the root point. Thus the angle has to be positive. However, if the computed angle is negative this is due to numeric
                    // errors which we can easily eliminate by projection.
                    if (newFanPoint(3) < 0) {
                        // ...angle is negative -> project point to x-axis passing through root point
                        if (newFanPoint(3) < -M_PI_2) {
                            // ...quadrant III -> project to negative x-axis
                            newFanPoint(3) = M_PI_2;
                        } else {
                            // ...quadrant IV -> project to positive x-axis
                            newFanPoint(3) = 0;
                        }
                    }

                    // Add point to fan at the right place (sorted!)
                    if (pointFan.size() == 0) {
                        // ...empty point fan -> add as first element
                        pointFan.push_front(newFanPoint);
                    } else {
                        // ...non-empty point fan -> search for correct place to insert
                        auto pointFanIterator = pointFan.begin();
                        while (pointFanIterator != pointFan.end()) { // Pass through all points in point fan
                            // Compare angle to existing point in fan
                            if (newFanPoint(3) < (*pointFanIterator)(3) - epsilon) {
                                // ...angle of new point is lower than existing point in fan -> insert new point before this one
                                pointFan.insert(pointFanIterator, newFanPoint);
                                break;
                            } else if (fabs(newFanPoint(3) - (*pointFanIterator)(3)) < epsilon) {
                                // ...angle of new point is (approximately) equal to existing point in fan -> check radius
                                if (newFanPoint(2) > (*pointFanIterator)(2) + epsilon) {
                                    // ...considered point is farther away as existing point in fan -> replace
                                    (*pointFanIterator) = newFanPoint;
                                    break;
                                } else {
                                    // ...new point is closer to root point (or equal distance) -> skip
                                    break;
                                }
                            } else {
                                // ...angle of new point is greater than existing point in fan -> switch to next element in fan
                                ++pointFanIterator;
                            }
                        }

                        // Check, if we reached the end of the point fan
                        if (pointFanIterator == pointFan.end()) {
                            // ...new point has greater angle as all points in the fan -> add to the end
                            pointFan.push_back(newFanPoint);
                        }
                    }
                }

                // Switch to next input point
                inputPointIterator++;
            }

            // Check, if point fan contains at least three points
            // (some points may have been removed)
            if (pointFan.size() < 3) {
                // Error -> empty convex hull
                if (result != nullptr)
                    *result = Polygon2DResult::ERROR_INVALID_POLYGON_MIN_THREE_NONCOINCIDING_POINTS_NOT_ON_LINE;
                m_vertices.clear();
                return false;
            }

            // Step 3: Compute actual convex hull
            // ----------------------------------
            std::list<Eigen::Matrix<double, 4, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 4, 1>>> convexHull; // Convex hull "stack"
            // (indices same as for point fan)

            // Add first three points to the "stack"
            auto pointFanIterator = pointFan.begin();
            convexHull.push_back(*(pointFanIterator)); // Add p0 to stack
            ++pointFanIterator;
            convexHull.push_back(*(pointFanIterator)); // Add p1 to stack
            ++pointFanIterator;
            convexHull.push_back(*(pointFanIterator)); // Add p2 to stack
            ++pointFanIterator;

            // Pass through all remaining points in point fan
            while (pointFanIterator != pointFan.end()) {
                // Remove existing points in stack which are not part of the convex hull (we have to make a left turn with each new point)
                while (convexHull.size() > 2 /* first two points are always part of the convex hull! */) {
                    // Get references to last and next to last element in stack
                    auto lastElementInStack = convexHull.end();
                    lastElementInStack--;
                    auto nextToLastElementInStack = lastElementInStack;
                    nextToLastElementInStack--;

                    // Compute rotation by cross product
                    // Vector "1": from last element in stack to next-to-last element in stack
                    // Vector "2": from last element in stack to new element
                    double x1 = (*nextToLastElementInStack)(0) - (*lastElementInStack)(0);
                    double y1 = (*nextToLastElementInStack)(1) - (*lastElementInStack)(1);
                    double x2 = (*pointFanIterator)(0) - (*lastElementInStack)(0);
                    double y2 = (*pointFanIterator)(1) - (*lastElementInStack)(1);
                    if (x1 * y2 - y1 * x2 < -epsilon) {
                        // ...left turn -> all existing points in stack seem to be part of the convex hull -> do not remove anything
                        break;
                    } else {
                        // ...right turn, or straight line -> topmost point in stack is not part of convex hull -> remove it and check previous points too (loop)
                        convexHull.pop_back();
                    }
                }

                // Add new point to hull
                convexHull.push_back(*pointFanIterator);

                // Switch to next point in fan
                ++pointFanIterator;
            }

            // Step 4: Overrite vertices of polygon with convex hull
            // -----------------------------------------------------
            m_vertices.clear();
            m_vertices.reserve(convexHull.size());
            auto convexHullIterator = convexHull.begin();
            while (convexHullIterator != convexHull.end()) {
                // Extract cartesian coordinates
                Polygon2DVertex newPoint;
                newPoint(0) = (*convexHullIterator)(0);
                newPoint(1) = (*convexHullIterator)(1);

                // Add vertex to list
                m_vertices.push_back(newPoint);

                // Switch to next element in convex hull
                convexHullIterator++;
            }

            // Success
            if (result != nullptr)
                *result = Polygon2DResult::SUCCESS;
            return true;
        }

        //! Check, if given point lies inside of the polygon ("intersects")
        /*!
         * Checks, if point lies within the polygon or coincides with a corner point of the polygon.
         *
         * \par Assumptions/Requirements:
         * - polygon contains at least 3 vertices
         * - non-intersecting polygon
         * - points ordered counter-clockwise
         *
         * \param [in] pointToCheck The 2D point to check for
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true`, if point lies within polygon, `false` if not or in case of an error
         */
        bool checkPointIntersection(const Polygon2DVertex& pointToCheck, Polygon2DResult* const result = nullptr) const
        {
            // Check, if point set contains at least three points
            if (m_vertices.size() < 3) {
                // Error -> invalid polygon!
                if (result != nullptr)
                    *result = Polygon2DResult::ERROR_INVALID_POLYGON_MIN_THREE_POINTS;
                assert(false);
                return false;
            }

            // Check, if point is inside according to http://paulbourke.net/geometry/polygonmesh/ (accessed on 08.09.2018 15:34:38)
            /*
             * "Signed angle method" by Philippe Reverdy
             * Create rays from given point to all vertices of the polygon. Sum up all angles between the "rays".
             * If the sum equals 2*PI the point lies within, if the sum equals 0 the point lies outside of the polygon.
             */
            double angle = 0;
            for (size_t i = 0; i < m_vertices.size(); i++) {
                // Get index of next vertex (next neighbor)
                size_t nextVertexIndex = i + 1;
                if (i == m_vertices.size() - 1) // Connect last vertex with first vertex (close loop)
                    nextVertexIndex = 0;

                // OWN MODIFICATION OF REFERENCE ALGORITHM:
                // Check, if given point coincides with the base vertex. In this case we count the point to be "inside" and abort all further computations
                if (m_vertices[i].isApprox(pointToCheck)) {
                    // Success
                    if (result != nullptr)
                        *result = Polygon2DResult::SUCCESS;
                    return true;
                }

                // Compute angle between both rays
                double deltaAngle = atan2(m_vertices[nextVertexIndex](1) - pointToCheck(1), m_vertices[nextVertexIndex](0) - pointToCheck(0)) - atan2(m_vertices[i](1) - pointToCheck(1), m_vertices[i](0) - pointToCheck(0));
                while (deltaAngle > M_PI)
                    deltaAngle -= 2.0 * M_PI;
                while (deltaAngle < -M_PI)
                    deltaAngle += 2.0 * M_PI;

                // Accumulate
                angle += deltaAngle;
            }

            // Check sum to get the result
            if (fabs(angle) < M_PI) {
                // Success
                if (result != nullptr)
                    *result = Polygon2DResult::SUCCESS;
                return false; // Consider 0...M_PI as outside
            } else {
                // Success
                if (result != nullptr)
                    *result = Polygon2DResult::SUCCESS;
                return true; // Consider M_PI...2*M_PI as inside
            }
        }

        //! Check, if given ray intersects with polygon
        /*!
         * Tests all edges of the polygon for intersection with the given ray. The case of coinciding edge<->ray also
         * counts as intersection.
         *
         * \par Assumptions/Requirements:
         * - polygon contains at least 3 vertices
         * - given \p rayDirection is not \f$[0,\,0]^T\f$
         * - \p minimumLambda has to be lower or equal to \p maximumLambda
         *
         * \param [in] rayOrigin Origin of ray
         * \param [in] rayDirection Direction of ray / second point on ray relative to origin (does not have to be normalized)
         * \param [in] minimumLambda Only count \f$ \lambda\f$'s which are equal or greater than this parameter (ignored, if \p minimumLambda = \p maximumLambda)
         * \param [in] maximumLambda Only count \f$ \lambda\f$'s which are equal or lower than this parameter (ignored, if \p minimumLambda = \p maximumLambda)
         * \param [in] onlyFirst Abort algorithm after the first intersection was found (to avoid unneccessary computations)
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return In case of one or more intersections, all \f$ \lambda\f$'s are stored in this vector (`intersectionpoint` = \p rayOrigin + `lambda` * \p rayDirection), empty if there are no intersections
         */
        std::vector<double> checkRayIntersections(const Polygon2DVertex& rayOrigin, const Polygon2DVertex& rayDirection, const double& minimumLambda, const double& maximumLambda, const bool& onlyFirst, Polygon2DResult* const result = nullptr) const
        {
            // Initialize return value
            std::vector<double> lambdas;
            lambdas.reserve(m_vertices.size()); // Pre-allocate memory for better performance (over estimate)
            // (edge count = maximum count of intersections)

            // Check, if point set contains at least three points
            if (m_vertices.size() < 3) {
                // Error -> invalid polygon!
                if (result != nullptr)
                    *result = Polygon2DResult::ERROR_INVALID_POLYGON_MIN_THREE_POINTS;
                assert(false);
                return lambdas;
            }

            // Check, if ray direction is valid
            if (rayDirection(0) == 0 && rayDirection(1) == 0) {
                // Error -> invalid ray direction!
                if (result != nullptr)
                    *result = Polygon2DResult::ERROR_INVALID_RAY_DIRECTION;
                assert(false);
                return lambdas;
            }

            // Check, if lambda boundaries are valid
            if (minimumLambda > maximumLambda) {
                // Error -> invalid boundaries for lambda!
                if (result != nullptr)
                    *result = Polygon2DResult::ERROR_INVALID_LAMBDA_BOUNDARIES;
                assert(false);
                return lambdas;
            }

            // Initialize helpers
            double epsilon = 1e-6; // Tolerance for various numeric comparisons

            // Iterate over all edges
            for (size_t i = 0; i < m_vertices.size(); i++) {
                // Get index of next vertex (next neighbor)
                size_t nextVertexIndex = i + 1;
                if (i == m_vertices.size() - 1) // Connect last vertex with first vertex (close loop)
                    nextVertexIndex = 0;

                // Compute intersection
                // ray(lambda) = rayOrigin + lambda * rayDirection, lambda arbitrary
                // edge(eta) = edgeOrigin + eta * edgeDirection, eta in [0, 1]
                // Solve linear system of equations (A*x=b) with x = [lambda, eta]^T
                const Polygon2DVertex& edgeOrigin = m_vertices[i];
                Polygon2DVertex edgeDirection = m_vertices[nextVertexIndex] - m_vertices[i];
                Polygon2DVertex b = edgeOrigin - rayOrigin;
                double detA = rayDirection(1) * edgeDirection(0) - rayDirection(0) * edgeDirection(1);

                // Check singularity of system
                if (fabs(detA) > epsilon) {
                    // ...non-singular (->non-parallel)
                    double lambda = (double)1.0 / detA * (edgeDirection(0) * b(1) - edgeDirection(1) * b(0));
                    double eta = (double)1.0 / detA * (rayDirection(0) * b(1) - rayDirection(1) * b(0));

                    // Check if the intersection occurs within the limits of the edge ("inside" the edge)
                    if (eta >= 0 && eta <= 1) {
                        // ...intersection is inside
                        // Check, if lambda is within bounds
                        if (minimumLambda == maximumLambda || (lambda >= minimumLambda && lambda <= maximumLambda)) {
                            lambdas.push_back(lambda);
                            if (onlyFirst == true) {
                                // Success
                                if (result != nullptr)
                                    *result = Polygon2DResult::SUCCESS;
                                return lambdas;
                            }
                        }
                    }
                    // else: intersection is outside -> do nothing
                } else {
                    // ...singular (->parallel). In this case we check the minimal distance between ray and edge. If the distance is near to zero we
                    // also compute the infinite set of intersecting lambdas (bounded by [lambda1, lambda2])
                    // To compute lambda1 and lambda2 we have to solve TWO linear systems of equations:
                    // --> System 1: edgeOrigin = rayOrigin + lambda1*rayDirection + xi*rayNormal ---> (C1*x1=b1) with x1 = [lambda1, xi]^T
                    // --> System 2: edgeEnd = rayOrigin + lambda2*rayDirection + xi*rayNormal ---> (C2*x2=b2) with x2 = [lambda2, xi]^T
                    // Note: b1 is the same as b (see above) and xi is the same for system 1 and 2

                    // Grab edge end
                    const Polygon2DVertex& edgeEnd = m_vertices[nextVertexIndex];

                    // Compute ray normal (=perpendicular (normalized) to rayDirection)
                    Polygon2DVertex rayNormal;
                    rayNormal(0) = rayDirection(1);
                    rayNormal(1) = -rayDirection(0);
                    rayNormal.normalize();

                    // Compute minimum distance from edge to ray (=xi) and lambda1/2
                    Polygon2DVertex& b1 = b;
                    Polygon2DVertex b2 = edgeEnd - rayOrigin;
                    double detC = rayDirection(0) * rayNormal(1) - rayDirection(1) * rayNormal(0); // Same for system 1 and 2 (Note: detC cannot be zero, since this would imply that rayDirection = [0, 0]^T which we already checked at the beginning)
                    double lambda1 = (double)1.0 / detC * (rayNormal(1) * b1(0) - rayNormal(0) * b1(1));
                    double lambda2 = (double)1.0 / detC * (rayNormal(1) * b2(0) - rayNormal(0) * b2(1));
                    double xi = (double)1.0 / detC * (rayDirection(0) * b1(1) - rayDirection(1) * b1(0)); // Same for system 1 and 2

                    // Check, if minimum distance is below an epsilon
                    if (fabs(xi) < epsilon) {
                        // ...edge lies on ray -> count as intersection
                        // Check, if there is a lambda within the bounds
                        double lambdaStart = lambda1;
                        double lambdaEnd = lambda2;
                        if (lambdaStart > lambdaEnd) // Sort lambda bounds
                        {
                            // Switch...
                            lambdaStart = lambda2;
                            lambdaEnd = lambda1;
                        }

                        // Check, if boundaries are given
                        if (minimumLambda == maximumLambda) {
                            // ...no -> every lambda is valid so use the smaller one
                            lambdas.push_back(lambdaStart);
                            if (onlyFirst == true) {
                                // Success
                                if (result != nullptr)
                                    *result = Polygon2DResult::SUCCESS;
                                return lambdas;
                            }
                        } else {
                            // ...yes -> compute intersection of both intervalls [minimumLambda, maximumLambda] and [lambdaStart, lambdaEnd]
                            if (minimumLambda > lambdaEnd || lambdaStart > maximumLambda) {
                                // ...empty set -> there is no valid lambda -> do not count as intersection
                                // --> switch to next edge
                            } else {
                                // ...intersection of intervals is not empty -> compute intersection bounds
                                double lambdaLowerBound = std::max(minimumLambda, lambdaStart);
                                //double lambdaUpperBound = std::min(maximumLambda,lambdaEnd);

                                // Use lower bound (minimal lambda) as solution
                                lambdas.push_back(lambdaLowerBound);
                                if (onlyFirst == true) {
                                    // Success
                                    if (result != nullptr)
                                        *result = Polygon2DResult::SUCCESS;
                                    return lambdas;
                                }
                            }
                        }
                    }
                    // else: edge does not lie on ray and is parallel -> no intersection -> switch to next edge
                }
            }

            // Remove double solutions
            if (lambdas.size() > 1) {
                std::vector<double> finalLambdas;
                finalLambdas.reserve(lambdas.size());
                for (size_t i = 0; i < lambdas.size(); i++) {
                    // Search for duplicates
                    bool isDuplicate = false;
                    for (size_t j = 0; j < finalLambdas.size(); j++)
                        if (lambdas[i] == finalLambdas[j])
                            isDuplicate = true;

                    // Only add to result, if this lambda is not alread contained in the current solution set
                    if (isDuplicate == false)
                        finalLambdas.push_back(lambdas[i]);
                }

                // Success
                if (result != nullptr)
                    *result = Polygon2DResult::SUCCESS;
                return finalLambdas;
            } else {
                // Success
                if (result != nullptr)
                    *result = Polygon2DResult::SUCCESS;
                return lambdas;
            }
        }

        //! Check, if given line intersects with polygon
        /*!
         * Tests all edges of the polygon for intersection with the given line. The case of coinciding edge<->line also
         * counts as intersection.
         * \warning If the line lies completely within the polygon this does **not** count as intersecion! In this case you
         * can use \ref checkPointIntersection() with \p lineStart or \p lineEnd instead.
         *
         * \par Assumptions/Requirements:
         * - polygon contains at least 3 vertices
         * - given line has non-zero length
         *
         * \param [in] lineStart Start point of line
         * \param [in] lineEnd End point of line
         * \param [in] onlyFirst Abort algorithm after the first intersection was found (to avoid unneccessary computations)
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return In case of one or more intersections, all \f$ \lambda\f$'s are stored in this vector (`intersectionpoint` = \p rayOrigin + `lambda` * \p rayDirection), empty if there are no intersections
         */
        std::vector<double> checkLineIntersections(const Polygon2DVertex& lineStart, const Polygon2DVertex& lineEnd, const bool& onlyFirst, Polygon2DResult* const result = nullptr) const
        {
            // Compute ray intersection (consider given line as ray and only count intersection in the bound lambda=[0, 1], i.e. within lineStart and lineEnd)
            return checkRayIntersections(lineStart, lineEnd - lineStart, 0, 1, onlyFirst, result);
        }

        //! Check, if this polygon intersects with another polygon
        /*!
         * \warning Complete inclusion **does** count as intersection!
         *
         * \warning This is a very simple implementation. There are algorithms which perform much better, especially
         * if the polygons are of a special type (e.g. convex)
         *
         * \par Assumptions/Requirements:
         * - both polygons contain at least 3 vertices
         *
         * \tparam OtherPolygonType Type of the other polygon
         *
         * \param [in] otherPolygon The other polygon to test the intersection with
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true`, if there is an intersection, `false` otherwise or in case of an error
         */
        template <class OtherPolygonType>
        bool checkPolygonIntersection(const OtherPolygonType& otherPolygon, Polygon2DResult* const result = nullptr) const
        {
            // Check, if point set contains at least three points (for both polygons)
            if (m_vertices.size() < 3 || otherPolygon.m_vertices.size() < 3) {
                // Error -> invalid polygon!
                if (result != nullptr)
                    *result = Polygon2DResult::ERROR_INVALID_POLYGON_MIN_THREE_POINTS;
                assert(false);
                return false;
            }

            // Step 1: check if any corner point of this polygon lies within the other polygon
            for (size_t i = 0; i < m_vertices.size(); i++)
                if (otherPolygon.checkPointIntersection(m_vertices[i], result) == true)
                    return true;

            // Step 2: check if any corner point of the other polygon lies within this polygon
            for (size_t i = 0; i < otherPolygon.m_vertices.size(); i++)
                if (checkPointIntersection(otherPolygon.m_vertices[i], result) == true)
                    return true;

            // Step 3: check if any edge of this polygon intersects with any edge of the other polygon
            for (size_t i = 0; i < m_vertices.size(); i++) {
                // Get index of next vertex (next neighbor)
                size_t nextVertexIndex = i + 1;
                if (i == m_vertices.size() - 1) // Connect last vertex with first vertex (close loop)
                    nextVertexIndex = 0;

                // Check for intersection
                if (otherPolygon.checkLineIntersections(m_vertices[i], m_vertices[nextVertexIndex], true, result).size() > 0)
                    return true;
            }

            // Otherwise: there is no intersection
            if (result != nullptr)
                *result = Polygon2DResult::SUCCESS;
            return false;
        }

        //! Project a given point to the border of the polygon
        /*!
         * Computes the point on the border of the polygon which is nearest to the given point.
         *
         * \par Assumptions/Requirements:
         * - polygon contains at least 3 vertices
         *
         * \param [in] point The point which should be projected
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return Projected point or same point in case of an error
         */
        Polygon2DVertex projectPointToBorder(const Polygon2DVertex& point, Polygon2DResult* const result = nullptr) const
        {
            // Initialize return value
            Polygon2DVertex projectedPoint = point;

            // Check, if point set contains at least three points
            if (m_vertices.size() < 3) {
                // Error -> invalid polygon!
                if (result != nullptr)
                    *result = Polygon2DResult::ERROR_INVALID_POLYGON_MIN_THREE_POINTS;
                assert(false);
                return projectedPoint;
            }

            // Initialize helpers
            double epsilon = 1e-6; // Tolerance for various numeric comparisons

            // Initial guess for the solution: take first vertex in polygon
            projectedPoint = m_vertices[0];
            double minimalDistance = (point - projectedPoint).norm();

            // Pass through all edges to find the minimum distance point
            for (size_t i = 0; i < m_vertices.size(); i++) {
                // Get index of next vertex (next neighbor)
                size_t nextVertexIndex = i + 1;
                if (i == m_vertices.size() - 1) // Connect last vertex with first vertex (close loop)
                    nextVertexIndex = 0;

                // Compute projection of point to line
                // Solve linear system of equations: point = lineStart + lambda*lineDirection + xi*lineNormal --> A*x=b with x=[lambda, xi]
                const Polygon2DVertex& edgeStart = m_vertices[i];
                const Polygon2DVertex& edgeEnd = m_vertices[nextVertexIndex];
                Polygon2DVertex edgeDirection = m_vertices[nextVertexIndex] - m_vertices[i];
                Polygon2DVertex edgeNormal;
                edgeNormal(0) = edgeDirection(1);
                edgeNormal(1) = -edgeDirection(0);
                edgeNormal.normalize();
                Polygon2DVertex b = point - edgeStart;
                double detA = edgeDirection(0) * edgeNormal(1) - edgeDirection(1) * edgeNormal(0);

                // Check, if system is non-singular
                if (fabs(detA) > epsilon) {
                    // ...non-singular
                    double lambda = (double)1.0 / detA * (edgeNormal(1) * b(0) - edgeNormal(0) * b(1));
                    //double xi = (double)1.0 / detA * (edgeDirection(0) * b(1) - edgeDirection(1) * b(0));

                    // Compute nearest point on ray through line (has to be clipped to the start and end of the edge)
                    Polygon2DVertex nearestPoint = edgeStart + lambda * edgeDirection;

                    // Clip nearest point to start/end of edge
                    if (lambda < 0)
                        nearestPoint = edgeStart;
                    if (lambda > 1)
                        nearestPoint = edgeEnd;

                    // Compute distance from nearest point on edge to given point
                    double distance = (point - nearestPoint).norm();

                    // Check, if this point is closer than the closest point so far
                    if (distance < minimalDistance) {
                        // Take this as the new nearest point
                        projectedPoint = nearestPoint;
                        minimalDistance = distance;
                    }
                }
                // else: singular -> implies, that lineDirection is of zero length -> skip this edge
            }

            // Success
            if (result != nullptr)
                *result = Polygon2DResult::SUCCESS;

            // Pass back projected point
            return projectedPoint;
        }

        //! Project a point to the polygon
        /*!
         * Computes the point in the polygon (area) which is nearest to the given point. In case the given
         * point already lies within the polygon the given point is passed back (unchanged).
         *
         * \par Assumptions/Requirements:
         * - polygon contains at least 3 vertices
         *
         * \param [in] point The point which should be projected
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return Projected point or same point in case of an error
         */
        Polygon2DVertex projectPoint(const Polygon2DVertex& point, Polygon2DResult* const result = nullptr) const
        {
            // Check, if point is inside the polygon
            if (checkPointIntersection(point, result) == true)
                return point; // ...inside -> pass unchanged point back
            else
                return projectPointToBorder(point, result); // ...outside -> project point to border of polygon
        }

        //! Compute convex polygon as intersection of two given convex polygons
        /*!
         * Computes the intersection of two given convex polygons and stores the result in the own
         * vertex set.
         *
         * \remark The intersection of two convex polygons is also convex!
         *
         * \par Assumptions/Requirements:
         * - both polygons contain at least 3 vertices and are convex
         *
         * \tparam FirstPolygonType Type of the first polygon
         * \tparam SecondPolygonType Type of the second polygon
         *
         * \param [in] firstConvexPolygon The first polygon to intersect with the second one.
         * \param [in] secondConvexPolygon The second polygon to intersect with the first one.
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true`, if a valid intersection was computed, `false` if there is no intersection or in case of an error
         */
        template <class FirstPolygonType, class SecondPolygonType>
        bool computeAsConvexPolygonIntersection(const FirstPolygonType& firstConvexPolygon, const SecondPolygonType& secondConvexPolygon, Polygon2DResult* const result = nullptr)
        {
            // Assume success
            if (result != nullptr)
                *result = Polygon2DResult::SUCCESS;

            // Reset own vertex set
            m_vertices.clear();
            m_vertices.reserve(3 * firstConvexPolygon.m_vertices.size() + secondConvexPolygon.m_vertices.size()); // Pre-allocate memory for better performance (over estimate)
            // (all points from first and second polygon + maximal two intersections per edge for one polygon)

            // Check, if point sets of both input polygons contains at least three points
            if (firstConvexPolygon.m_vertices.size() < 3 || secondConvexPolygon.m_vertices.size() < 3) {
                // Error -> invalid polygon!
                if (result != nullptr)
                    *result = Polygon2DResult::ERROR_INVALID_POLYGON_MIN_THREE_POINTS;
                assert(false);
                return false;
            }

            // Step 1: Add all points of the first polygon which lie in the second polygon to the own vertex set
            for (size_t i = 0; i < firstConvexPolygon.m_vertices.size(); i++)
                if (secondConvexPolygon.checkPointIntersection(firstConvexPolygon.m_vertices[i]) == true)
                    m_vertices.push_back(firstConvexPolygon.m_vertices[i]);

            // Step 2: Add all points of the second polygon which lie in the first polygon to the own vertex set
            for (size_t i = 0; i < secondConvexPolygon.m_vertices.size(); i++)
                if (firstConvexPolygon.checkPointIntersection(secondConvexPolygon.m_vertices[i]) == true)
                    m_vertices.push_back(secondConvexPolygon.m_vertices[i]);

            // Step 3: Check all edges of the first polygon for an intersection with the second polygon and add real intersection points to own vertex set
            for (size_t i = 0; i < firstConvexPolygon.m_vertices.size(); i++) {
                // Get index of next vertex (next neighbor)
                size_t nextVertexIndex = i + 1;
                if (i == firstConvexPolygon.m_vertices.size() - 1) // Connect last vertex with first vertex (close loop)
                    nextVertexIndex = 0;

                // Check for intersections
                std::vector<double> lambdas = secondConvexPolygon.checkLineIntersections(firstConvexPolygon.m_vertices[i], firstConvexPolygon.m_vertices[nextVertexIndex], false);
                for (size_t j = 0; j < lambdas.size(); j++) {
                    // Add intersection point to own vertex set
                    m_vertices.push_back(firstConvexPolygon.m_vertices[i] + lambdas[j] * (firstConvexPolygon.m_vertices[nextVertexIndex] - firstConvexPolygon.m_vertices[i]));
                }
            }

            // Step 4: Compute convex hull of own vertex set (the intersection of two convex polygons is convex too)
            if (m_vertices.size() >= 3)
                return reComputeConvexHull();
            else
                return false;
        }

        // Encoding
        // --------
        //! Encode member data as XML element and add it to the specified stream
        /*!
         * \param [in,out] stream The stream to which the member data should be appended to.
         * \param [in] XMLIndentationLevel Level of indentation in XML format (used to add spaces)
         * \param [in] XMLTabWhiteSpaces Count of whitespaces corresponding to a tab.
         * \param [in] numericFormat C-format specifier for numeric values.
         * \return Count of appended characters (elements in stream).
         */
        io::encoding::CharacterStreamSize encodeToXML(io::encoding::CharacterStream& stream, const size_t& XMLIndentationLevel, const size_t& XMLTabWhiteSpaces, const std::string& numericFormat = "%.7g") const
        {
            io::encoding::CharacterStreamSize addedElements = 0;

            // Start XML element
            for (size_t i = 0; i < XMLIndentationLevel * XMLTabWhiteSpaces; i++)
                addedElements += io::encoding::encode(stream, (char)' ');
            addedElements += io::encoding::encode(stream, "<Polygon2D");

            // Write vertex data
            addedElements += io::encoding::encode(stream, " VertexData=\"");
            for (size_t i = 0; i < m_vertices.size(); i++) {
                if (i > 0)
                    addedElements += io::encoding::encode(stream, (char)';');
                for (size_t j = 0; j < 2; j++) {
                    if (j > 0)
                        addedElements += io::encoding::encode(stream, (char)',');
                    addedElements += io::encoding::encode(stream, (double)m_vertices[i][j], numericFormat);
                }
            }
            addedElements += io::encoding::encode(stream, "\">");

            // End XML element
            addedElements += io::encoding::encode(stream, "</Polygon2D>\n");

            // Pass back added elements in stream
            return addedElements;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };

    //! \}
} // namespace geometry
} // namespace broccoli

#endif // HAVE_EIGEN3
