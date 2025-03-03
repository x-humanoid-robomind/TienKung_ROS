/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include <Eigen/Dense>
#include <algorithm>
#include <math.h>

namespace broccoli {
namespace geometry {
    /*!
     * \addtogroup broccoli_geometry
     * \{
     */

    //! Representation of Denavit-Hartenberg parameters
    /*!
     * Convention according to Craig, J. J. (1989). "Introduction to Robotics, Mechanics & Control". Addison-Wesley Publishing Company, Inc.
     *
     *   * \copybrief a()
     *   * \copybrief alpha()
     *   * \copybrief d()
     *   * \copybrief theta()
     */
    class DHParameters {
    public:
        //! Default constructor
        DHParameters()
        {
        }

        //! Specialized constructor
        /*!
         * \param a \f$a_{i-1}\f$
         * \param alpha \f$\alpha_{i-1}\f$
         * \param d \f$d_i\f$
         * \param theta \f$\theta_i\f$
         */
        DHParameters(const double& a, const double& alpha, const double& d, const double& theta)
            : m_parameters(a, alpha, d, theta)
        {
        }

        //! Specialized constructor
        /*! Triggers \ref fromTransform(const Eigen::Matrix3d&, const Eigen::Vector3d&, const bool&) */
        DHParameters(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) { fromTransform(rotation, translation); }

        //! Specialized constructor
        /*! Triggers \ref fromTransform(const Eigen::Isometry3d&, const bool&) */
        DHParameters(const Eigen::Isometry3d& transform) { fromTransform(transform); }

        // Members
        // -------
    protected:
        Eigen::Vector4d m_parameters = Eigen::Vector4d::Zero(); //!< \copybrief parameters()

        // Conversions
        // -----------
    public:
        //! Computes the homogeneous transformation \f${}_{i-1} D_i\f$ from the \f$i\f$-frame to the \f$i-1\f$-frame
        /*!
         * \f[
         * {}_{i-1} D_i = \left[\begin{array}{cccc}
         * \cos(\theta_i) & -\sin(\theta_i) & 0 & a_{i-1}\\
         * \sin(\theta_i) \cos(\alpha_{i-1}) & \cos(\theta_i) \cos(\alpha_{i-1}) & -\sin(\alpha_{i-1}) & -\sin(\alpha_{i-1}) d_i\\
         * \sin(\theta_i) \sin(\alpha_{i-1}) & \cos(\theta_i) \sin(\alpha_{i-1}) & \cos(\alpha_{i-1}) & \cos(\alpha_{i-1}) d_i\\
         * 0 & 0 & 0 & 1
         * \end{array}\right]
         * \f]
         */
        Eigen::Isometry3d toTransform() const
        {
            // Compute helpers
            const double cosAlpha = cos(m_parameters(1));
            const double sinAlpha = sin(m_parameters(1));
            const double cosTheta = cos(m_parameters(3));
            const double sinTheta = sin(m_parameters(3));

            // Compute transform
            Eigen::Isometry3d returnValue;
            returnValue(0, 0) = cosTheta;
            returnValue(0, 1) = -sinTheta;
            returnValue(0, 2) = 0.0;
            returnValue(0, 3) = m_parameters(0);
            returnValue(1, 0) = sinTheta * cosAlpha;
            returnValue(1, 1) = cosTheta * cosAlpha;
            returnValue(1, 2) = -sinAlpha;
            returnValue(1, 3) = -sinAlpha * m_parameters(2);
            returnValue(2, 0) = sinTheta * sinAlpha;
            returnValue(2, 1) = cosTheta * sinAlpha;
            returnValue(2, 2) = cosAlpha;
            returnValue(2, 3) = cosAlpha * m_parameters(2);
            returnValue(3, 0) = 0.0;
            returnValue(3, 1) = 0.0;
            returnValue(3, 2) = 0.0;
            returnValue(3, 3) = 1.0;
            return returnValue;
        }

        //! Computes the DH parameters from the given homogeneous transformation \f${}_{i-1} D_i\f$
        /*!
         * \copydetails toTransform()
         *
         * \param [in] rotation Rotation part of \f${}_{i-1} D_i\f$ in the specified form
         * \param [in] translation Translation part of \f${}_{i-1} D_i\f$ in the specified form
         * \param [in] computeConversionError If `true`, the conversion error (=maximum absolute difference (component-wise) between the original transform and the transform re-computed from the DH-parameters) is computed
         * \return If \p computeConversionError is `true`, the conversion error is returned. A high error indicates, that the given transform does not comply with the desired matrix structure specified by the DH-convention. If \p computeConversionError is `false`, 0 is returned.
         */
        double fromTransform(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation, const bool& computeConversionError = false)
        {
            // Get intermediate values
            const double& cosAlpha = rotation(2, 2);
            const double sinAlpha = -rotation(1, 2);
            const double& cosTheta = rotation(0, 0);
            const double sinTheta = -rotation(0, 1);

            // Compute a_{i-1}
            m_parameters(0) = translation(0);

            // Compute \alpha_{i-1}
            m_parameters(1) = atan2(sinAlpha, cosAlpha);

            // Compute d_i
            if (fabs(cosAlpha) > 1e-9)
                m_parameters(2) = translation(2) / cosAlpha;
            else if (fabs(sinAlpha) > 1e-9)
                m_parameters(2) = -translation(1) / sinAlpha;
            else
                m_parameters(2) = 0.0;

            // Compute \theta_i
            m_parameters(3) = atan2(sinTheta, cosTheta);

            // Check, if we should compute the conversion error
            if (computeConversionError == true) {
                const Eigen::Isometry3d newTransform = toTransform();
                const Eigen::Matrix3d rotationDifference = newTransform.linear() - rotation;
                const Eigen::Vector3d translationDifference = newTransform.translation() - translation;
                const double rotationError = std::max(fabs(rotationDifference.minCoeff()), fabs(rotationDifference.maxCoeff()));
                const double translationError = std::max(fabs(translationDifference.minCoeff()), fabs(translationDifference.maxCoeff()));
                return std::max(rotationError, translationError);
            } else
                return 0.0;
        }

        //! Computes the DH parameters from the given homogeneous transformation \f${}_{i-1} D_i\f$
        /*!
         * \copydetails toTransform()
         *
         * \param [in] transform Homogeneous transform \f${}_{i-1} D_i\f$ in the specified form
         * \param [in] computeConversionError If `true`, the conversion error (=maximum absolute difference (component-wise) between the original transform and the transform re-computed from the DH-parameters) is computed
         * \return If \p computeConversionError is `true`, the conversion error is returned. A high error indicates, that the given transform does not comply with the desired matrix structure specified by the DH-convention. If \p computeConversionError is `false`, 0 is returned.
         */
        double fromTransform(const Eigen::Isometry3d& transform, const bool& computeConversionError = false) { return fromTransform(transform.linear(), transform.translation(), computeConversionError); }

        //! Computes the homogeneous transformation \f${}_i D_{i-1}\f$ from the \f$i-1\f$-frame to the \f$i\f$-frame
        /*!
         * \f[
         * {}_i D_{i-1} = \left[\begin{array}{cccc}
         * \cos(\theta_i) & \sin(\theta_i) \cos(\alpha_{i-1}) & \sin(\theta_i) \sin(\alpha_{i-1}) & -\cos(\theta_i) a_{i-1}\\
         *  -\sin(\theta_i) & \cos(\theta_i) \cos(\alpha_{i-1}) & \cos(\theta_i) \sin(\alpha_{i-1}) & \sin(\theta_i) a_{i-1}\\
         * 0 & -\sin(\alpha_{i-1}) & \cos(\alpha_{i-1}) & -d_i\\
         * 0 & 0 & 0 & 1
         * \end{array}\right]
         * \f]
         */
        Eigen::Isometry3d toTransformInverse() const
        {
            // Compute helpers
            const double cosAlpha = cos(m_parameters(1));
            const double sinAlpha = sin(m_parameters(1));
            const double cosTheta = cos(m_parameters(3));
            const double sinTheta = sin(m_parameters(3));

            // Compute transform
            Eigen::Isometry3d returnValue;
            returnValue(0, 0) = cosTheta;
            returnValue(0, 1) = sinTheta * cosAlpha;
            returnValue(0, 2) = sinTheta * sinAlpha;
            returnValue(0, 3) = -cosTheta * m_parameters(0);
            returnValue(1, 0) = -sinTheta;
            returnValue(1, 1) = cosTheta * cosAlpha;
            returnValue(1, 2) = cosTheta * sinAlpha;
            returnValue(1, 3) = sinTheta * m_parameters(0);
            returnValue(2, 0) = 0.0;
            returnValue(2, 1) = -sinAlpha;
            returnValue(2, 2) = cosAlpha;
            returnValue(2, 3) = -m_parameters(2);
            returnValue(3, 0) = 0.0;
            returnValue(3, 1) = 0.0;
            returnValue(3, 2) = 0.0;
            returnValue(3, 3) = 1.0;
            return returnValue;
        }

        //! Computes the DH parameters from the given homogeneous transformation \f${}_i D_{i-1}\f$
        /*!
         * \copydetails toTransformInverse()
         *
         * \param [in] rotation Rotation part of \f${}_i D_{i-1}\f$ in the specified form
         * \param [in] translation Translation part of \f${}_i D_{i-1}\f$ in the specified form
         * \param [in] computeConversionError If `true`, the conversion error (=maximum absolute difference (component-wise) between the original transform and the transform re-computed from the DH-parameters) is computed
         * \return If \p computeConversionError is `true`, the conversion error is returned. A high error indicates, that the given transform does not comply with the desired matrix structure specified by the DH-convention. If \p computeConversionError is `false`, 0 is returned.
         */
        double fromTransformInverse(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation, const bool& computeConversionError = false)
        {
            // Get intermediate values
            const double& cosAlpha = rotation(2, 2);
            const double sinAlpha = -rotation(2, 1);
            const double& cosTheta = rotation(0, 0);
            const double sinTheta = -rotation(1, 0);

            // Compute a_{i-1}
            if (fabs(cosTheta) > 1e-9)
                m_parameters(0) = -translation(0) / cosTheta;
            else if (fabs(sinTheta) > 1e-9)
                m_parameters(0) = translation(1) / sinTheta;
            else
                m_parameters(2) = 0.0;

            // Compute \alpha_{i-1}
            m_parameters(1) = atan2(sinAlpha, cosAlpha);

            // Compute d_i
            m_parameters(2) = -translation(2);

            // Compute \theta_i
            m_parameters(3) = atan2(sinTheta, cosTheta);

            // Check, if we should compute the conversion error
            if (computeConversionError == true) {
                const Eigen::Isometry3d newTransform = toTransformInverse();
                const Eigen::Matrix3d rotationDifference = newTransform.linear() - rotation;
                const Eigen::Vector3d translationDifference = newTransform.translation() - translation;
                const double rotationError = std::max(fabs(rotationDifference.minCoeff()), fabs(rotationDifference.maxCoeff()));
                const double translationError = std::max(fabs(translationDifference.minCoeff()), fabs(translationDifference.maxCoeff()));
                return std::max(rotationError, translationError);
            } else
                return 0.0;
        }

        //! Computes the DH parameters from the given homogeneous transformation \f${}_i D_{i-1}\f$
        /*!
         * \copydetails toTransformInverse()
         *
         * \param [in] transform Homogeneous transform \f${}_i D_{i-1}\f$ in the specified form
         * \param [in] computeConversionError If `true`, the conversion error (=maximum absolute difference (component-wise) between the original transform and the transform re-computed from the DH-parameters) is computed
         * \return If \p computeConversionError is `true`, the conversion error is returned. A high error indicates, that the given transform does not comply with the desired matrix structure specified by the DH-convention. If \p computeConversionError is `false`, 0 is returned.
         */
        double fromTransformInverse(const Eigen::Isometry3d& transform, const bool& computeConversionError = false) { return fromTransformInverse(transform.linear(), transform.translation(), computeConversionError); }

        // Setters and Getters
        // -------------------
    public:
        //! The parameters \f$ [a_{i-1},\,\alpha_{i-1},\,d_i,\,\theta_i] \f$
        const Eigen::Vector4d& parameters() const { return m_parameters; }
        //! \copydoc parameters() const
        Eigen::Vector4d& parameters() { return m_parameters; }

        //! Parameter \f$ a_{i-1} \f$ (distance between the \f$z_{i-1}\f$ and \f$z_i\f$ axis along the \f$x_{i−1}\f$ axis)
        const double& a() const { return m_parameters(0); }
        //! \copydoc a() const
        double& a() { return m_parameters(0); }

        //! Parameter \f$ \alpha_{i-1} \f$ (angle between the \f$z_{i-1}\f$ and \f$z_i\f$ axis around the \f$x_{i−1}\f$ axis)
        const double& alpha() const { return m_parameters(1); }
        //! \copydoc alpha() const
        double& alpha() { return m_parameters(1); }

        //! Parameter \f$ d_i \f$ (distance between the \f$x_{i-1}\f$ and \f$x_i\f$ axis along the \f$z_i\f$ axis)
        const double& d() const { return m_parameters(2); }
        //! \copydoc d() const
        double& d() { return m_parameters(2); }

        //! Parameter \f$ \theta_i \f$ (angle between the \f$x_{i-1}\f$ and \f$x_i\f$ axis around the \f$z_i\f$ axis)
        const double& theta() const { return m_parameters(3); }
        //! \copydoc theta() const
        double& theta() { return m_parameters(3); }

        //! Sets all parameters
        /*!
         * \param [in] a \f$a_{i-1}\f$
         * \param [in] alpha \f$\alpha_{i-1}\f$
         * \param [in] d \f$d_i\f$
         * \param [in] theta \f$\theta_i\f$
         */
        void setParameters(const double& a, const double& alpha, const double& d, const double& theta)
        {
            m_parameters(0) = a;
            m_parameters(1) = alpha;
            m_parameters(2) = d;
            m_parameters(3) = theta;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    //! \}
} // namespace geometry
} // namespace broccoli

#endif // HAVE_EIGEN3
