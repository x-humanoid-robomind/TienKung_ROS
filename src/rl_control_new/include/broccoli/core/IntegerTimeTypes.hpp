/*
 * This file is part of broccoli
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once

#include "FixedStepArithmeticNumber.hpp"

namespace broccoli {
namespace core {
    /*!
     * \addtogroup broccoli_core_integertime
     * \{
     */

    /*!
     * \brief Represents seconds as fixed-step integer arithmetic number.
     * \note The double representation of this number has the unit [seconds]
     */
    template <typename T>
    class IntegerSeconds : public FixedStepArithmeticNumber<T> {
    public:
        //! Constructs an IntegerSeconds instance from a double
        /*!
         * \param value The number of seconds to represent. Only whole seconds can be stored.
         */
        IntegerSeconds(const double& value)
            : FixedStepArithmeticNumber<T>(value, definedStepSize())
        {
        }

        //! Constructs an IntegerSeconds instance initialized with zero.
        IntegerSeconds()
            : FixedStepArithmeticNumber<T>((T)0, definedStepSize())
        {
        }

        //! Constructs an IntegerSeconds instance from an integer
        /*!
         * \param value The number of seconds to represent.
         */
        IntegerSeconds(const T& value)
            : FixedStepArithmeticNumber<T>(value, definedStepSize())
        {
        }

    protected:
        //! Defines the predefined step size
        double definedStepSize() const
        {
            return 1.0;
        }
    };

    /*!
     * \brief Represents milliseconds as fixed-step integer arithmetic number.
     * \note The double representation of this number has the unit [seconds]
     */
    template <typename T>
    class IntegerMilliSeconds : public FixedStepArithmeticNumber<T> {
    public:
        //! Constructs an IntegerMilliSeconds instance from a double
        /*!
         * \param value The number of seconds to represent. Only whole milliseconds can be stored.
         */
        IntegerMilliSeconds(const double& value)
            : FixedStepArithmeticNumber<T>(value, definedStepSize())
        {
        }

        //! Constructs an IntegerMilliSeconds instance initialized with zero.
        IntegerMilliSeconds()
            : FixedStepArithmeticNumber<T>((T)0, definedStepSize())
        {
        }

        //! Constructs an IntegerMilliSeconds instance from an integer
        /*!
         * \param value The number of milliseconds to represent.
         */
        IntegerMilliSeconds(const T& value)
            : FixedStepArithmeticNumber<T>(value, definedStepSize())
        {
        }

    protected:
        //! Defines the predefined step size
        double definedStepSize() const
        {
            return 1.0e-03;
        }
    };

    /*!
     * \brief Represents microseconds as fixed-step integer arithmetic number.
     * \note The double representation of this number has the unit [seconds]
     */
    template <typename T>
    class IntegerMicroSeconds : public FixedStepArithmeticNumber<T> {
    public:
        //! Constructs an IntegerMicroSeconds instance from a double
        /*!
         * \param value The number of seconds to represent. Only whole microseconds can be stored.
         */
        IntegerMicroSeconds(const double& value)
            : FixedStepArithmeticNumber<T>(value, definedStepSize())
        {
        }

        //! Constructs an IntegerMicroSeconds instance initialized with zero.
        IntegerMicroSeconds()
            : FixedStepArithmeticNumber<T>((T)0, definedStepSize())
        {
        }

        //! Constructs an IntegerMicroSeconds instance from an integer
        /*!
         * \param value The number of microseconds to represent.
         */
        IntegerMicroSeconds(const T& value)
            : FixedStepArithmeticNumber<T>(value, definedStepSize())
        {
        }

    protected:
        //! Defines the predefined step size
        double definedStepSize() const
        {
            return 1.0e-06;
        }
    };

    /*!
     * \brief Represents nanoseconds as fixed-step integer arithmetic number.
     * \note The double representation of this number has the unit [seconds]
     */
    template <typename T>
    class IntegerNanoSeconds : public FixedStepArithmeticNumber<T> {
    public:
        //! Constructs an IntegerNanoSeconds instance from a double
        /*!
         * \param value The number of seconds to represent. Only whole nanoseconds can be stored.
         */
        IntegerNanoSeconds(const double& value)
            : FixedStepArithmeticNumber<T>(value, definedStepSize())
        {
        }

        //! Constructs an IntegerNanoSeconds instance initialized with zero.
        IntegerNanoSeconds()
            : FixedStepArithmeticNumber<T>((T)0, definedStepSize())
        {
        }

        //! Constructs an IntegerNanoSeconds instance from an integer
        /*!
         * \param value The number of nanoseconds to represent.
         */
        IntegerNanoSeconds(const T& value)
            : FixedStepArithmeticNumber<T>(value, definedStepSize())
        {
        }

    protected:
        //! Defines the predefined step size
        double definedStepSize() const
        {
            return 1.0e-09;
        }
    };

    //! \}
} // namespace core
} // namespace broccoli
