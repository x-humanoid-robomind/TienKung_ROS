/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include <utility>

namespace broccoli {
namespace core {
    /*!
     * \addtogroup broccoli_core_utils
     * \{
     */

    /*!
     * \brief Helper to implicitly construct object of type T
     *
     * This is convenient when constructing objects with deleted copy-constructor.
     * Usage Example:
     * \code
     * template<typename T, typename... Args>
     * implicit_construct<T> createObject(Args&&... args) {
     *     return {std::forward<Args>(args)...}; // does not require copy constructor
     * }
     * \endcode
     * \tparam T Type of object to construct
     */
    template <typename T>
    struct implicit_construct : T {
        /*!
         * \brief Constructor
         * \tparam Args Types of arguments
         * \param args Optional arguments passed to the constructor
         */
        template <typename... Args>
        implicit_construct(Args&&... args)
            : T(std::forward<Args>(args)...)
        {
        }
    };

    /*!
     * \brief Helper to implicitly construct object of type T and call a callable with the newly created object
     *
     * This is convenient when constructing objects with deleted copy-constructor
     * while still being able to execute a callable prior to returning it.
     * Usage Example:
     * \code
     * template<typename T, typename... Args>
     * implicit_construct_and_call<T> createObject(Args&&... args) {
     *     // does not require copy constructor and calls f(object, 1) after construction
     *     return {std::bind(f, std::placeholders::_1, 1), std::forward<Args>(args)...};
     * }
     * \endcode
     * \tparam T The type to construct
     */
    template <typename T>
    struct implicit_construct_and_call : T {
        /*!
         * \brief Constructor
         * \tparam Callable Type of callable object
         * \tparam Args Type of optional arguments
         * \param toCallWithObject Callable object, which is called with the created object after construction
         * \param args Optional arguments to the constructor of T
         */
        template <typename Callable, typename... Args>
        implicit_construct_and_call(Callable&& toCallWithObject, Args&&... args)
            : T(std::forward<Args>(args)...)
        {
            toCallWithObject(*this);
        }
    };

    /*!
     * \brief Copies count bytes from src to dest in reverse order, starting from src[count-1]
     *
     * The behavior is undefined if either pointer is a nullptr or if memory is access out of bounds.
     * \param dest The destination buffer
     * \param src The source buffer
     * \param count The number of bytes
     * \return The destination buffer dest
     */
    inline void* reverse_memcpy(void* dest, const void* src, std::size_t count)
    {
        auto* d = static_cast<unsigned char*>(dest);
        const auto* s = static_cast<const unsigned char*>(src);

        for (std::size_t i = 0; i < count; i++) {
            d[count - 1 - i] = s[i];
        }
        return dest;
    }

    //! \}
} // namespace core
} // namespace broccoli
