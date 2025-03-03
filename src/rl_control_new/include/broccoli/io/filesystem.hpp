/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "compression.hpp"
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace broccoli {
namespace io {
    namespace filesystem {
        /*!
         * \addtogroup broccoli_io_filesystem
         * \{
         */

        // Type definitions
        typedef compression::DataStream DataStream; //!< Specifies the used type for data streams

        //! Computes the filename (with extension) from a given file path
        /*!
         * \param [in] filePath The file path to parse
         * \param [in] delimiter The delimiting character between directory and file name
         * \return File name (with extension)
         */
        static inline std::string parseFileName(const std::string& filePath, const char& delimiter = '/')
        {
            const size_t lastDelimiterIndex = filePath.rfind(delimiter); // Position of last delimiter in the file path
            if (lastDelimiterIndex != std::string::npos) {
                // Delimiter was found -> split into parts
                if (lastDelimiterIndex + 1 < filePath.size())
                    return filePath.substr(lastDelimiterIndex + 1); // Delimiter is NOT last character in file path -> use second part
                else
                    return ""; // Delimiter IS last character in file path -> empty file name
            } else
                return filePath; // No delimiter was found -> file path is file name
        }

        //! Extracts the file extension (without dot) from a given file path or file name
        /*!
         * \param [in] filePathOrName The file path or file name
         * \return File extension (without dot)
         */
        static inline std::string parseFileExtension(const std::string& filePathOrName)
        {
            const size_t lastDotIndex = filePathOrName.rfind('.'); // Position of last dot in the file path/name
            if (lastDotIndex != std::string::npos) {
                // Dot was found -> split into parts
                if (lastDotIndex + 1 < filePathOrName.size())
                    return filePathOrName.substr(lastDotIndex + 1); // Dot is NOT last character in file path/name -> use second part
                else
                    return ""; // Dot IS last character in file path/name -> empty file extension
            } else
                return ""; // No point was found -> no file extension
        }

        //! Extracts compression options from the given file extension
        /*!
         * \param [in] fileExtension The file extension (with or without dot, lower- or upper-case)
         * \return Expected compression options
         */
        static inline compression::CompressionOptions fileExtensionToCompressionOptions(const std::string& fileExtension)
        {
            if (fileExtension == "zlib" || fileExtension == ".zlib" || fileExtension == "ZLIB" || fileExtension == ".ZLIB")
                return compression::CompressionOptions::zlib();
            else if (fileExtension == "gz" || fileExtension == ".gz" || fileExtension == "GZ" || fileExtension == ".GZ")
                return compression::CompressionOptions::gzip();
            else
                return compression::CompressionOptions::noCompression();
        }

        //! Extracts compression options from the given file path of file name
        /*!
         * \param [in] filePathOrName The file path or file name
         * \return Expected compression options
         */
        static inline compression::CompressionOptions filePathOrNameToCompressionOptions(const std::string& filePathOrName)
        {
            return fileExtensionToCompressionOptions(parseFileExtension(filePathOrName));
        }

        //! Checks, if a specified file exists
        /*!
         * \remark In case the requested information cannot be obtained, `false` is returned (assume that it does not exist).
         *
         * \remark In case of an error the underlying lowlevel functions set the variable `errno` which can be used to further
         * classify the error.
         *
         * \param [in] filePath Path to file to test (absolute path or relative to executable path).
         * \return `true` if specified file exists, `false` if not (or path corresponds not to a file but a directory instead)
         */
        static inline bool fileExists(const std::string& filePath)
        {
            // Catch lowlevel exceptions
            try {
                // Try to obtain info about file system element
                struct ::stat elementInfo;
                if (::stat(filePath.c_str(), &elementInfo) != 0) {
                    // ...could not obtain info -> file does not exist
                    return false;
                }

                // We got the element info -> check if element is a regular file
                if ((elementInfo.st_mode & S_IFMT) == S_IFREG)
                    return true; // Element is a regular file
                else
                    return false; // Element is not a regular file (maybe directory, socket, etc. )
            } catch (...) {
                return false; // ...could not obtain info -> something went wrong
            }
        }

        //! Returns the total size of the given file in bytes, or -1 in case of an error
        /*! \attention On non-UNIX systems this might give wrong results. Should be replaced by `std::filesystem::file_size` (C++17) in future. */
        static inline int64_t fileSize(const std::string& filePath)
        {
            std::ifstream inputFile(filePath, std::ifstream::ate | std::ifstream::binary);
            return inputFile.tellg();
        }

        //! Deletes the specified file
        /*!
         * \remark In case of an error the underlying lowlevel functions set the variable `errno` which can be used to further
         * classify the error.
         *
         * \param [in] filePath Path to file to remove (absolute path or relative to executable path).
         * \return `true` if specified file was removed, `false` if an error occured
         */
        static inline bool removeFile(const std::string& filePath)
        {
            // Catch lowlevel exceptions
            try {
                // Try to remove file
                if (::remove(filePath.c_str()) != 0)
                    return false; // ...something went wrong
                else
                    return true; // ...success!
            } catch (...) {
                return false; // ...something went wrong
            }
        }

        //! Copies the given source file to the specified destination
        /*!
         * \param [in] sourceFile The path to the source file (input)
         * \param [in] destinationFile The path to the destination file (output)
         * \return `true` on success, `false` otherwise
         */
        static inline bool copyFile(const std::string& sourceFile, const std::string& destinationFile)
        {
            // Abort, if source file does not exist
            if (fileExists(sourceFile) == false)
                return false;

            // Open input stream
            std::ifstream inputStream(sourceFile, std::ifstream::binary);
            if (inputStream.is_open() == false)
                return false;

            // Open output stream
            std::ofstream outputStream(destinationFile, std::ofstream::binary);
            if (outputStream.is_open() == false) {
                inputStream.close();
                return false;
            }

            // Copy data
            outputStream << inputStream.rdbuf();

            // Close files
            inputStream.close();
            outputStream.close();

            // Success
            return true;
        }

        //! Computes the directory from a given file path
        /*!
         * \param [in] filePath The file path to parse
         * \param [in] delimiter The delimiting character between directory and file name
         * \return Path to directory containing the file
         */
        static inline std::string parseDirectory(const std::string& filePath, const char& delimiter = '/')
        {
            const size_t lastDelimiterIndex = filePath.rfind(delimiter); // Position of last delimiter in the file path
            if (lastDelimiterIndex != std::string::npos) {
                // Delimiter was found -> split into parts
                if (lastDelimiterIndex > 0)
                    return filePath.substr(0, lastDelimiterIndex); // Delimiter is NOT first character in file path -> use first part
                else
                    return ""; // Delimiter IS first character in file path -> empty directory
            } else
                return ""; // No delimiter was found -> no directory
        }

        //! Checks, if a specified directory exists
        /*!
         * \remark In case the requested information cannot be obtained, `false` is returned (assume that it does not exist).
         *
         * \remark In case of an error the underlying lowlevel functions set the variable `errno` which can be used to further
         * classify the error.
         *
         * \param [in] directoryPath Path to directory to test (absolute path or relative to executable path).
         * \return `true` if specified directory exists, `false` if not (or path corresponds not to a directory but a file instead)
         */
        static inline bool directoryExists(const std::string& directoryPath)
        {
            // Catch lowlevel exceptions
            try {
                // Try to obtain info about file system element
                struct ::stat elementInfo;
                if (::stat(directoryPath.c_str(), &elementInfo) != 0) {
                    // ...could not obtain info -> directory does not exist
                    return false;
                }

                // We got the element info -> check if element is a directory
                if ((elementInfo.st_mode & S_IFMT) == S_IFDIR)
                    return true; // Element is a directory
                else
                    return false; // Element is not a directory (maybe file, socket, etc. )
            } catch (...) {
                return false; // ...could not obtain info -> something went wrong
            }
        }

        //! Tries to create a directory in the specified path
        /*!
         * \attention The parent directory has to exist for this method to work!
         *
         * \remark In case of an error the underlying lowlevel functions set the variable `errno` which can be used to further
         * classify the error.
         *
         * \param [in] directoryPath Path to directory to create (absolute path or relative to executable path).
         * \param [in] accessMode Permission of directory
         * \return `true` if specified directory was created, `false` if not
         */
        static inline bool createDirectory(const std::string& directoryPath, const mode_t& accessMode = S_IRWXU | S_IRWXG | S_IRWXO)
        {
            // Catch lowlevel exceptions
            try {
                if (::mkdir(directoryPath.c_str(), accessMode) != 0)
                    return false; // Could not create directory (error during creation or directory already exists)
                else
                    return true; // Directory successfully created
            } catch (...) {
                return false; // ...something went wrong
            }
        }

        //! Deletes the specified **empty** directory
        /*!
         * \warning The specified directory has to be **empty** for this method to work!
         *
         * \remark In case of an error the underlying lowlevel functions set the variable `errno` which can be used to further
         * classify the error.
         *
         * \param [in] directoryPath Path to directory to remove (absolute path or relative to executable path).
         * \return `true` if specified directory was removed, `false` if an error occured
         */
        static inline bool removeDirectory(const std::string& directoryPath)
        {
            // Catch lowlevel exceptions
            try {
                // Try to remove directory
                if (::rmdir(directoryPath.c_str()) != 0)
                    return false; // ...something went wrong
                else
                    return true; // ...success!
            } catch (...) {
                return false; // ...something went wrong
            }
        }

        //! Writes the given data buffer to the specified file
        /*!
         * \param [in] filePath Path to the output file
         * \param [in] data Data buffer to be written to the file (uncompressed)
         * \param [in] compression Specifies compression options
         * \return `true` on success, `false` otherwise
         */
        static inline bool writeFile(const std::string& filePath, const DataStream& data, const compression::CompressionOptions& compression)
        {
            try {
                std::ofstream outputStream(filePath, std::ofstream::binary);
                if (outputStream.is_open() == false)
                    return false;
                if (compression.compressed() == false)
                    outputStream.write(reinterpret_cast<const char*>(data.data()), data.size());
                else {
                    DataStream compressedData;
                    if (compression::compress(data, compressedData, compression) == false)
                        return false;
                    outputStream.write(reinterpret_cast<const char*>(compressedData.data()), compressedData.size());
                }
                outputStream.close();
            } catch (...) {
                // Something went wrong...
                return false;
            }

            // No errors -> success
            return true;
        }

        //! Writes the given data buffer to the specified file (automatically detects compression options)
        /*!
         * \param [in] filePath Path to the output file
         * \param [in] data Data buffer to be written to the file (uncompressed)
         * \return `true` on success, `false` otherwise
         */
        static inline bool writeFile(const std::string& filePath, const DataStream& data) { return writeFile(filePath, data, filePathOrNameToCompressionOptions(filePath)); }

        //! Reads the given data buffer from the specified file
        /*!
         * \attention For best performance, the data buffer should be pre-allocated with the estimated maximum (uncompressed) file size (use resize() and not reserve())!
         *
         * \param [in] filePath Path to the input file
         * \param [out] data Data buffer to be read from the file (uncompressed)
         * \param [in] compression Specifies compression options
         * \return `true` on success, `false` otherwise
         */
        static inline bool readFile(const std::string& filePath, DataStream& data, const compression::DecompressionOptions& compression)
        {
            // Get file size first
            const int64_t inputFileSize = fileSize(filePath); // Raw byte count of input file
            if (inputFileSize < 0)
                return false; // Could not determine file size

            // Read file
            try {
                std::ifstream inputStream(filePath, std::ifstream::binary);
                if (inputStream.is_open() == false)
                    return false;
                if (compression.compressed() == false) {
                    data.resize(inputFileSize);
                    inputStream.read(reinterpret_cast<char*>(data.data()), data.size());
                } else {
                    DataStream compressedData;
                    compressedData.resize(inputFileSize);
                    inputStream.read(reinterpret_cast<char*>(compressedData.data()), compressedData.size());
                    if (compression::decompress(compressedData, data, compression) == false)
                        return false;
                }
                inputStream.close();
            } catch (...) {
                // Something went wrong...
                return false;
            }

            // No errors -> success
            return true;
        }

        //! Reads the given data buffer from the specified file (automatically detects compression options)
        /*!
         * \attention For best performance, the data buffer should be pre-allocated with the estimated maximum (uncompressed) file size (use resize() and not reserve())!
         *
         * \param [in] filePath Path to the input file
         * \param [out] data Data buffer to be read from the file (uncompressed)
         * \return `true` on success, `false` otherwise
         */
        static inline bool readFile(const std::string& filePath, DataStream& data) { return readFile(filePath, data, filePathOrNameToCompressionOptions(filePath)); }

        //! \}
    } // namespace filesystem
} // namespace io
} // namespace broccoli
