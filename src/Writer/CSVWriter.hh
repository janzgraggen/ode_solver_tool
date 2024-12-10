/**
 * @file CSVWriter.hh
 * @brief Header file for the `CSVWriter` class.
 *
 * This class provides functionality to create and manage CSV output files for storing
 * time-series or system state information. It ensures that output directories are created
 * automatically and handles the writing of header rows and data rows to CSV files.
 *
 * @author janzgraggen
 * @date 08/12/2024
 */

#ifndef CSVWRITER_H
#define CSVWRITER_H

#include <fstream>
#include <Eigen/Dense>
#include <string>
#include <filesystem>  // For handling directories

namespace fs = std::filesystem;  // To easily handle file paths and directories

/**
 * @class CSVWriter
 * @brief A class to manage writing system state and time data to a CSV file.
 *
 * This class handles the creation of output directories, opening files for writing,
 * and writing headers and rows of data containing system states and timestamps.
 */
class CSVWriter {
public:
    /**
     * @brief Constructor that opens a CSV file for writing.
     *
     * Automatically creates the output directory if it does not already exist.
     *
     * @param filename The name of the CSV file to be created for output.
     */
    CSVWriter(const std::string& filename);

    /**
     * @brief Destructor for the `CSVWriter` class.
     *
     * Closes the output file if it is open to ensure proper resource management.
     */
    ~CSVWriter();

    /**
     * @brief Opens the output file and ensures the required directory structure.
     *
     * Constructs the file path and opens the file for writing while creating
     * the necessary directories if they do not already exist.
     *
     * @param filename The name of the CSV file to open.
     */
    void openFile(const std::string& filename);

    /**
     * @brief Writes the header row to the CSV file.
     *
     * Generates a header row with column names corresponding to time and state vector components.
     *
     * @param y The state vector (Eigen::VectorXd) whose components will be labeled in the header.
     */
    void writeHeader(const Eigen::VectorXd& y);

    /**
     * @brief Writes a row of time and system state data to the CSV file.
     *
     * Logs the current timestamp and the system's state vector components to the file.
     *
     * @param t The current time value.
     * @param y The state vector (Eigen::VectorXd) containing system state information.
     */
    void writeLine(double t, const Eigen::VectorXd& y);

private:
    std::ofstream file; /**< The file stream used to write data to the CSV file. */
    bool hasOutput;     /**< A flag indicating whether the file output is successfully initialized. */
};

#endif // CSVWRITER_H
