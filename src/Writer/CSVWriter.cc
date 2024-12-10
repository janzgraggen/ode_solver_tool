/**
 * @file CSVWriterMethods.cc
 * @brief Implementation of the `CSVWriter` class methods for managing CSV output.
 *
 * This file contains the implementation of methods to handle the creation,
 * writing, and management of CSV output files. It includes functionality to
 * create the output directory if it doesn't already exist, open CSV files,
 * and write headers and rows containing system time and state information.
 *
 * @author janzgraggen
 * @date 08/12/2024
 */

#include "CSVWriter.hh"
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

/**
 * @brief Constructor for the CSVWriter class.
 *
 * Initializes the CSVWriter object and attempts to open the specified output file.
 * If the filename is empty or set to "null", no file will be created.
 *
 * @param filename The name of the CSV file to be opened for writing.
 *                 If an empty or "null" filename is provided, no output file is created.
 */
CSVWriter::CSVWriter(const std::string& filename)
    : hasOutput(false) {  // Initialize hasOutput to false
    if (!filename.empty() && filename != "null") {
        openFile(filename);
    }
}

/**
 * @brief Destructor for the CSVWriter class.
 *
 * Closes the output file if it is currently open to ensure proper resource management.
 */
CSVWriter::~CSVWriter() {
    if (file.is_open()) {
        file.close();
    }
}

/**
 * @brief Opens a CSV file for writing output.
 *
 * Handles the creation of the output directory if it does not already exist.
 * Constructs the full file path and attempts to open it in write mode,
 * truncating any existing content.
 *
 * @param filename The name of the CSV file to be created.
 *                 If the filename is empty or "null", no output file will be created.
 */
void CSVWriter::openFile(const std::string& filename) {
    if (filename.empty() || filename == "null") {
        // An empty filename is valid; no file will be created
        hasOutput = false;
        return;
    }

    // Ensure the output directory exists, create if it doesn't
    std::string directory = "../output";  // Change this to the desired directory
    if (!fs::exists(directory)) {
        fs::create_directories(directory);
    }

    // Build the full file path (e.g., "output/results.csv")
    std::string filepath = directory + "/" + filename;

    // Open the file in output mode and truncate (overwrite) if it already exists
    file.open(filepath, std::ios::out | std::ios::trunc);
    if (file) {
        hasOutput = true;
    } else {
        hasOutput = false;  // If the file couldn't be opened, mark it as invalid
    }
}

/**
 * @brief Writes the header row to the CSV file.
 *
 * Writes the column names to the CSV file, starting with "Time" followed by
 * the components of the state vector `y` (e.g., Time, y0, y1, ..., yn).
 *
 * @param y The state vector (Eigen::VectorXd) representing system states.
 * This method creates a header row describing the contents of subsequent rows.
 */
void CSVWriter::writeHeader(const Eigen::VectorXd& y) {
    if (!hasOutput) return;  // Do nothing if no file is valid

    // Write the header: Time, y0, y1, ..., yn
    file << "Time";
    for (int i = 0; i < y.size(); ++i) {
        file << ",y" << i;
    }
    file << "\n";
}

/**
 * @brief Writes a row of data to the CSV file.
 *
 * Logs the time step along with the components of the state vector `y`.
 * Each row contains the time `t` and the corresponding system state values.
 *
 * @param t The current time step.
 * @param y The state vector (Eigen::VectorXd) containing system state variables.
 * This method writes the time value followed by the state values, separated by commas.
 */
void CSVWriter::writeLine(double t, const Eigen::VectorXd& y) {
    if (!hasOutput) return;  // Do nothing if no file is valid

    // Write the time and the corresponding solution vector to the file
    file << t;
    for (int i = 0; i < y.size(); ++i) {
        file << "," << y(i);
    }
    file << "\n";
}
