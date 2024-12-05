#ifndef CSVWRITER_H
#define CSVWRITER_H

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <string>
#include <filesystem>  // For handling directories

namespace fs = std::filesystem;  // To easily handle file paths and directories

class CSVWriter {
public:
    /**
     * @brief Constructor that opens a CSV file for writing.
     * Creates the output directory if it doesn't exist.
     *
     * @param filename The name of the CSV file to write to.
     */
    CSVWriter(const std::string& filename);

    /**
     * @brief Destructor to ensure the file is properly closed.
     */
    ~CSVWriter();

    /**
     * @brief Opens the file and creates the directory if needed.
     *
     * @param filename The file to write.
     */
    void openFile(const std::string& filename);

    void writeHeader(const Eigen::VectorXd& y);
    void writeLine(double t, const Eigen::VectorXd& y);

private:
    std::ofstream file; /**< The file stream used for writing data. */
};

#endif // CSVWRITER_H
