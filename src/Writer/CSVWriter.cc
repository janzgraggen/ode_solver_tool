#include "CSVWriter.hh"
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

CSVWriter::CSVWriter(const std::string& filename) {
    // Open the file and create the directory if it does not exist
    openFile(filename);
}

CSVWriter::~CSVWriter() {
    // Close the file when the object is destroyed
    if (file.is_open()) {
        file.close();
    }
}

void CSVWriter::openFile(const std::string& filename) {
    // Ensure the output directory exists, create if it doesn't
    std::string directory = "../output";  // Change this to the desired directory
    if (!fs::exists(directory)) {
        fs::create_directories(directory);
    }

    // Build the full file path (e.g., "output/results.csv")
    std::string filepath = directory + "/" + filename;

    // Open the file in output mode and truncate (overwrite) if it exists
    file.open(filepath, std::ios::out | std::ios::trunc);
    if (!file) {
        std::cerr << "Error: Unable to open file for writing: " << filepath << "\n";
    }
}

void CSVWriter::writeHeader(const Eigen::VectorXd& y) {
    // Write the header: Time, y0, y1, ..., yn
    file << "Time";
    for (int i = 0; i < y.size(); ++i) {
        file << ",y" << i;
    }
    file << "\n";
}

void CSVWriter::writeLine(double t, const Eigen::VectorXd& y) {
    // Write the time and the corresponding solution vector to the file
    file << t;
    for (int i = 0; i < y.size(); ++i) {
        file << "," << y(i);
    }
    file << "\n";
}
