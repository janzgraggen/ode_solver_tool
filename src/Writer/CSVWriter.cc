#include "CSVWriter.hh"
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

CSVWriter::CSVWriter(const std::string& filename)
    : hasOutput(false) {  // Initialize hasOutput to false
    if (!filename.empty() || filename == "null") {
        openFile(filename);
    }
}

CSVWriter::~CSVWriter() {
    if (file.is_open()) {
        file.close();
    }
}

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

    // Open the file in output mode and truncate (overwrite) if it exists
    file.open(filepath, std::ios::out | std::ios::trunc);
    if (file) {
        hasOutput = true;
    } else {
        hasOutput = false;  // If the file couldn't be opened, mark it as invalid
    }
}

void CSVWriter::writeHeader(const Eigen::VectorXd& y) {
    if (!hasOutput) return;  // Do nothing if no file is valid
    // Write the header: Time, y0, y1, ..., yn
    file << "Time";
    for (int i = 0; i < y.size(); ++i) {
        file << ",y" << i;
    }
    file << "\n";
}

void CSVWriter::writeLine(double t, const Eigen::VectorXd& y) {
    if (!hasOutput) return;  // Do nothing if no file is valid
    // Write the time and the corresponding solution vector to the file
    file << t;
    for (int i = 0; i < y.size(); ++i) {
        file << "," << y(i);
    }
    file << "\n";
}
