#ifndef RUNNER_HH
#define RUNNER_HH

#include "../Reader/Reader.hh"  // Assuming Reader is responsible for reading YAML
#include <string>

class Runner {
private:
    std::string config_file;
    Reader Rdr;  // Assuming Reader can handle the YAML parsing

public:
    //constructor
    Runner(const std::string& config_file);

    //destructor 
    ~Runner();

    //run method
    Eigen::VectorXd run();

};


#endif // RUNNER_HH