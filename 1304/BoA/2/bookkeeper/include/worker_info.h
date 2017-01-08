#ifndef WORKER_INFO_H
#define WORKER_INFO_H

#include <string>

struct WorkerInfo {

    std::string firstName;
    std::string lastName;
    long salary;
    bool isAppeared;

    WorkerInfo(const std::string& firstName, const std::string& lastName, long salary) {
        this->firstName = firstName;
        this->lastName = lastName;
        this->salary = salary;
        isAppeared = false;
    }

};

#endif /* WORKER_INFO_H */
