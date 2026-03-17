#ifndef SERIALMANAGER_H
#define SERIALMANAGER_H

#include <fcntl.h>

#include <string>

void serialInit(std::string fiveG, std::string mesh);

int receiveData(char* buffer, size_t buffer_size);

void fiveG(void);

void mesh(void);

int sendData(std::string msg);

std::string readSerialInfo();

#endif  // SERIALMANAGER_H
