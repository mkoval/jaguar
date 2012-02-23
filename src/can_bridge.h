#ifndef CANBRIDGE_H_
#define CANBRIDGE_H_

#include <exception>
#include <vector>

namespace can {

class CANBridge
{
public:
    virtual void send(uint32_t id, uint8_t const *data, size_t length) = 0;
    virtual uint32_t recv(uint8_t *data, size_t length) = 0;
};

class CANException : public std::exception {
public:
    CANException(std::string what) : m_what(what) {}
    virtual char const* what() const throw() { return m_what.c_str(); }

private:
    std::string m_what;
}; 

#endif
