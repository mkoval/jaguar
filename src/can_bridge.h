#ifndef CANBRIDGE_H_
#define CANBRIDGE_H_

#include <exception>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>

namespace can {

class Token;

typedef boost::shared_ptr<Token> TokenPtr;

class CANMessage {
public:
    uint32_t id;
    std::vector<uint8_t> payload;

    CANMessage(uint32_t p_id, std::vector<uint8_t> const &p_payload)
        : id(p_id), payload(p_payload)
    {
    }

    virtual ~CANMessage(void)
    {
    }
};

class CANBridge {
public:
    virtual void send(CANMessage const &message) = 0;
    virtual TokenPtr recv(uint32_t id) = 0;
};

class Token : boost::noncopyable
{
public:
	Token(void) {}
	virtual ~Token(void) {}
	virtual void block(void) = 0;
    virtual bool ready(void) const = 0;
	virtual boost::shared_ptr<CANMessage const> message(void) const = 0;
};

class CANException : public std::exception {
public:
    CANException(std::string what) : m_what(what) {}
    CANException(int code, std::string what) : m_code(code), m_what(what) {}
	virtual ~CANException(void) throw() {}
    virtual char const* what() const throw() { return m_what.c_str(); }
    virtual int code() const throw() { return m_code; }

private:
    int m_code;
    std::string m_what;
};

};

#endif