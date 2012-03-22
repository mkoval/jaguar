#ifndef CANBRIDGE_H_
#define CANBRIDGE_H_

#include <exception>
#include <stdint.h>
#include <string>
#include <vector>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

namespace can {

class Token;

typedef boost::shared_ptr<Token> TokenPtr;

class CANMessage {
public:
    typedef boost::shared_ptr<CANMessage> Ptr;

    CANMessage(uint32_t p_id)
        : id(p_id) {}

    CANMessage(uint32_t p_id, std::vector<uint8_t> const &p_payload)
        : id(p_id), payload(p_payload) {}

    virtual ~CANMessage(void) {}

    uint32_t id;
    std::vector<uint8_t> payload;
};

class CANBridge {
public:
    typedef boost::function<void (CANMessage::Ptr)> Callback;
    typedef boost::function<void (CANMessage::Ptr)> recv_callback;
    typedef void (error_callback)(char const *func, char const *file, unsigned line, std::string const &msg);

    virtual void send(CANMessage const &message) = 0;
    virtual TokenPtr recv(uint32_t id) = 0;
    virtual void attach_callback(uint32_t id, recv_callback cb) = 0;
    virtual void attach_callback(uint32_t id, uint32_t id_mask,
		    recv_callback cb) = 0;
    virtual void attach_callback(error_callback cb) = 0;
};

class Token : boost::noncopyable
{
public:
    typedef boost::shared_ptr<Token> Ptr;

	Token(void) {}
	virtual ~Token(void) {}
	virtual void block(void) = 0;
	virtual bool timed_block(boost::posix_time::time_duration const &duration) = 0;
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

/* vim: set et sts=4 sw=4 ts=4: */
