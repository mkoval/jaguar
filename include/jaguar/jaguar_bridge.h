#ifndef JAGUAR_BRIDGE_H_
#define JAGUAR_BRIDGE_H_

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/signal.hpp>
#include <boost/thread.hpp>
#include <map>
#include <vector>
#include <stdint.h>
#include "can_bridge.h"
#include "jaguar_helper.h"

typedef boost::asio::buffers_iterator<
    boost::asio::streambuf::const_buffers_type> asio_iterator;


namespace can {

enum ReceiveState {
    kWaiting,
    kLength,
    kPayload,
    kComplete
};

template<typename T>
class masked_number
{
public:
	typedef T type;

	masked_number(T const& num_, T const& mask_)
	: num(num_), mask(mask_)
	{
		assert((num_ & ~mask_) == 0);
	}

	template <class U>
	masked_number(masked_number<U> const &mn)
	: num(mn.num), mask(mn.mask)
	{}

	bool matches(T val)
	{
		return (val & mask) == num;
	}

	T num;
	T mask;
};

template<typename T>
masked_number<T> make_masked_number(T const &n, T const &m)
{
	return masked_number<T>(n, m);
}

class JaguarToken;

class JaguarBridge : public CANBridge
{
public:
    JaguarBridge(std::string port);
    virtual ~JaguarBridge(void);

    virtual void send(CANMessage const &message);
    virtual TokenPtr recv(uint32_t id);

    virtual void attach_callback(uint32_t id, recv_callback cb);
    virtual void attach_callback(uint32_t id, uint32_t id_mask,
		    recv_callback cb);
    //virtual bool detach_callback(uint32_t id, recv_callback cb);

private:
    typedef boost::signal<void (boost::shared_ptr<CANMessage>)> callback_signal;
    typedef boost::shared_ptr<callback_signal> callback_signal_ptr;

    typedef std::map<uint32_t, callback_signal_ptr> callback_table;

    typedef std::pair<masked_number<uint32_t>, callback_signal_ptr> mask_callback;
    typedef std::list<mask_callback> callback_list;

    typedef boost::shared_ptr<JaguarToken> token_ptr;
    typedef std::map<uint32_t, token_ptr>  token_table;

    static uint8_t const kSOF, kESC;
    static uint8_t const kSOFESC, kESCESC;
    static size_t const kReceiveBufferLength;

    boost::asio::io_service  io_;
    boost::asio::serial_port serial_;

    boost::thread recv_thread_;
    std::vector<uint8_t> recv_buffer_;
    callback_table callbacks_;
    callback_list  callbacks_list_;
    boost::mutex callback_mutex_;
    token_table tokens_;

    std::vector<uint8_t> packet_;
    ReceiveState state_;
    size_t length_;
    bool escape_;

    boost::shared_ptr<CANMessage> recv_byte(uint8_t byte);
    void recv_handle(boost::system::error_code const& error, size_t count);
    void recv_message(boost::shared_ptr<CANMessage> msg);

    boost::shared_ptr<CANMessage> unpack_packet(std::vector<uint8_t> const &packet);
    size_t encode_bytes(uint8_t const *bytes, size_t length, std::vector<uint8_t> &buffer);
};

class JaguarToken : public Token {
public:
    virtual ~JaguarToken(void);
    virtual void block(void);
    virtual bool timed_block(boost::posix_time::time_duration const &duration);
    virtual boost::shared_ptr<CANMessage const> message(void) const;
    virtual bool ready(void) const;

private:    
    boost::shared_ptr<CANMessage> message_;
    boost::condition_variable cond_;
    boost::mutex mutex_;
    bool done_;

    JaguarToken(void);
    virtual void unblock(boost::shared_ptr<CANMessage> message);

    friend class JaguarBridge;
};

};

#endif

/* vim set: sts=4 et sw=4 ts=4 : */
