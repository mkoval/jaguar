#include <iostream>
#include <string>
#include <fstream>
#include <boost/assign/list_of.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function_equal.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <jaguar/can_bridge.h>
#include <jaguar/jaguar.h>
#include <jaguar/jaguar_api.h>
#include <jaguar/jaguar_helper.h>

using namespace jaguar;
using namespace testing;

using boost::assign::list_of;
using boost::spirit::byte_;
using boost::spirit::little_word;
using boost::spirit::little_dword;

class CANBridgeMock : public can::CANBridge
{
public:
    MOCK_METHOD1(send, void (can::CANMessage const &message));
    MOCK_METHOD1(recv, can::TokenPtr (uint32_t id));
    MOCK_METHOD2(attach_callback, void (uint32_t id, can::CANBridge::recv_callback cb));
};

class MockToken : public can::Token {
public:
    typedef boost::shared_ptr<MockToken> Ptr;

    MOCK_METHOD0(block, void (void));
    MOCK_CONST_METHOD0(ready, bool (void));
    MOCK_CONST_METHOD0(message, boost::shared_ptr<can::CANMessage const> (void));
};

JAGUAR_MAKE_STATUS(Mock1, uint8_t, byte_(0x01), byte_);
JAGUAR_MAKE_STATUS(Mock2, uint8_t, byte_(0x02), byte_);

class JaguarTest : public testing::Test
{
protected:
    virtual void SetUp(void)
    {
        num_ = 5;
        bridge_ = boost::make_shared<CANBridgeMock>();
        jaguar_ = boost::shared_ptr<Jaguar>(new jaguar::Jaguar(*bridge_, num_));

        callback1_ptr_ = boost::bind(&JaguarTest::callback1, this, _1);
        callback2_ptr_ = boost::bind(&JaguarTest::callback2, this, _1);
    }

    MOCK_METHOD1(callback1, void (uint8_t));
    MOCK_METHOD1(callback2, void (uint8_t));

    boost::function<void (uint8_t)> callback1_ptr_;
    boost::function<void (uint8_t)> callback2_ptr_;

    uint8_t num_;
    boost::shared_ptr<CANBridgeMock> bridge_;
    boost::shared_ptr<Jaguar>        jaguar_;
    can::Token::Ptr                  token_;
};

TEST_F(JaguarTest, config_brushes_set)
{
    uint32_t const request_id = pack_id(num_,
        Manufacturer::kTexasInstruments,
        DeviceType::kMotorController,
        APIClass::kConfiguration,
        Configuration::kNumberOfBrushes
    );
    uint32_t const ack_id = pack_ack(num_,
        Manufacturer::kTexasInstruments,
        DeviceType::kMotorController
    );

    EXPECT_CALL(*bridge_, send(AllOf(
        Field(&can::CANMessage::id, request_id),
        Field(&can::CANMessage::payload, ElementsAre(0x12))
    )));
    EXPECT_CALL(*bridge_, recv(ack_id)).WillOnce(Return(token_));

    jaguar_->config_brushes_set(0x12);
}

TEST_F(JaguarTest, Status_read)
{
    std::vector<uint8_t> payload = list_of(0x12);
    Status::Ptr status = Mock1(callback1_ptr_);

    EXPECT_CALL(*this, callback1(0x12));

    status->read(&payload.front(), &payload.back() + 1);
}

TEST_F(JaguarTest, Status_write)
{
    std::vector<uint8_t> payload;
    std::back_insert_iterator<std::vector<uint8_t> > it(payload);
    Status::Ptr status = Mock1(callback1_ptr_);

    status->write(it);

    ASSERT_THAT(payload, ElementsAre(0x01));
}

TEST_F(JaguarTest, AggregateStatus_readChains)
{
    std::vector<uint8_t> payload = list_of(0xF1)(0xF2);
    AggregateStatus status = Mock1(callback1_ptr_) << Mock2(callback2_ptr_);

    EXPECT_CALL(*this, callback1(0xF1));
    EXPECT_CALL(*this, callback2(0xF2));

    status.read(&payload.front(), &payload.back() + 1);
}

TEST_F(JaguarTest, AggregateStatus_writeChains)
{
    std::vector<uint8_t> payload;
    std::back_insert_iterator<std::vector<uint8_t> > it(payload);
    AggregateStatus status = Mock1(callback1_ptr_) << Mock2(callback2_ptr_);

    status.write(it);

    ASSERT_THAT(payload, ElementsAre(0x01, 0x02));
}