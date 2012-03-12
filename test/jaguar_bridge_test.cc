#include <iostream>
#include <string>
#include <fstream>
#include <boost/assign/list_of.hpp>
#include <boost/shared_ptr.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "jaguar_bridge.h"

using namespace testing;
using boost::assign::list_of;

static std::string const kPathReal = "/dev/ttys002";
static std::string const kPathTest = "/dev/ttys003";
static std::vector<uint8_t> const kEmptyPayload(0);

class JaguarBridgeTest : public ::testing::Test
{
protected:
	virtual void SetUp(void)
	{
		stream_.open(kPathTest.c_str(), std::ios::in  | std::ios::ate
		                              | std::ios::out | std::ios::app
		                              | std::ios::binary);
		bridge_ = new can::JaguarBridge(kPathReal);

		called1a_ = 0;
		called1b_ = 0;
		called2_  = 0;
	}

	virtual void TearDown(void)
	{
		delete bridge_;
		stream_.close();
	}

	void callback1a(boost::shared_ptr<can::CANMessage> msg)
	{
		++called1a_;
		ASSERT_EQ(msg->id, 0x00000001);
		ASSERT_THAT(msg->payload, ElementsAre(0x01, 0x01));
	}

	void callback1b(boost::shared_ptr<can::CANMessage> msg)
	{
		++called1b_;
		ASSERT_EQ(msg->id, 0x00000001);
		ASSERT_THAT(msg->payload, ElementsAre(0x01, 0x01));
	}

	void callback2(boost::shared_ptr<can::CANMessage> msg)
	{
		++called2_;
		ASSERT_EQ(msg->id, 0x00000002);
		ASSERT_THAT(msg->payload, ElementsAre(0x02, 0x02));
	}

	can::JaguarBridge *bridge_;
	std::fstream stream_;
	int called1a_, called1b_, called2_;
};

TEST_F(JaguarBridgeTest, sendIncludesSOF)
{
	bridge_->send(can::CANMessage(0x00000000, kEmptyPayload));

	char sof;
	stream_.get(sof);
	ASSERT_EQ(sof, '\xFF');
	ASSERT_TRUE(stream_.good());
}

TEST_F(JaguarBridgeTest, sendIncludesIdentifier)
{
	bridge_->send(can::CANMessage(0x11223344, kEmptyPayload));

	std::vector<char> packet(6);
	stream_.read(&packet[0], packet.size());
	char const *expected = "\xFF\x04\x44\x33\x22\x11";
	ASSERT_THAT(packet, ElementsAreArray(expected, packet.size()));
	ASSERT_TRUE(stream_.good());
}

TEST_F(JaguarBridgeTest, sendIncludesPayload)
{
	std::vector<uint8_t> payload = list_of(0x11)(0x22);
	bridge_->send(can::CANMessage(0x00000000, payload));

	std::vector<char> packet(8);
	stream_.read(&packet[0], packet.size());
	char const *expected = "\xFF\x06\x00\x00\x00\x00\x11\x22";
	ASSERT_THAT(packet, ElementsAreArray(expected, packet.size()));
	ASSERT_TRUE(stream_.good());

}

TEST_F(JaguarBridgeTest, sendEscapesSOF)
{
	bridge_->send(can::CANMessage(0x000000FF, kEmptyPayload));

	std::vector<char> packet(7);
	stream_.read(&packet[0], packet.size());
	char const *expected = "\xFF\x04\xFE\xFE\x00\x00";
	ASSERT_THAT(packet, ElementsAreArray(expected, packet.size()));
	ASSERT_TRUE(stream_.good());
}

TEST_F(JaguarBridgeTest, sendEscapesESC)
{
	bridge_->send(can::CANMessage(0x000000FE, kEmptyPayload));

	std::vector<char> packet(7);
	stream_.read(&packet[0], packet.size());
	char const *expected = "\xFF\x04\xFE\xFD\x00\x00";
	ASSERT_THAT(packet, ElementsAreArray(expected, packet.size()));
	ASSERT_TRUE(stream_.good());
}

TEST_F(JaguarBridgeTest, attach_callbackMatchingCallbackInvoked)
{
	bridge_->attach_callback(0x00000001, boost::bind(&JaguarBridgeTest::callback1a, this, _1));

	char const *packet = "\xFF\x06\x01\x00\x00\x00\x01\x01";
	stream_.write(packet, 8);
	stream_.flush();
	sleep(1);
	ASSERT_EQ(called1a_, 1);
}

TEST_F(JaguarBridgeTest, attach_callbackMatchingMultipleCallbacksInvoked)
{
	bridge_->attach_callback(0x00000001, boost::bind(&JaguarBridgeTest::callback1a, this, _1));
	bridge_->attach_callback(0x00000001, boost::bind(&JaguarBridgeTest::callback1b, this, _1));

	char const *packet = "\xFF\x06\x01\x00\x00\x00\x01\x01";
	stream_.write(packet, 8);
	stream_.flush();
	sleep(1);
	ASSERT_EQ(called1a_, 1);
	ASSERT_EQ(called1b_, 1);
}

TEST_F(JaguarBridgeTest, attach_callbackMismatchedCallbackNotInvoked)
{
	bridge_->attach_callback(0x00000002, boost::bind(&JaguarBridgeTest::callback2, this, _1));

	char const *packet = "\xFF\x06\x01\x00\x00\x00\x01\x01";
	stream_.write(packet, 8);
	stream_.flush();
	sleep(1);
	ASSERT_EQ(called1b_, 0);
}

TEST_F(JaguarBridgeTest, recvTokenBlocksUntilReady)
{
	can::TokenPtr token = bridge_->recv(0x00000001);
	ASSERT_FALSE(token->ready());

	char const *packet = "\xFF\x06\x01\x00\x00\x00\x01\x01";
	stream_.write(packet, 8);
	stream_.flush();
	token->block();

	ASSERT_TRUE(token->ready());
}

TEST_F(JaguarBridgeTest, recvTokenContainsMessage)
{
	can::TokenPtr token = bridge_->recv(0x00000001);

	char const *packet = "\xFF\x06\x01\x00\x00\x00\x01\x01";
	stream_.write(packet, 8);
	stream_.flush();
	token->block();

	std::vector<uint8_t> const  payload_expected = list_of(0x01)(0x01);
	std::vector<uint8_t> const &payload_actual   = token->message()->payload;
	ASSERT_EQ(token->message()->id, 0x00000001);
	//ASSERT_EQ(payload_actual, payload_expected);
}

TEST_F(JaguarBridgeTest, recvMismatchedTokensNotReady)
{
	can::TokenPtr token1 = bridge_->recv(0x00000001);
	can::TokenPtr token2 = bridge_->recv(0x00000002);

	char const *packet = "\xFF\x06\x01\x00\x00\x00\x01\x01";
	stream_.write(packet, 8);
	stream_.flush();
	token1->block();	

	ASSERT_TRUE(token1->ready());
	ASSERT_FALSE(token2->ready());
}

TEST_F(JaguarBridgeTest, recvResetsTokenStatus)
{
	can::TokenPtr token1 = bridge_->recv(0x00000001);

	char const *packet = "\xFF\x06\x01\x00\x00\x00\x01\x01";
	stream_.write(packet, 8);
	stream_.flush();
	token1->block();	

	ASSERT_TRUE(token1->ready());
	can::TokenPtr token2 = bridge_->recv(0x00000001);
	ASSERT_FALSE(token2->ready());

	stream_.write(packet, 8);
	stream_.flush();
	token2->block();	

	ASSERT_TRUE(token2->ready());
}