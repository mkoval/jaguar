#include <iostream>
#include <string>
#include <fstream>
#include <boost/shared_ptr.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "jaguar_bridge.h"

using namespace testing;

static std::string const kPathInput  = "/dev/ptyp0";
static std::string const kPathOutput = "/dev/ttyp0";

class JaguarBridgeTest : public ::testing::Test
{
protected:
	virtual void SetUp(void)
	{
		stream_.open(kPathInput.c_str(), std::ios::in | std::ios::ate);
		bridge_ = new can::JaguarBridge(kPathOutput);
	}

	virtual void TearDown(void)
	{
		delete bridge_;
		stream_.close();
	}

	can::JaguarBridge *bridge_;
	std::fstream stream_;
};

TEST_F(JaguarBridgeTest, sendIncludesSOF)
{
	bridge_->send(0, NULL, 0);

	char sof;
	stream_.get(sof);
	ASSERT_EQ(sof, '\xFF');
	ASSERT_TRUE(stream_.good());
}

TEST_F(JaguarBridgeTest, sendIncludesIdentifier)
{
	bridge_->send(0x11223344, NULL, 0);

	std::vector<char> packet(6);
	stream_.read(&packet[0], packet.size());
	char const *expected = "\xFF\x04\x44\x33\x22\x11";
	ASSERT_THAT(packet, ElementsAreArray(expected, packet.size()));
	ASSERT_TRUE(stream_.good());
}

TEST_F(JaguarBridgeTest, sendIncludesPayload)
{
	uint8_t const payload[] = { '\x11', '\x22' };
	bridge_->send(0x00000000, &payload, 2);

	std::vector<char> packet(8);
	stream_.read(&packet[0], packet.size());
	char const *expected = "\xFF\x06\x00\x00\x00\x00\x11\x22";
	ASSERT_THAT(packet, ElementsAreArray(expected, packet.size()));
	ASSERT_TRUE(stream_.good());

}

TEST_F(JaguarBridgeTest, sendEscapesSOF)
{
	bridge_->send(0x000000FF, NULL, 0);

	std::vector<char> packet(7);
	stream_.read(&packet[0], packet.size());
	char const *expected = "\xFF\x04\xFE\xFE\x00\x00";
	ASSERT_THAT(packet, ElementsAreArray(expected, packet.size()));
	ASSERT_TRUE(stream_.good());
}

TEST_F(JaguarBridgeTest, sendEscapesESC)
{
	bridge_->send(0x000000FE, NULL, 0);

	std::vector<char> packet(7);
	stream_.read(&packet[0], packet.size());
	char const *expected = "\xFF\x04\xFE\xFD\x00\x00";
	ASSERT_THAT(packet, ElementsAreArray(expected, packet.size()));
	ASSERT_TRUE(stream_.good());
}


