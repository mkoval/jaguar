#include <iostream>
#include <string>
#include <fstream>
#include <boost/assign/list_of.hpp>
#include <boost/shared_ptr.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <jaguar/jaguar_api.h>
#include <jaguar/jaguar_bridge.h>
#include <jaguar/jaguar_helper.h>

using namespace jaguar;
using namespace testing;

TEST(JaguarHelper, pack_idMatchesDatasheet)
{
	uint32_t id = pack_id(5, Manufacturer::kTexasInstruments, DeviceType::kMotorController,
                          APIClass::kVoltageControl, VoltageControl::kVoltageSet);
	ASSERT_EQ(0x02020085, id);
}

TEST(JaguarHelper, pack_ackMatchesDatasheet)
{
	uint32_t id = pack_ack(5, Manufacturer::kTexasInstruments, DeviceType::kMotorController);
	ASSERT_EQ(0x02022005, id);
}
