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

using boost::assign::list_of;

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

TEST(JaguarHelper, fixedPointParser)
{
    std::vector<uint8_t> data = list_of(0xA3)(0x30)(0x00)(0x00);
    double output;
    boost::spirit::qi::parse(
        data.begin(), data.end(),
        u16p16_parser(),
        output
    );
    ASSERT_EQ(output, 0.19);
}