#include <iostream>
#include <string>
#include <boost/bind.hpp>
#include <boost/proto/proto.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/typeof/typeof.hpp>

#include <boost/spirit/include/karma.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/qi_binary.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>

namespace mpl = boost::mpl;
namespace fusion = boost::fusion;
namespace proto = boost::proto;

using proto::_;
using boost::spirit::byte_;
using boost::spirit::little_word;
using boost::spirit::little_dword;

struct StatusDomain;

template <typename T, typename P, typename G>
struct Status
{
    Status(P p_parser, G p_generator)
        : parser(p_parser), generator(p_generator) {}

    friend std::ostream &operator<<(std::ostream &s, Status) {
        return s << "Status";
    }

    typedef P ParserType;
    typedef G GeneratorType;
    P const parser;
    G const generator;
    std::string a;
};

template <typename T, typename P, typename G>
Status<T, P, G> make_status(P parser, G generator)
{
    return Status<T, P, G>(parser, generator);
}

#define MAKE_STATUS(name, value) proto::terminal<BOOST_TYPEOF(value)>::type name = { value }
#if 0
MAKE_STATUS(OutputVoltagePercentage, make_status<int16_t>(little_word, byte_(1) << byte_(2)));
MAKE_STATUS(BusVoltage, make_status<int16_t>(little_word, byte_(5) << byte_(6)));
MAKE_STATUS(Temperature, make_status<int16_t>(little_word, byte_(7) << byte_(8)));
MAKE_STATUS(Position, make_status<int32_t>(little_dword, byte_(9) << byte_(10) << byte_(11) << byte_(12)));
MAKE_STATUS(Speed, make_status<int32_t>(little_dword, byte_(13) << byte_(14) << byte_(15) << byte_(16)));
MAKE_STATUS(LimitNonClearing, make_status<uint8_t>(byte_, byte_(17)));
MAKE_STATUS(LimitClearing, make_status<uint8_t>(byte_, byte_(18)));
MAKE_STATUS(OutputVoltageVolts, make_status<int16_t>(little_word, byte_(22) << byte_(23)));
MAKE_STATUS(CurrentFaultCounter, make_status<uint8_t>(byte_, byte_(24)));
MAKE_STATUS(TemperatureFaultCounter, make_status<uint8_t>(byte_, byte_(25)));
MAKE_STATUS(GateFaultCounter, make_status<uint8_t>(byte_, byte_(27)));
MAKE_STATUS(CommunicationFaultCounter, make_status<uint8_t>(byte_, byte_(28)));
#endif
MAKE_STATUS(abc, (make_status<int, std::string, std::string>("a", "a")));
MAKE_STATUS(def, (make_status<int, std::string, std::string>("b", "b")));

struct Value
  : proto::when<
        proto::terminal< _ >
      , proto::_value
    >
{};

struct display {
    template<typename T>
    void operator()(T const &t) const {
        std::cout << t << std::endl;
    }
};

template <typename T>
typename T::GeneratorType get_generator(T const &t) {
    return t.generator;
}

int main(int argc, char **argv)
{
    fusion::for_each(
        fusion::transform(
            proto::flatten(
                abc << abc << abc
            ),
            Value()
        ),
        display()
    );

#if 0
    Value get_value;
    auto x = get_value(abc);
    std::cout << x << std::endl;
#endif

    return 0;
}
