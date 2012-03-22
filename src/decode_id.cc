
#include <string>
#include <jaguar/jaguar_helper.h>

int main(int argc, char *argv[])
{
	if (argc != 2) {
		std::cerr << "err: no argument\n"
			  << "usage: ./device_id <raw hex id, le>"
			  << std::endl;
		return -1;
	}

	std::stringstream s;
	s << argv[1];
	uint32_t raw_id;
	s >> std::hex >> raw_id;

	jaguar::CANId id(le32toh(raw_id));

	std::cout << le32toh(raw_id) << std::endl
		  << id << std::endl;

	return 0;
}
