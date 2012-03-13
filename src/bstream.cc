#include <streambuf>

class binary_istream : public std::streambuf
{
public:
	binary_istream(void const *data, size_t length);

private:	
    int_type underflow(void);
    int_type uflow(void);
    int_type pbackfail(int_type ch);
    std::streamsize showmanyc(void);

	void const *begin_;
	void const *end_
	void       *current_;
}

binary_istream::binary_istream(void const *data, size_t length)
	: begin_(data),
	  end_(data + length),
	  current_(data)
{
}

char_array_buffer::int_type binary_istream::char_array_buffer::underflow(void)
{
	if (current_ == end_) {
		return traits_type::eof();
	} else {
		return 
	}
}

