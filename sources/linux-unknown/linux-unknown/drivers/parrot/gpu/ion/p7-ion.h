#ifndef P7_ION_H
#define P7_ION_H

enum{
	P7ION_GET_BUFFER_INFO
};

struct p7ion_buffer_info{
	struct ion_handle* handle;
	unsigned long int phys_address;
	size_t len;
};


#endif /*P7_ION_H*/
