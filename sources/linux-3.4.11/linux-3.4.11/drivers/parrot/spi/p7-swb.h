

#ifndef _P7_SPI_SWB_H
#define _P7_SPI_SWB_H

struct p7spi_core;
enum p7spi_core_type;

struct p7swb_desc {
	bool                        in_used;
	bool                        out_used;
	struct p7spi_core const *   owner;
};

enum p7swb_direction {
	P7_SWB_DIR_IN,
	P7_SWB_DIR_OUT,
	P7_SWB_DIR_BOTH,
};

enum p7swb_function {
	P7_SWB_SPI_FIRST_FUNC,
	P7_SWB_SPI_CLK,
	P7_SWB_SPI_SS,
	P7_SWB_SPI_DATA0,
	P7_SWB_SPI_DATA1,
	P7_SWB_SPI_DATA2,
	P7_SWB_SPI_DATA3,
	P7_SWB_SPI_LAST_FUNC,

	P7_SWB_MPEG_FIRST_FUNC,
	P7_SWB_MPEG_CLK,
	P7_SWB_MPEG_DATA,
	P7_SWB_MPEG_VALID,
	P7_SWB_MPEG_SYNC,
	P7_SWB_MPEG_LAST_FUNC,

	P7_SWB_FIRST_STATE,
	P7_SWB_LOW,
	P7_SWB_HIGH,
	P7_SWB_Z,
	P7_SWB_LAST_STATE,

	P7_SWB_FUNCTION_LEN,
};

int p7swb_core_offset(enum p7spi_core_type type);
extern int p7swb_request_pad(int core_id, enum p7spi_core_type,
                               void const * const owner, unsigned int pad,
                               enum p7swb_direction dir, enum p7swb_function func);
extern void p7swb_release_pad(void const * const owner, unsigned int pad,
                               enum p7swb_direction dir);
#endif
