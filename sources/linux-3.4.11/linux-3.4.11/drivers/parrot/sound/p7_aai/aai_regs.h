
#ifndef _AAI_REGS_H_
#define _AAI_REGS_H_

#ifdef CONFIG_SND_AAI_DEBUG
uint32_t aai_reg_addr(int index);
const char *aai_reg_name(int index);
void aai_dump_regs(struct card_data_t *aai, uint32_t blocks);
int aai_reg_canread(int index);
int aai_reg_canwrite(int index);
static inline int aai_reg_canreadback(int index)
{
	return aai_reg_canread(index);
}

#else

static inline uint32_t aai_reg_addr(int index) { return index; }
static inline const char *aai_reg_name(int index)
{
	return "noname";
}

static inline void aai_dump_regs(struct card_data_t *aai, uint32_t blocks) {}
static inline int aai_reg_canread(int index) { return 1; }
static inline int aai_reg_canreadback(int index) { return 0; }
static inline int aai_reg_canwrite(int index) { return 1; }
#endif

void aai_reset_regs(struct card_data_t *aai);

#endif

