
#ifndef _AAI_CLK_H_
#define _AAI_CLK_H_

int aai_get_clk(struct card_data_t *aai, enum aai_clk);
void aai_set_clk(struct card_data_t *aai, enum aai_clk, int rate);
void aai_release_clk(struct card_data_t *aai, enum aai_clk);

#endif

