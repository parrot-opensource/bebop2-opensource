
#ifndef _AAI_GROUP_H_
#define _AAI_GROUP_H_

int aai_group_set(struct card_data_t *aai, char *name, int group_nb);
int aai_group_unset(struct card_data_t *aai, struct aai_device_t *chan);
int aai_group_start(struct card_data_t *aai, int group_nb);
int aai_group_stop(struct card_data_t *aai, int group_nb);

#endif

