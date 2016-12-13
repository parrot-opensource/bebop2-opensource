
#include "aai.h"
#include "aai_hw.h"
#include "aai_group.h"

static int aai_group_is_started(struct card_data_t *aai, int group_nb)
{
	return !!(aai->groups_started & (1 << group_nb));
}

int aai_group_set(struct card_data_t *aai, char *name, int group_nb)
{
	int err = 0;
	int reg_nb;
	int reg_shift;
	uint32_t cfg;
	struct aai_device_t *chan = NULL;
	int i;

	if (group_nb < 0 || group_nb >= 0xf) {
		dev_warn(aai->dev, "%s: invalid group number (%d)\n",
			 __func__, group_nb);
		err = -EINVAL;
		goto exit;
	}

	/* channel lookup */
	for (i = 0; i < aai->chans_nb; i++) {
		chan = &aai->chans[i];
		if (!strcmp(chan->name, name))
			break;
	}
	if (i == aai->chans_nb) {
		dev_warn(aai->dev, "%s: invalid channel: '%s'\n",
			 __func__, name);
		err = -EINVAL;
		goto exit;
	}

	if (aai_group_is_started(aai, group_nb)) {
		dev_warn(aai->dev, "%s: can't add '%s' to already started group %d\n",
			 __func__, name, group_nb);
		err = -EBUSY;
		goto exit;
	}

	pr_err(" >> %s: '%s' %d\n", __func__, name, group_nb);

	/*
	 * Mark channel as belonging to group_nb
	 */
	if (chan->group != -1) {
		dev_warn(aai->dev, "%s: %s already synchronized to group %d\n",
			 __func__, chan->name, chan->group);
		err = -EINVAL;
		goto exit;
	}
	chan->group = group_nb;

	/*
	 * Write relevant register
	 */
	reg_nb = chan->fifo.fifo_id >> 3;
	reg_shift = (chan->fifo.fifo_id % 8) * 4;
	if (chan->direction & AAI_TX) {
		cfg = aai_readreg(aai, AAI_MAIN_IT_GROUPS_TX(reg_nb));
		cfg &= ~(0xf << reg_shift);
		cfg |= group_nb << reg_shift;
		aai_writereg(aai, cfg, AAI_MAIN_IT_GROUPS_TX(reg_nb), 0);
	} else if (chan->direction & AAI_RX) {
		cfg = aai_readreg(aai, AAI_MAIN_IT_GROUPS_RX(reg_nb));
		cfg &= ~(0xf << reg_shift);
		cfg |= group_nb << reg_shift;
		aai_writereg(aai, cfg, AAI_MAIN_IT_GROUPS_RX(reg_nb), 0);
	} else {
		dev_err(aai->dev, "wrong channel direction\n");
		err = -EINVAL;
		goto exit;
	}

exit:
	return err;
}

int aai_group_unset(struct card_data_t *aai, struct aai_device_t *chan)
{
	int err = 0;
	int reg_nb;
	int reg_shift;
	uint32_t cfg;
	int group_nb = chan->group;

	if (chan->group == -1) {
		/* nothing to do */
		err = 0;
		goto exit;
	}

	reg_nb = chan->fifo.fifo_id >> 3;
	reg_shift = (chan->fifo.fifo_id % 8) * 4;
	if (chan->direction & AAI_TX) {
		cfg = aai_readreg(aai, AAI_MAIN_IT_GROUPS_TX(reg_nb));
		cfg |= 0xf << reg_shift;	/* reset value */
		aai_writereg(aai, cfg, AAI_MAIN_IT_GROUPS_TX(reg_nb), 0);
	} else if (chan->direction & AAI_RX) {
		cfg = aai_readreg(aai, AAI_MAIN_IT_GROUPS_RX(reg_nb));
		cfg |= 0xf << reg_shift;	/* reset value */
		aai_writereg(aai, cfg, AAI_MAIN_IT_GROUPS_RX(reg_nb), 0);
	} else {
		dev_err(aai->dev, "wrong channel direction\n");
		err = -EINVAL;
		goto exit;
	}

	chan->group = -1;

	/* update global ref count */
	aai->groups_ref_count[group_nb]--;
	if (aai->groups_ref_count[group_nb] < 0)
		aai->groups_ref_count[group_nb] = 0;

	if (!aai->groups_ref_count[group_nb])
		aai_group_stop(aai, group_nb);

exit:
	return err;
}

int aai_group_start(struct card_data_t *aai, int group_nb)
{
	int err = 0;

	if (group_nb < 0 || group_nb >= NB_GROUPS) {
		dev_warn(aai->dev, "%s: invalid group number (%d)\n",
			 __func__, group_nb);
		err = -EINVAL;
		goto exit;
	}

	if (!aai->groups_ref_count[group_nb]) {
		dev_warn(aai->dev, "%s: no channel in this group, abort\n",
			 __func__);
		err = -EINVAL;
		goto exit;
	}

	if (aai->groups_started & 1 << group_nb) {
		dev_warn(aai->dev, "%s: group %d already started\n",
			 __func__, group_nb);
		err = -EBUSY;
		goto exit;
	}

	aai->groups_started |= 1 << group_nb;
	aai_writereg(aai, aai->groups_started, AAI_MAIN_IT_GROUPS_START, 0);

	aai_group_it_en(aai, group_nb);

exit:
	return err;
}

int aai_group_stop(struct card_data_t *aai, int group_nb)
{
	int err = 0;

	if (group_nb < 0 || group_nb >= 0xf) {
		dev_warn(aai->dev, "%s: invalid group number (%d)\n",
			 __func__, group_nb);
		err = -EINVAL;
		goto exit;
	}

	aai->groups_started &= ~(1 << group_nb);
	aai_writereg(aai, aai->groups_started, AAI_MAIN_IT_GROUPS_STOP, 0);

	aai_group_it_dis(aai, group_nb);

exit:
	return err;
}

