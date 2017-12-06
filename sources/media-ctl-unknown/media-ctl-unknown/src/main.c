/*
 * Media controller test application
 *
 * Copyright (C) 2010-2011 Ideas on board SPRL
 *
 * Contact: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; either version 2.1 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/stat.h>

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <linux/types.h>
#include <linux/media.h>
#include <linux/v4l2-mediabus.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>

#include "mediactl.h"
#include "options.h"
#include "v4l2subdev.h"
#include "tools.h"

/* -----------------------------------------------------------------------------
 * Printing
 */

static void v4l2_subdev_print_format(struct media_entity *entity,
	unsigned int pad, enum v4l2_subdev_format_whence which)
{
	struct v4l2_mbus_framefmt format;
	struct v4l2_rect rect;
	int ret;

	ret = v4l2_subdev_get_format(entity, &format, pad, which);
	if (ret != 0)
		return;

	printf("[%s %ux%u", v4l2_subdev_pixelcode_to_string(format.code),
	       format.width, format.height);

	ret = v4l2_subdev_get_crop(entity, &rect, pad, which);
	if (ret == 0)
		printf(" (%u,%u)/%ux%u", rect.left, rect.top,
		       rect.width, rect.height);
	printf("]");
}

static const char *media_entity_type_to_string(unsigned type)
{
	static const struct {
		__u32 type;
		const char *name;
	} types[] = {
		{ MEDIA_ENT_T_DEVNODE, "Node" },
		{ MEDIA_ENT_T_V4L2_SUBDEV, "V4L2 subdev" },
	};

	unsigned int i;

	type &= MEDIA_ENT_TYPE_MASK;

	for (i = 0; i < ARRAY_SIZE(types); i++) {
		if (types[i].type == type)
			return types[i].name;
	}

	return "Unknown";
}

static const char *media_entity_subtype_to_string(unsigned type)
{
	static const char *node_types[] = {
		"Unknown",
		"V4L",
		"FB",
		"ALSA",
		"DVB",
	};
	static const char *subdev_types[] = {
		"Unknown",
		"Sensor",
		"Flash",
		"Lens",
	};

	unsigned int subtype = type & MEDIA_ENT_SUBTYPE_MASK;

	switch (type & MEDIA_ENT_TYPE_MASK) {
	case MEDIA_ENT_T_DEVNODE:
		if (subtype >= ARRAY_SIZE(node_types))
			subtype = 0;
		return node_types[subtype];

	case MEDIA_ENT_T_V4L2_SUBDEV:
		if (subtype >= ARRAY_SIZE(subdev_types))
			subtype = 0;
		return subdev_types[subtype];
	default:
		return node_types[0];
	}
}

static const char *media_pad_type_to_string(unsigned flag)
{
	static const struct {
		__u32 flag;
		const char *name;
	} flags[] = {
		{ MEDIA_PAD_FL_SINK, "Sink" },
		{ MEDIA_PAD_FL_SOURCE, "Source" },
	};

	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(flags); i++) {
		if (flags[i].flag & flag)
			return flags[i].name;
	}

	return "Unknown";
}

static void media_print_topology_dot(struct media_device *media)
{
	unsigned int i, j;

	printf("digraph board {\n");
	printf("\trankdir=TB\n");

	for (i = 0; i < media->entities_count; ++i) {
		struct media_entity *entity = &media->entities[i];
		unsigned int npads;

		switch (media_entity_type(entity)) {
		case MEDIA_ENT_T_DEVNODE:
			printf("\tn%08x [label=\"%s\\n%s\", shape=box, style=filled, "
			       "fillcolor=yellow]\n",
			       entity->info.id, entity->info.name, entity->devname);
			break;

		case MEDIA_ENT_T_V4L2_SUBDEV:
			printf("\tn%08x [label=\"{{", entity->info.id);

			for (j = 0, npads = 0; j < entity->info.pads; ++j) {
				if (!(entity->pads[j].flags & MEDIA_PAD_FL_SINK))
					continue;

				printf("%s<port%u> %u", npads ? " | " : "", j, j);
				npads++;
			}

			printf("} | %s", entity->info.name);
			if (entity->devname)
				printf("\\n%s", entity->devname);
			printf(" | {");

			for (j = 0, npads = 0; j < entity->info.pads; ++j) {
				if (!(entity->pads[j].flags & MEDIA_PAD_FL_SOURCE))
					continue;

				printf("%s<port%u> %u", npads ? " | " : "", j, j);
				npads++;
			}

			printf("}}\", shape=Mrecord, style=filled, fillcolor=green]\n");
			break;

		default:
			continue;
		}

		for (j = 0; j < entity->num_links; j++) {
			struct media_link *link = &entity->links[j];

			if (link->source->entity != entity)
				continue;

			printf("\tn%08x", link->source->entity->info.id);
			if (media_entity_type(link->source->entity) == MEDIA_ENT_T_V4L2_SUBDEV)
				printf(":port%u", link->source->index);
			printf(" -> ");
			printf("n%08x", link->sink->entity->info.id);
			if (media_entity_type(link->sink->entity) == MEDIA_ENT_T_V4L2_SUBDEV)
				printf(":port%u", link->sink->index);

			if (link->flags & MEDIA_LNK_FL_IMMUTABLE)
				printf(" [style=bold]");
			else if (!(link->flags & MEDIA_LNK_FL_ENABLED))
				printf(" [style=dashed]");
			printf("\n");
		}
	}

	printf("}\n");
}

static void media_print_topology_text(struct media_device *media)
{
	static const struct {
		__u32 flag;
		char *name;
	} link_flags[] = {
		{ MEDIA_LNK_FL_ENABLED, "ENABLED" },
		{ MEDIA_LNK_FL_IMMUTABLE, "IMMUTABLE" },
		{ MEDIA_LNK_FL_DYNAMIC, "DYNAMIC" },
	};

	unsigned int i, j, k;
	unsigned int padding;

	printf("Device topology\n");

	for (i = 0; i < media->entities_count; ++i) {
		struct media_entity *entity = &media->entities[i];

		padding = printf("- entity %u: ", entity->info.id);
		printf("%s (%u pad%s, %u link%s)\n", entity->info.name,
			entity->info.pads, entity->info.pads > 1 ? "s" : "",
			entity->num_links, entity->num_links > 1 ? "s" : "");
		printf("%*ctype %s subtype %s\n", padding, ' ',
			media_entity_type_to_string(entity->info.type),
			media_entity_subtype_to_string(entity->info.type));
		if (entity->devname[0])
			printf("%*cdevice node name %s\n", padding, ' ', entity->devname);

		for (j = 0; j < entity->info.pads; j++) {
			struct media_pad *pad = &entity->pads[j];

			printf("\tpad%u: %s ", j, media_pad_type_to_string(pad->flags));

			if (media_entity_type(entity) == MEDIA_ENT_T_V4L2_SUBDEV)
				v4l2_subdev_print_format(entity, j, V4L2_SUBDEV_FORMAT_ACTIVE);

			printf("\n");

			for (k = 0; k < entity->num_links; k++) {
				struct media_link *link = &entity->links[k];
				struct media_pad *source = link->source;
				struct media_pad *sink = link->sink;
				bool first = true;
				unsigned int i;

				if (source->entity == entity && source->index == j)
					printf("\t\t-> \"%s\":%u [",
						sink->entity->info.name, sink->index);
				else if (sink->entity == entity && sink->index == j)
					printf("\t\t<- \"%s\":%u [",
						source->entity->info.name, source->index);
				else
					continue;

				for (i = 0; i < ARRAY_SIZE(link_flags); i++) {
					if (!(link->flags & link_flags[i].flag))
						continue;
					if (!first)
						printf(",");
					printf("%s", link_flags[i].name);
					first = false;
				}

				printf("]\n");
			}
		}
		printf("\n");
	}
}

static void media_print_topology(struct media_device *media, int dot)
{
	if (dot)
		media_print_topology_dot(media);
	else
		media_print_topology_text(media);
}

int main(int argc, char **argv)
{
	struct media_device *media;
	int ret = -1;

	if (parse_cmdline(argc, argv))
		return EXIT_FAILURE;

	/* Open the media device and enumerate entities, pads and links. */
	if (media_opts.verbose)
		media = media_open_debug(
			media_opts.devname,
			(void (*)(void *, ...))fprintf, stdout);
	else
		media = media_open(media_opts.devname);
	if (media == NULL) {
		printf("Failed to open %s\n", media_opts.devname);
		goto out;
	}

	if (media_opts.print) {
		printf("Media controller API version %u.%u.%u\n\n",
		       (media->info.media_version << 16) & 0xff,
		       (media->info.media_version << 8) & 0xff,
		       (media->info.media_version << 0) & 0xff);
		printf("Media device information\n"
		       "------------------------\n"
		       "driver          %s\n"
		       "model           %s\n"
		       "serial          %s\n"
		       "bus info        %s\n"
		       "hw revision     0x%x\n"
		       "driver version  %u.%u.%u\n\n",
		       media->info.driver, media->info.model,
		       media->info.serial, media->info.bus_info,
		       media->info.hw_revision,
		       (media->info.driver_version << 16) & 0xff,
		       (media->info.driver_version << 8) & 0xff,
		       (media->info.driver_version << 0) & 0xff);
	}

	if (media_opts.entity) {
		struct media_entity *entity;

		entity = media_get_entity_by_name(media, media_opts.entity,
						  strlen(media_opts.entity));
		if (entity == NULL) {
			printf("Entity '%s' not found\n", media_opts.entity);
			goto out;
		}

		printf("%s\n", entity->devname);
	}

	if (media_opts.pad) {
		struct media_pad *pad;

		pad = media_parse_pad(media, media_opts.pad, NULL);
		if (pad == NULL) {
			printf("Pad '%s' not found\n", media_opts.pad);
			goto out;
		}

		v4l2_subdev_print_format(pad->entity, pad->index,
					 V4L2_SUBDEV_FORMAT_ACTIVE);
		printf("\n");
	}

	if (media_opts.print || media_opts.print_dot) {
		media_print_topology(media, media_opts.print_dot);
		printf("\n");
	}

	if (media_opts.reset) {
		if (media_opts.verbose)
			printf("Resetting all links to inactive\n");
		ret = media_reset_links(media);
		if (ret) {
			printf("Unable to reset links: %s (%d)\n",
			       strerror(-ret), -ret);
			goto out;
		}
	}

	if (media_opts.links) {
		ret = media_parse_setup_links(media, media_opts.links);
		if (ret) {
			printf("Unable to parse link: %s (%d)\n",
			       strerror(-ret), -ret);
			goto out;
		}
	}

	if (media_opts.formats) {
		ret = v4l2_subdev_parse_setup_formats(media,
						      media_opts.formats);
		if (ret) {
			printf("Unable to parse format: %s (%d)\n",
			       strerror(-ret), -ret);
			goto out;
		}
	}

	if (media_opts.interactive) {
		while (1) {
			char buffer[32];
			char *end;

			printf("Enter a link to modify or enter to stop\n");
			if (fgets(buffer, sizeof buffer, stdin) == NULL)
				break;

			if (buffer[0] == '\n')
				break;

			ret = media_parse_setup_link(media, buffer, &end);
			if (ret)
				printf("Unable to parse link: %s (%d)\n",
				       strerror(-ret), -ret);
		}
	}

	ret = 0;

out:
	if (media)
		media_close(media);

	return ret ? EXIT_FAILURE : EXIT_SUCCESS;
}

