#ifdef HAVE_CONFIG_H
#include <config.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>

#include "common.h"
#include "log.h"
#include "appconfig.h"
#include "procfile.h"
#include "rrd.h"
#include "plugin_proc.h"

#define SIZEOF_ARRAY(x) (sizeof((x)) / sizeof((x)[0]))

typedef struct tegra_stat {
	char filename[FILENAME_MAX + 1];
} TEGRA_STAT;

enum {
	GPU_LOAD,
	RAM_BW,
	THERMAL,
	CLOCK,
	CPU_FREQ,
};

static TEGRA_STAT values[] = {
	[GPU_LOAD] = { "/sys/devices/platform/host1x/gpu.0/load" },
	[RAM_BW] = { "/sys/kernel/debug/tegra_actmon/emc/avg_activity" },
	[THERMAL] = { "/sys/devices/virtual/thermal/thermal_zone" },
	[CLOCK] = { "/sys/kernel/debug/clock" },
	[CPU_FREQ] = { "/sys/devices/system/cpu/cpu" },
};

#define TEGRA_MAX_NAME 32

typedef struct tegra_temp {
	procfile *ff;
	char filename[FILENAME_MAX + 1];
	char type[TEGRA_MAX_NAME];
	int used;
} TEGRA_THERMAL;

#define TEGRA_MAX_THERMALS	9
static TEGRA_THERMAL thermals[TEGRA_MAX_THERMALS];

typedef struct tegra_clock {
	procfile *ff;
	char filename[FILENAME_MAX + 1];
	char name[TEGRA_MAX_NAME];
	int used;
} TEGRA_CLOCK;

static TEGRA_CLOCK clocks[] = {
	{ .ff = NULL, .filename = "cpu", .name = "cpu", .used = 1 },
	{ .ff = NULL, .filename = "gk20a.gbus", .name = "gpu", .used = 1 },  // K1
	{ .ff = NULL, .filename = "gm20b.gbus", .name = "gpu", .used = 1  }, // X1
	{ .ff = NULL, .filename = "avp.sclk", .name = "bpmp", .used = 1  },
};

typedef struct tegra_cpufreq {
	procfile *ff;
	char name[TEGRA_MAX_NAME];
	int used;
} TEGRA_CPUFREQ;

#define TEGRA_MAX_CPU 4
static TEGRA_CPUFREQ cpufreqs[TEGRA_MAX_CPU];

int do_sys_tegra(int update_every, unsigned long long dt) {
	static procfile *ff_gpu = NULL, *ff_ram = NULL;
	static procfile *ff_cpufreqs[TEGRA_MAX_CPU];
	static int first_run = 1;
	int i;

	if (first_run) {
		for (i = 0; i < SIZEOF_ARRAY(thermals); i++)
			thermals[i].used = 1;
		first_run = 0;
	}

	if(dt) {};

	// GPU Load
	if(!ff_gpu) {
		char filename[FILENAME_MAX + 1];
		snprintf(filename, FILENAME_MAX, "%s%s", global_host_prefix, values[GPU_LOAD].filename);
		ff_gpu = procfile_open(config_get("plugin:proc:tegra:gpu_load", "filename to monitor", filename), " \t,:|/", PROCFILE_FLAG_DEFAULT);
	}
	if(!ff_gpu) return 1;

	// Memory Bandwidth
	if(!ff_ram) {
		char filename[FILENAME_MAX + 1];
		snprintf(filename, FILENAME_MAX, "%s%s", global_host_prefix, values[RAM_BW].filename);
		ff_ram = procfile_open(config_get("plugin:proc:tegra:ram_bw", "filename to monitor", filename), " \t,:|/", PROCFILE_FLAG_NO_SEEK);
	}
	if(!ff_ram) return 1;

	// Temperatures
	for (i = 0; i < SIZEOF_ARRAY(thermals); i++) {
		if (thermals[i].used && !thermals[i].ff) {
			char filename[FILENAME_MAX + 1];
			struct stat st;

			snprintf(filename, FILENAME_MAX, "%s/%s%d/type", global_host_prefix, values[THERMAL].filename, i);
			if (stat(filename, &st) == -1 && errno == ENOENT) {
				thermals[i].used = 0;
				continue;
			}

			// read thermal zone name
			procfile *ff = procfile_open(filename, " \t,:|/", PROCFILE_FLAG_NO_SEEK);
			if(!ff) continue; // check for next thermal zone
			ff = procfile_readall(ff);
			if(!ff) continue; // check for next thermal zone
			strncpy(thermals[i].type, procfile_lineword(ff, 0, 0), TEGRA_MAX_NAME);
			procfile_close(ff);

			// read thermal zone name
			snprintf(thermals[i].filename, FILENAME_MAX, "%s/%s%d/temp", global_host_prefix, values[THERMAL].filename, i);
			thermals[i].ff = procfile_open(thermals[i].filename, " \t,:|/", PROCFILE_FLAG_DEFAULT);
			if (!thermals[i].ff) thermals[i].used = 0;
		}
	}

	// Clocks
	for (i = 0; i < SIZEOF_ARRAY(clocks); i++) {
		if (clocks[i].used && !clocks[i].ff) {
			char filename[FILENAME_MAX + 1];
			struct stat st;

			snprintf(filename, FILENAME_MAX, "%s/%s/%s/rate", global_host_prefix, values[CLOCK].filename, clocks[i].filename);
			if (stat(filename, &st) == -1 && errno == ENOENT) {
				clocks[i].used = 0;
				continue;
			}

			clocks[i].ff = procfile_open(filename, " \t,:|/", PROCFILE_FLAG_NO_SEEK);
			if (!clocks[i].ff) clocks[i].used = 0;
		}
	}

	// CPU frequencies
	for (i = 0; i < SIZEOF_ARRAY(cpufreqs); i++) {
		if (!cpufreqs[i].ff) {
			char filename[FILENAME_MAX + 1];
			snprintf(filename, FILENAME_MAX, "%s/%s%d/cpufreq/cpuinfo_cur_freq", global_host_prefix, values[CPU_FREQ].filename, i);
			snprintf(cpufreqs[i].name, TEGRA_MAX_NAME, "cpu%d", i);
			procfile *ff = procfile_open(filename, " \t,:|/", PROCFILE_FLAG_DEFAULT);
			if(!ff) continue;
			cpufreqs[i].ff = ff;
			cpufreqs[i].used = 1;
		}
	}

	RRDSET *st;

	// GPU Load
	unsigned long gpu_load = 0;
	ff_gpu = procfile_readall(ff_gpu);
	if(!ff_gpu) return 0; // we return 0, so that we will retry to open it next time
	gpu_load = strtoul(procfile_lineword(ff_gpu, 0, 0), NULL, 10);

	st = rrdset_find_byname("system.gpu");
	if(!st) {
		st = rrdset_create("system", "gpu", NULL, "gpu", NULL, "GPU Load", "percentage", 101, update_every, RRDSET_TYPE_AREA);
		rrddim_add(st, "load", NULL, 1, 10, RRDDIM_ABSOLUTE);
	}
	else rrdset_next(st);

	rrddim_set(st, "load", gpu_load);
	rrdset_done(st);

	// Memory Bandwidth
	unsigned long ram_bw = 0;
	ff_ram = procfile_readall(ff_ram);
	if(!ff_ram) return 0; // we return 0, so that we will retry to open it next time
	ram_bw = strtoul(procfile_lineword(ff_ram, 0, 0), NULL, 10);

	st = rrdset_find_byname("system.membw");
	if(!st) {
		st = rrdset_create("system", "membw", NULL, "bandwidth", NULL, "Memory Bandwidth", "percentage", 102, update_every, RRDSET_TYPE_AREA);
		rrddim_add(st, "bandwidth", NULL, 100, 1536000, RRDDIM_ABSOLUTE);
	}
	else rrdset_next(st);

	rrddim_set(st, "bandwidth", ram_bw);
	rrdset_done(st);

	// Temperatures
	st = rrdset_find_byname("system.thermal");
	if(!st) {
		st = rrdset_create("system", "thermal", NULL, "thermal", NULL, "Temperatures", "degree Celcius", 103, update_every, RRDSET_TYPE_LINE);
		for (i = 0; i < SIZEOF_ARRAY(thermals); i++) {
			if (thermals[i].used) rrddim_add(st, thermals[i].type, NULL, 1, 1000, RRDDIM_ABSOLUTE);
		}
	}
	else rrdset_next(st);
	for (i = 0; i < SIZEOF_ARRAY(thermals); i++) {
		if (thermals[i].used) {
			thermals[i].ff = procfile_readall(thermals[i].ff);
			if(!thermals[i].ff) {
				thermals[i].used = 0; // disable thermal zone
				continue;
			}
			unsigned long temperature = strtoul(procfile_lineword(thermals[i].ff, 0, 0), NULL, 10);
			rrddim_set(st, thermals[i].type, temperature);
		}
	}
	rrdset_done(st);

	// Clocks
	st = rrdset_find_byname("system.clocks");
	if(!st) {
		st = rrdset_create("system", "clocks", NULL, "clocks", NULL, "Clocks", "kHz", 2000, update_every, RRDSET_TYPE_LINE);
		for (i = 0; i < SIZEOF_ARRAY(clocks); i++) {
			if (clocks[i].used) rrddim_add(st, clocks[i].name, NULL, 1, 1000000, RRDDIM_ABSOLUTE);
		}
	}
	else rrdset_next(st);
	for (i = 0; i < SIZEOF_ARRAY(clocks); i++) {
		if (clocks[i].used) {
			clocks[i].ff = procfile_readall(clocks[i].ff);
			if(!clocks[i].ff) continue;
			unsigned long long clock = strtoull(procfile_lineword(clocks[i].ff, 0, 0), NULL, 10);
			rrddim_set(st, clocks[i].name, clock);
		}
	}
	rrdset_done(st);

	// CPU Frequencies
	st = rrdset_find_byname("cpu.frequency");
	if(!st) {
		st = rrdset_create("cpu", "frequency", NULL, "frequency", NULL, "Frequencies", "kHz", 2000, update_every, RRDSET_TYPE_LINE);
		for (i = 0; i < SIZEOF_ARRAY(cpufreqs); i++) {
			if (cpufreqs[i].used) rrddim_add(st, cpufreqs[i].name, NULL, 1, 1000, RRDDIM_ABSOLUTE);
		}
	}
	else rrdset_next(st);
	for (i = 0; i < SIZEOF_ARRAY(cpufreqs); i++) {
		if (cpufreqs[i].used) {
			cpufreqs[i].ff = procfile_readall(cpufreqs[i].ff);
			if(!cpufreqs[i].ff) continue;
			unsigned long long freq = strtoull(procfile_lineword(cpufreqs[i].ff, 0, 0), NULL, 10);
			rrddim_set(st, cpufreqs[i].name, freq);
		}
	}
	rrdset_done(st);


	return 0;
}
