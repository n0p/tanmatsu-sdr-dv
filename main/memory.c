// SPDX-License-Identifier: AGPL-3.0-only
// Frequency memories: 10 banks of 10, kept in a text file on the internal FAT partition (the one the
// launcher shows as /int). One memory per line:
//
//   <bank> <memory> <frequency in MHz> <mode>
//
// Lines starting with '#' and lines that do not parse are ignored when reading; the file is
// rewritten completely on every save, so comments added by hand do not survive one.

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include "app.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "wear_levelling.h"

static const char TAG[] = "memory";

#define MEM_MOUNT "/int"
#define MEM_PART  "locfd"
#define MEM_DIR   "/int/sdr-dv"
#define MEM_TMP   MEM_DIR "/memories.tmp"

typedef struct {
    uint32_t freq_hz;  // 0 = empty
    uint8_t  mode;
} mem_slot_t;

static mem_slot_t  mem[MEM_BANKS][MEM_SLOTS];
static wl_handle_t wl = WL_INVALID_HANDLE;
static bool        mounted;
static char        status[48] = "not loaded";

// The file system is only mounted while the file is read or written: its buffers come out of the
// internal RAM that the USB driver and the decoders need, and nothing is left open when the app
// is left through a restart.
static bool mem_mount(void) {
    if (mounted) return true;
    const esp_vfs_fat_mount_config_t cfg = {
        .format_if_mount_failed = false,  // the partition belongs to the launcher; never wipe it from here
        .max_files              = 2,
        .allocation_unit_size   = CONFIG_WL_SECTOR_SIZE,
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(MEM_MOUNT, MEM_PART, &cfg, &wl);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "mounting %s failed: %s", MEM_PART, esp_err_to_name(err));
        snprintf(status, sizeof(status), "no storage (%s)", esp_err_to_name(err));
        return false;
    }
    mounted = true;
    return true;
}

static void mem_unmount(void) {
    if (!mounted) return;
    esp_vfs_fat_spiflash_unmount_rw_wl(MEM_MOUNT, wl);
    wl      = WL_INVALID_HANDLE;
    mounted = false;
}

static bool mem_write_mounted(void) {
    struct stat st;
    if (stat(MEM_DIR, &st) != 0 && mkdir(MEM_DIR, 0775) != 0) {
        snprintf(status, sizeof(status), "mkdir failed (errno %d)", errno);
        return false;
    }
    FILE* f = fopen(MEM_TMP, "w");
    if (!f) {
        snprintf(status, sizeof(status), "cannot write (errno %d)", errno);
        return false;
    }
    bool ok = fprintf(f, "# Tanmatsu SDR memories: bank (0-9), memory (0-9), frequency in MHz, mode\n") > 0;
    for (int b = 0; b < MEM_BANKS; b++) {
        for (int s = 0; s < MEM_SLOTS; s++) {
            uint32_t hz = mem[b][s].freq_hz;
            if (hz == 0) continue;
            ok &= fprintf(f, "%d %d %u.%06u %s\n", b, s, (unsigned)(hz / 1000000), (unsigned)(hz % 1000000),
                          mode_names[mem[b][s].mode]) > 0;
        }
    }
    ok &= fclose(f) == 0;
    if (!ok) {
        snprintf(status, sizeof(status), "write failed (errno %d)", errno);
        unlink(MEM_TMP);
        return false;
    }
    // FAT has no atomic replace: the old file goes first.
    unlink(MEM_FILE);
    if (rename(MEM_TMP, MEM_FILE) != 0) {
        snprintf(status, sizeof(status), "rename failed (errno %d)", errno);
        return false;
    }
    snprintf(status, sizeof(status), "ok");
    return true;
}

static bool mem_write_file(void) {
    if (!mem_mount()) return false;
    bool ok = mem_write_mounted();
    mem_unmount();
    return ok;
}

static int mem_read_file(FILE* f) {
    char line[96];
    int  count = 0;
    while (fgets(line, sizeof(line), f)) {
        char* p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '#' || *p == '\r' || *p == '\n' || *p == 0) continue;
        // strtol / strtod rather than sscanf: the float conversions of scanf may be compiled out.
        char*  end;
        int    b = (int)strtol(p, &end, 10);
        if (end == p) continue;
        p     = end;
        int s = (int)strtol(p, &end, 10);
        if (end == p) continue;
        p          = end;
        double mhz = strtod(p, &end);
        if (end == p) continue;
        p = end;
        while (*p == ' ' || *p == '\t') p++;
        char name[12];
        int  len = 0;
        while (p[len] > ' ' && len < (int)sizeof(name) - 1) {
            name[len] = p[len];
            len++;
        }
        name[len]   = 0;
        int    mode = mode_from_name(name);
        double hz   = mhz * 1e6 + 0.5;
        if (b < 0 || b >= MEM_BANKS || s < 0 || s >= MEM_SLOTS || mode < 0) continue;
        if (hz < (double)FREQ_MIN_HZ || hz > (double)FREQ_MAX_HZ) continue;
        mem[b][s].freq_hz = (uint32_t)hz;
        mem[b][s].mode    = (uint8_t)mode;
        count++;
    }
    return count;
}

void memo_init(void) {
    memset(mem, 0, sizeof(mem));
    if (!mem_mount()) return;
    FILE* f = fopen(MEM_FILE, "r");
    if (f) {
        int n = mem_read_file(f);
        fclose(f);
        mem_unmount();
        snprintf(status, sizeof(status), "ok");
        ESP_LOGI(TAG, "%d memories read from %s", n, MEM_FILE);
        return;
    }
    mem_unmount();
    // First start: something to try the keys with. TETRA carriers and a DMR hotspot frequency.
    mem[0][1] = (mem_slot_t){391187500, MODE_TETRA};
    mem[0][2] = (mem_slot_t){391887500, MODE_TETRA};
    mem[0][3] = (mem_slot_t){391387500, MODE_TETRA};
    mem[0][4] = (mem_slot_t){430400000, MODE_DMR};
    if (mem_write_file()) ESP_LOGI(TAG, "created %s", MEM_FILE);
    else ESP_LOGW(TAG, "could not create %s: %s", MEM_FILE, status);
}

bool memo_get(int bank, int slot, uint32_t* freq_hz, demod_mode_t* mode) {
    if (bank < 0 || bank >= MEM_BANKS || slot < 0 || slot >= MEM_SLOTS) return false;
    if (mem[bank][slot].freq_hz == 0) return false;
    if (freq_hz) *freq_hz = mem[bank][slot].freq_hz;
    if (mode) *mode = (demod_mode_t)mem[bank][slot].mode;
    return true;
}

bool memo_set(int bank, int slot, uint32_t freq_hz, demod_mode_t mode) {
    if (bank < 0 || bank >= MEM_BANKS || slot < 0 || slot >= MEM_SLOTS) return false;
    mem[bank][slot].freq_hz = freq_hz;
    mem[bank][slot].mode    = (uint8_t)mode;
    return mem_write_file();
}

const char* memo_status(void) {
    return status;
}
