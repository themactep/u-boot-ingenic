/*
 *SD update support
 */

#include <common.h>
#include <environment.h>
#include <command.h>
#include <malloc.h>
#include <image.h>
#include <asm/byteorder.h>
#include <asm/io.h>
#include <spi_flash.h>
#include <linux/mtd/mtd.h>
#include <fat.h>

#ifdef CONFIG_AUTO_UPDATE  /* cover the whole file */

#ifdef CONFIG_AUTO_SD_UPDATE
#ifndef CONFIG_MMC
#error "should have defined CONFIG_MMC"
#endif
#include <mmc.h>
#endif

#undef AU_DEBUG
#undef debug
#define AU_DEBUG

#ifdef	AU_DEBUG
#define debug(fmt, args...)	printf(fmt, ##args)
#else
#define debug(fmt, args...)
#endif	/* AU_DEBUG */

/* possible names of files on the medium. */
#define AU_BOOT	"autoupdate-uboot.bin"
#define AU_FULL	"autoupdate-full.bin"

struct flash_layout {
	long start;
	long end;
};
static struct spi_flash *flash;

struct medium_interface {
	char name[20];
	int (*init) (void);
	void (*exit) (void);
};

/* layout of the FLASH. ST = start address, ND = end address. */
#define AU_FL_BOOT_ST	0x0
#define AU_FL_BOOT_ND	0x40000
#define AU_FL_FULL_ST	0x0
#define AU_FL_FULL_ND	0x1000000

// Not used
//static int au_stor_curr_dev; /* current device */

/* index of each file in the following arrays */
#define IDX_BOOT	0
#define IDX_FULL	1

/* max. number of files which could interest us */
#define AU_MAXFILES	2

/* pointers to file names */
char *aufile[AU_MAXFILES] = {
	AU_BOOT,
	AU_FULL
};

/* sizes of flash areas for each file */
long ausize[AU_MAXFILES] = {
	AU_FL_BOOT_ND - AU_FL_BOOT_ST,
	AU_FL_FULL_ND - AU_FL_FULL_ST
};

/* array of flash areas start and end addresses */
struct flash_layout aufl_layout[AU_MAXFILES] = {
	{ AU_FL_BOOT_ST, AU_FL_BOOT_ND },
	{ AU_FL_FULL_ST, AU_FL_FULL_ND },
};

/* where to load files into memory */
#define LOAD_ADDR ((unsigned char *)0x80600000)

/* the app is the largest image */
#define MAX_LOADSZ ausize[IDX_FULL]

int LOAD_ID = -1; //default update all

int autoupdate_status = -1;

static int au_check_cksum_valid(int idx, long nbytes)
{
	image_header_t *hdr;
	unsigned long checksum;

	hdr = (image_header_t *)LOAD_ADDR;

	if (nbytes != (sizeof(*hdr) + ntohl(hdr->ih_size))) {
		printf("SDU:   Image %s bad total SIZE\n", aufile[idx]);
		return -1;
	}
	/* check the data CRC */
	checksum = ntohl(hdr->ih_dcrc);

	if (crc32(0, (unsigned char const *)(LOAD_ADDR + sizeof(*hdr)),
			ntohl(hdr->ih_size)) != checksum) {
		printf("SDU:   Image %s bad data checksum\n", aufile[idx]);
		return -1;
	}

	return 0;
}

static int au_check_header_valid(int idx, long nbytes)
{
	image_header_t *hdr;
	unsigned long checksum;

	char env[20];
	char auversion[20];

	hdr = (image_header_t *)LOAD_ADDR;
	/* check the easy ones first */
#if 0
	#define CHECK_VALID_DEBUG
#else
	#undef CHECK_VALID_DEBUG
#endif

#ifdef CHECK_VALID_DEBUG
	printf("magic %#x %#x\n",  ntohl(hdr->ih_magic), IH_MAGIC);
	printf("arch  %#x %#x\n",  hdr->ih_arch, IH_ARCH_MIPS);
	printf("size  %#x %#lx\n", ntohl(hdr->ih_size), nbytes);
	printf("type  %#x %#x\n",  hdr->ih_type, IH_TYPE_FIRMWARE);
#endif
	if (nbytes < sizeof(*hdr)) {
		printf("SDU:   Image %s bad header SIZE\n", aufile[idx]);
		return -1;
	}
	if (ntohl(hdr->ih_magic) != IH_MAGIC || hdr->ih_arch != IH_ARCH_MIPS) {
		printf("SDU:   Image %s bad MAGIC or ARCH\n", aufile[idx]);
		return -1;
	}
	/* check the hdr CRC */
	checksum = ntohl(hdr->ih_hcrc);
	hdr->ih_hcrc = 0;

	if (crc32(0, (unsigned char const *)hdr, sizeof(*hdr)) != checksum) {
		printf("SDU:   Image %s bad header checksum\n", aufile[idx]);
		return -1;
	}
	hdr->ih_hcrc = htonl(checksum);
	/* check the type - could do this all in one gigantic if() */
	if ((idx == IDX_BOOT) && (hdr->ih_type != IH_TYPE_FIRMWARE)) {
		printf("SDU:   Image %s wrong type\n", aufile[idx]);
		return -1;
	}

	if ((idx == IDX_FULL) && (hdr->ih_type != IH_TYPE_FIRMWARE)) {
		printf("SDU:   Image %s wrong type\n", aufile[idx]);
		return -1;
	}
	/* recycle checksum */
	checksum = ntohl(hdr->ih_size);

	/* check the size does not exceed space in flash. HUSH scripts */
	/* all have ausize[] set to 0 */
	if ((ausize[idx] != 0) && (ausize[idx] < checksum)) {
		printf("SDU:   Image %s is bigger than FLASH\n", aufile[idx]);
		return -1;
	}

	sprintf(env, "%lx", (unsigned long)ntohl(hdr->ih_time));
	/*setenv(auversion, env);*/

	return 0;
}

static int au_do_update(int idx, long file_size)
{
	debug("SDU:   au_do_update(%d, %lu)\n", idx, file_size);

	unsigned long start;
	unsigned long partition_size = ausize[idx];

	int rc;
	void *buf;
	char *pbuf;

	switch (idx) {
		case IDX_BOOT:
			printf("SDU:   Found U-boot image to flash!\n");
			start = AU_FL_BOOT_ST;
			break;
		case IDX_FULL:
			printf("SDU:   Found full firmware image to flash!\n");
			start = AU_FL_FULL_ST;
			break;
	}

	/* Erase the address range. */
	printf("SDU:   Erasing flash from address 0x%lx to 0x%lx (length: 0x%lx)\n", start, start + partition_size, partition_size);
	rc = flash->erase(flash, start, partition_size);
	if (rc) {
		printf("SDU:   SPI flash sector erase failed\n");
		return 1;
	}

	buf = map_physmem((unsigned long)LOAD_ADDR, partition_size, MAP_WRBACK);
	if (!buf) {
		puts("SDU:   Failed to map physical memory\n");
		return 1;
	}

	pbuf = buf; // use the buffer directly

	/* Copy the data from RAM to FLASH */
	printf("SDU:   flash write...\n");
	rc = flash->write(flash, start, file_size, pbuf);
	if (rc) {
		printf("SDU:   SPI flash write failed, return %d\n", rc);
		return 1;
	}

	unmap_physmem(buf, partition_size);

	return 0;
}

/*
 * If none of the update file(u-boot, full) was found
 * in the medium, return -1;
 * If u-boot has been updated, return 1;
 * Others, return 0;
 */
static int update_to_flash(void)
{
	int i = 0;
	long sz;
	int res;
	int boot_updated = 0;
	int full_updated = 0;
	int image_found = 0;
	int j;

	// Define which files to automatically update
	int auto_update_files[] = {IDX_BOOT, IDX_FULL};
	int num_auto_updates = sizeof(auto_update_files) / sizeof(auto_update_files[0]);

	for (i = 0; i < num_auto_updates; i++) {
		sz = file_fat_read(aufile[i], LOAD_ADDR, (i != IDX_FULL) ? MAX_LOADSZ : flash->size);
		if (sz <= 0) {
			debug("SDU:   %s not found\n", aufile[i]);
			continue;
		}

		char empty_flag[1] = {0};
		switch (i) {
			case IDX_BOOT:
				if (file_fat_read("autoupdate-uboot.done", LOAD_ADDR, 1) >= 0) {
					printf("SDU:   Flag file autoupdate-uboot.done exists, skipping %s\n", AU_BOOT);
					continue;
				}

				res = au_do_update(i, sz);
				if (res != 0) {
					return res;
				}

				if (i == IDX_BOOT) {
					// Write the autoupdate-uboot.done file after successful flash
					if (file_fat_write("autoupdate-uboot.done", empty_flag, sizeof(empty_flag)) < 0) {
						printf("SDU:   Error creating flag file autoupdate-uboot.done\n");
						return -1;
					} else {
						printf("SDU:   Flag file autoupdate-uboot.done created\n");
						return 0;
					}
				}
				break;

			case IDX_FULL:
				if (file_fat_read("autoupdate-full.done", LOAD_ADDR, 1) >= 0) {
					printf("SDU:   Flag file autoupdate-full.done exists, skipping %s\n", AU_FULL);
					continue;
				}

				res = au_do_update(i, sz);
				if (res != 0) {
					return res;
				}

				if (i == IDX_FULL) {
					// Write the autoupdate-full.done file after successful flash
					if (file_fat_write("autoupdate-full.done", empty_flag, sizeof(empty_flag)) < 0) {
						printf("SDU:   Error creating flag file autoupdate-full.done\n");
						return -1;
					} else {
						printf("SDU:   Flag file autoupdate-full.done created\n");
						return 1;
					}
				}
				break;
		}
	}

	return -1;
}

/*
 * This is called by board_init() after the hardware has been set up
 * and is usable. Only if SPI flash initialization failed will this function
 * return -1, otherwise it will return 0;
 */
int do_auto_update(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	block_dev_desc_t *storage_device;
	int old_ctrlc;
	int state = -1;
	long start = -1;
	long end = 0;

	// Probe the SPI flash and ensure it initializes correctly
	flash = spi_flash_probe(0, 0, 1000000, 0x3);
	if (!flash) {
		printf("SDU:   Failed to initialize SPI flash\n");
		return -1;
	}

	printf("SDU:   Checking for autoupdate files...\n");

	if (argc == 3 || argc > 4) {
		return CMD_RET_USAGE;
	}

	if (argc == 2 || argc == 4) {
		LOAD_ID = simple_strtoul(argv[1], NULL, 16);
		if (LOAD_ID < 0 || LOAD_ID > AU_MAXFILES) {
			printf("SDU:   Unsupported ID!\n");
			return CMD_RET_USAGE;
		}
	}

	if (argc == 4) {
		start = simple_strtoul(argv[2], NULL, 16);
		end   = simple_strtoul(argv[3], NULL, 16);
		if (start >= 0 && end && end > start) {
			ausize[LOAD_ID] = end - start;
			aufl_layout[LOAD_ID].start = start;
			aufl_layout[LOAD_ID].end   = end;
		} else {
			printf("SDU:   Wrong address, using defaults!\n");
		}
	}

	if (argc == 1) {
		// Default behavior
	}

	debug("SDU:   Device name: mmc\n");
	storage_device = get_dev("mmc", 0);
	if (!storage_device) {
		debug("SDU:   Unknown device type!\n");
		return 0;
	}

	if (fat_register_device(storage_device, 1) != 0) {
		debug("SDU:   Unable to use mmc 1:1 for fatls\n");
		return -1;
	}

/* As with cmd_sdstart.c, lets disable this for now, if we are running, assume the FS is valid
	if (file_fat_detectfs() != 0) {
		debug("file_fat_detectfs failed\n");
		return -1;
	}
*/
	/*
	 * make sure that we see CTRL-C
	 * and save the old state
	 */
	old_ctrlc = disable_ctrlc(0);

	state = update_to_flash();

	/* restore the old state */
	disable_ctrlc(old_ctrlc);

	LOAD_ID = -1;

	if (state == 1) {
		printf("SDU:   Auto-update completed successfully. Saving environment...\n");
		saveenv();
		printf("SDU:   Environment saved.\n");
	}

	return (state == -1) ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	sdupdate,	9,	1,	do_auto_update,
	"auto upgrade files from mmc to flash",
	"\n"
);
#endif /* CONFIG_AUTO_UPDATE */
