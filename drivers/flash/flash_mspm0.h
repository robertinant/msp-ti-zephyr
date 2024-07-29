#ifndef ZEPHYR_DRIVERS_FLASH_FLASH_MSPM0_H
#define ZEPHYR_DRIVERS_FLASH_FLASH_MSPM0_H_


#include <zephyr/drivers/flash.h>

#if DT_PROP(DT_INST(0, soc_nv_flash), write_block_size)
#define FLASH_MSPM0_WRITE_BLOCK_SIZE \
	DT_PROP(DT_INST(0, soc_nv_flash), write_block_size)
#else
#error Flash write block size not available
	/* Flash Write block size is extracted from device tree */
	/* as flash node property 'write-block-size' */
#endif


struct flash_mspm0_priv {
	FLASHCTL_Regs *regs;
	struct k_sem sem;
};

#if defined(CONFIG_MULTITHREADING)

static inline void _flash_mspm0_sem_take(const struct *dev){
	k_sem_take(&FLASH_MSPM0_PRIV(dev->sem, K_FOREVER));
	//FIXME: Locking here
}

static inline void _flash_mspm0_give(const struct device *dev){
	//FIXME: unlocking here
	k_sem_give(&FLASH_MSPM0_PRIV(dev)->sem);
}

#define flash_mspm0_sem_init(dev) k_sem_init(&FLASH_MSPM0_PRIV(dev)->sem, 1, 1)
#define flash_mspm0_sem_take(dev) _flash_mspm0_sem_take(dev)
#define flash_mspm0_sem_give(dev) _flash_mspm0_sem_give(dev)
#else

#define flash_mspm0_sem_init(dev)
#define flash_mspm0_sem_take(dev)
#define flash_mspm0_sem_give(dev)
#endif

#define FLASH_MSPM0_PRIV(dev) ((struct flash_mspm0_priv *)((dev)->data))

#define FLASH_MSPM0_REGS(dev) (FLASH_MSPM0_PRIV(dev)->regs)

// enum mspm0_ex_ops {
//     FLASH_MSPM0_EX_OP_SECTOR_WP = FLASH_EX_OP_VENDOR_BASE,

// 	FLASH_MSPM0_EX_OP_RDP,

// 	FLASH_MSPM0
// };


#ifdef CONFIG_FLASH_PAGE_LAYOUT
static inline bool flash_mspm0_range_exists(const struct device *dev,
					    off_t offset,
					    uint32_t len)
{
	struct flash_pages_info info;

	return !(flash_get_page_info_by_offs(dev, offset, &info) ||
		 flash_get_page_info_by_offs(dev, offset + len - 1, &info));
}
#endif	/* CONFIG_FLASH_PAGE_LAYOUT */



static inline bool flash_mspm0_valid_write(off_t offset, uint32_t len)
{
	return ((offset % FLASH_MSPM0_WRITE_BLOCK_SIZE == 0) &&
		(len % FLASH_MSPM0_WRITE_BLOCK_SIZE == 0U));
}

bool flash_mspm0_valid_range(const struct device *dev, off_t offset, uint32_t len, bool write);

int flash_mspm0_write_range(const struct device *dev, unsigned int offset, const void *data, unsigned int len);

int flash_mspm0_block_erase_loop(const struct device *dev, unsigned int offset, unsigned int len);

int flash_mspm0_wait_flash_idle(const struct device *dev);

static int flash_mspm0_check_status(const struct device *dev);

static int flash_mspm0_erase(const struct device *dev, uint32_t addr, DL_FLASHCTL_REGION_SELECT regionSelect);
int flash_mspm0_block_erase_loop(const struct device *dev,
				 unsigned int offset,
				 unsigned int len);
#endif /* ZEPHYR_DRIVERS_FLASH_FLASH_MSPM0_H_ */