#include <zephyr/kernel.h>
#include <zephyr/device.h>
#define DT_DRV_COMPAT ti_mspm0_flash_controller
#include <zephyr/drivers/flash.h>
#include <soc.h>
#include <zephyr/logging/log.h>
#include <driverlib/dl_flashctl.h>
#include "flash_mspm0.h"

struct flash_mspm0_config {
	FLASHCTL_Regs *regs;
};

struct flash_mspm0_data {
	struct k_sem flash_busy_sem;
};

/* Currently hardcoded for MSPM0G3507 family, will broaden in future */
#define MSPM0_BANK_COUNT		1
#define MSPM0_FLASH_SIZE		(FLASH_SIZE)
#define MSPM0_FLASH_PAGE_SIZE		(FLASH_PAGE_SIZE)
#define MSPM0_PAGES_PER_BANK		\
	((MSPM0_FLASH_SIZE / MSPM0_FLASH_PAGE_SIZE) / MSPM0_BANK_COUNT)

LOG_MODULE_REGISTER(flash_mspm0, CONFIG_FLASH_LOG_LEVEL);

// #define FLASH_TIMEOUT (2 * DT_PROP(DT_INST(0,mspm0_nv_flash), max_erase_time))

#define FLASH_TIMEOUT 16000000

// #if defined(CONFIG_MULTITHREADING)

// static inline void _flash_mspm0_sem_take(const struct *dev){
// 	k_sem_take(&FLASH_MSPM0_PRIV(dev->sem, K_FOREVER));
// 	//FIXME: Locking here
// }

// static inline void _flash_mspm0_give(const struct device *dev){
// 	//FIXME: unlocking here
// 	k_sem_give(&FLASH_MSPM0_PRIV(dev)->sem);
// }

// #define flash_mspm0_sem_init(dev) k_sem_init(&FLASH_MSPM0_PRIV(dev)->sem, 1, 1)
// #define flash_mspm0_sem_take(dev) _flash_mspm0_sem_take(dev)
// #define flash_mspm0_sem_give(dev) _flash_mspm0_sem_give(dev)
// #else

// #define flash_mspm0_sem_init(dev)
// #define flash_mspm0_sem_take(dev)
// #define flash_mspm0_sem_give(dev)
// #endif
static const struct flash_parameters flash_mspm0_parameters = {
	.write_block_size = FLASH_MSPM0_WRITE_BLOCK_SIZE,
	.erase_value = 0xff
};

static int flash_mspm0_init(const struct device *dev){
	struct flash_mspm0_data *flash_data = dev->data;
#if (CONFIG_FLASH_PAGE_LAYOUT)
	const struct flash_pages_layout *layout;
	size_t layout_size;

	flash_mspm0_page_layout(dev, &layout, &layout_size);
	for (size_t i = 0; i < layout_size; i++) {
		LOG_DBG("Block %zu: bs: %zu count: %zu", i,
			layout[i].pages_size, layout[i].pages_count);
	}
#endif
	k_sem_init(&flash_data->flash_busy_sem, 1, 1);
	return 0;
}

bool __weak flash_mspm0_valid_range(const struct device *dev, off_t offset, uint32_t len, bool write){
	if(write && !flash_mspm0_valid_write(offset, len)){
		return false;
	}
	//return true;
	return flash_mspm0_range_exists(dev, offset, len);
}


static int flash_mspm0_erase(const struct device *dev, off_t offset, size_t len){
	struct flash_mspm0_data *flash_data = dev->data;
	FLASHCTL_Regs *regs = FLASH_MSPM0_REGS(dev);
	int rc;
	bool status = false;

	/* currently hardcoded for sectors */
	if(len != 1024)
	{
		return -ENOTSUP;
	}

	if(!flash_mspm0_valid_range(dev, offset, len, true)){
		LOG_ERR("Erase range invalid. Offset %ld, len: %zu", (long int) offset, len);
		return -EINVAL;
	}

	k_sem_take(&flash_data->flash_busy_sem, K_FOREVER);

	// LOG_DBG("Erase offset: %ld, len: %zu", (long int) offset, len);
	DL_FlashCTL_unprotectSector(regs, offset, DL_FLASHCTL_REGION_SELECT_MAIN);
	/* blocking and should ultimately be done from RAM on single bank devices */
	DL_FlashCTL_eraseMemoryFromRAM(regs, offset, DL_FLASHCTL_COMMAND_SIZE_SECTOR);
	status = DL_FlashCTL_waitForCmdDone(regs);

	k_sem_give(&flash_data->flash_busy_sem);

	if(status == true){
		rc = 0; /* success */
	}
	else{
		rc = -EIO;
	}
	return rc;
	/*FIXME: only need unprotect, erase, and wait for completion. return 0 on success*/



	// if(rc == 0){
	//     rc = flash_mspm0_block_erase_loop(dev, offset, len);
	// }



	// if(!rc2){
	//     rc = rc2;
	// }

	// //FIXME: flash_mspm0_sem_give(dev);

	// return rc;
}

static int flash_mspm0_write(const struct device *dev, off_t offset, const void *data, size_t len){
	struct flash_mspm0_data *flash_data = dev->data;
	FLASHCTL_Regs *regs = FLASH_MSPM0_REGS(dev);
	bool status;
	int rc;

	if(len == 0 || (len % 8) != 0){
		return -EINVAL;
	}

	if(!flash_mspm0_valid_range(dev, offset, len, true)){
		LOG_ERR("Erase range invalid. Offset %ld, len: %zu", (long int) offset, len);
		return -EINVAL;
	}

	k_sem_take(&flash_data->flash_busy_sem, K_FOREVER);

	DL_FlashCTL_unprotectSector(regs, offset, DL_FLASHCTL_REGION_SELECT_MAIN);
	DL_FlashCTL_programMemoryFromRAM64WithECCGenerated(regs, offset, (uint32_t *) data);
	status = DL_FlashCTL_waitForCmdDone(regs);

	k_sem_give(&flash_data->flash_busy_sem);

	if(status == true){
		rc = 0; /* success */
	}
	else{
		rc = -EIO;
	}

	return rc;

}

static int flash_mspm0_read(const struct device *dev, off_t offset,
				void *data,
				size_t len)
{
	if(!len){
		return -EINVAL;
	}
	if(!flash_mspm0_valid_range(dev, offset, len, false)){
		//LOG_ERR("Read range invalid. Offset %ld, len %zu", (int) offset, len);
		return -EINVAL;
	}

	LOG_DBG("Read offset: %ld, len %zu", (long int) offset, len);
	memcpy(data, (uint8_t*) FLASH_MSPM0_BASE_ADDRESS + offset, len);
	return 0;
}

static const struct flash_parameters * flash_mspm0_get_parameters(const struct device *dev){
	ARG_UNUSED(dev);
	return &flash_mspm0_parameters;
}

void flash_mspm0_page_layout(const struct device *dev,
				 const struct flash_pages_layout **layout,
				 size_t *layout_size)
{
	static struct flash_pages_layout mspm0_flash_layout = {
		.pages_count = 128,
		.pages_size = 1024,
	};

	ARG_UNUSED(dev);

	if(mspm0_flash_layout.pages_count == 0){
		mspm0_flash_layout.pages_count = MSPM0_FLASH_SIZE / MSPM0_FLASH_PAGE_SIZE;
		mspm0_flash_layout.pages_size = MSPM0_FLASH_PAGE_SIZE;
	}

	*layout = &mspm0_flash_layout;
	*layout_size = 1;

}

static const struct flash_mspm0_config flash_mspm0_cfg = {
	.regs = (FLASHCTL_Regs *)DT_INST_REG_ADDR(0),
};

static const struct flash_driver_api flash_mspm0_driver_api = {
	.erase = flash_mspm0_erase,
	.write = flash_mspm0_write,
	.read = flash_mspm0_read,
	.get_parameters = flash_mspm0_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_mspm0_page_layout,
#endif
};

static struct flash_mspm0_data flash_mspm0_data;

DEVICE_DT_INST_DEFINE(0, flash_mspm0_init, NULL,
			&flash_mspm0_data, &flash_mspm0_cfg, POST_KERNEL,
			CONFIG_FLASH_INIT_PRIORITY, &flash_mspm0_driver_api);

// DEVICE_DT_INST_DEFINE(0, flash_mspm0_init, NULL,
// 		    &flash_mspm0_cfg, NULL, POST_KERNEL,
// 		    CONFIG_FLASH_INIT_PRIORITY, &flash_mspm0_driver_api);
