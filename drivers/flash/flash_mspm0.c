#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <soc.h>
#include <zephyr/loggig/log.h>
#include <driverlib/dl_flashctl.h>
#include "flash_mspm0.h"

#define DT_DRV_COMPAT ti_mspm0_flash_controller

/*Functions I think are probably good begin here*/
LOG_MODULE_REGISTER(flash_mspm0, CONFIG_FLASH_LOG_LEVEL); 

#define FLASH_TIMEOUT \
    (2 * DT_PROP(DT_INST(0,mspm0_nv_flash), max_erase_time))

static const struct flash_parameters flash_mspm0_parameters = {
    .write_block_size = FLASH_MSPM0_WRITE_BLOCK_SIZE,
    .erase_value = 0xff   
};

static int flash_mspm0_write_protection(const struct device *dev, bool enable);

bool __weak flash_mspm0_valid_range(const struct device *dev, off_t offset, uint32_t len, bool write){
    if(write && !flash_mspm0_valid_write(offset, len)){
        return false;
    }

    return flash_mspm0_range_exists(dev, offset, len);
}

/*Functions I think are probably good end here*/

static void flash_mspm0_flush_caches(const struct device *dev, off_t offset, size_t len){
	//NOTE: May need to include conditionals for different types of boards

	/*if data cache is enabled (and exists), disable cache. Reset it and then re-enable*/
    
	/*If instruction cache is enabled, disable cache. reset and then re-enable*/
	
}


static int flash_mspm0_erase(const struct device *dev, off_t offset, size_t len){
    // int rc;
    if(!flash_mspm0_valid_range(dev, offset, len, true)){
        LOG_ERR("Erase range invalid. Offset %ld, len: %zu", (long int) offset, len);
        return -EINVAL;
    }

    // if(!len){
    //     return 0;
    // }

    //FIXME: flash_mspm0_sem_take(dev);
    bool status = true;
    // LOG_DBG("Erase offset: %ld, len: %zu", (long int) offset, len);
    DL_FlashCTL_unprotectSector(FLASH_MSPM0_REGS, offset, DL_FLASHCTL_REGION_SELECT_MAIN);
    DL_FlashCTL_eraseMemory(FLASH_MSPM0_REGS, offset, DL_FLASHCTL_COMMAND_SIZE_SECTOR);
    
    status = DL_FlashCTL_waitForCmdDone(FLASH_MSPM0_REGS);

    if(!status){
        DL_FlashCTL_protectSector(FLASH_MSPM0_REGS, offset, DL_FLASHCTL_REGION_SELECT_MAIN);
        return -EINVAL;
    }
    else{
        DL_FlashCTL_protectSector(FLASH_MSPM0_REGS, offset, DL_FLASHCTL_REGION_SELECT_MAIN);
        return 0;
    }
    /*FIXME: only need unprotect, erase, and wait for completion. return 0 on success*/


    // rc = flash_mspm0_write_protection(dev, offset, len);
    // if(rc == 0){
    //     rc = flash_mspm0_block_erase_loop(dev, offset, len);
    // }

    // int rc2 = flash_mspm0_write_protection(dev, true);

    // if(!rc2){
    //     rc = rc2;
    // }

    // //FIXME: flash_mspm0_sem_give(dev);

    // return rc;
}


static int flash_mspm0_write_protection(const struct device *dev, bool enable)
{
	FLASH_TypeDef *regs = FLASH_MSPM0_REGS(dev);

	int rc = 0;

	if (enable) {
		rc = flash_stm32_wait_flash_idle(dev);
		if (rc) {
			flash_stm32_sem_give(dev);
			return rc;
		}
    }
/* FIXME: ST32 functionality below, unsure if there are equivalents for MSP*/
// #if defined(FLASH_SECURITY_NS)
// 	if (enable) {
// 		regs->NSCR |= FLASH_STM32_NSLOCK;
// 	} else {
// 		if (regs->NSCR & FLASH_STM32_NSLOCK) {
// 			regs->NSKEYR = FLASH_KEY1;
// 			regs->NSKEYR = FLASH_KEY2;
// 		}
// 	}
// #elif defined(FLASH_CR_LOCK)
// 	if (enable) {
// 		regs->CR |= FLASH_CR_LOCK;
// 	} else {
// 		if (regs->CR & FLASH_CR_LOCK) {
// 			regs->KEYR = FLASH_KEY1;
// 			regs->KEYR = FLASH_KEY2;
// 		}
// 	}
// #else
// 	if (enable) {
// 		regs->PECR |= FLASH_PECR_PRGLOCK;
// 		regs->PECR |= FLASH_PECR_PELOCK;
// 	} else {
// 		if (regs->PECR & FLASH_PECR_PRGLOCK) {
// 			LOG_DBG("Disabling write protection");
// 			regs->PEKEYR = FLASH_PEKEY1;
// 			regs->PEKEYR = FLASH_PEKEY2;
// 			regs->PRGKEYR = FLASH_PRGKEY1;
// 			regs->PRGKEYR = FLASH_PRGKEY2;
// 		}
// 		if (FLASH->PECR & FLASH_PECR_PRGLOCK) {
// 			LOG_ERR("Unlock failed");
// 			rc = -EIO;
// 		}
// 	}
// #endif /* FLASH_SECURITY_NS */

	if (enable) {
		LOG_DBG("Enable write protection");
	} else {
		LOG_DBG("Disable write protection");
	}

	return rc;
}

int flash_mspm0_wait_flash_idle(const struct device *dev){
    int64_t timeout_time = k_uptime_get() + FLASH_TIMEOUT;
    int rc;
    uint32_t busy_flags;

    rc = flash_mspm0_check_status(dev);
    if(rc < 0){
        return -EIO;
    }

    busy_flags = /*FIXME: Flash SR busy*/;

    while((FLASH_MSPM0_REGS(dev)->/*FIXME: Flash SR reg*/) & busy_flags){
        if(k_uptime_get() > timeout_time){
            LOG_ERR("Timeout! val: %d", FLASH_TIMEOUT);
            return -EIO;
        }
    }
    return 0;
}

static int flash_mspm0_check_status(const struct device *dev){
    if(FLASH_MSPM0_REGS(dev)->/*FIXME: security register*/ & /*FIXME: security errors*/){
        LOG_DBG("Status: 0x%08lx", FLASH_MSPM0_REGS(dev)->/*FIXME: security register*/ & /*FIXME: security errors*/);

        /*FIXME: clear errors in SR to unblock usage of the flash*/

        return -EIO
    }

    return 0;
}

int flash_mspm0_block_erase_loop(const struct device *dev,
				 unsigned int offset,
				 unsigned int len){
        
}



