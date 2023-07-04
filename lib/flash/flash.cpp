#include <cstdio>
#include "flash.h"



// We're going to erase and reprogram a region 256k from the start of flash.
// Once done, we can access this at XIP_BASE + 2000k.
#define FLASH_TARGET_OFFSET (2000 * 1024)

const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);

int update_flash_data() {
    uint32_t status;
    uint8_t flash_databuffer[FLASH_PAGE_SIZE];

    axis_Union.axis_fval = servo_current_limit;
    for(int i = 0; i<4; i++){
        flash_databuffer[0+i] = axis_Union.axis_val[i];
    }
    // printf(" %f servo_current_limit\n", axis_Union.axis_fval);
    
    axis_Union.axis_fval = servo_angle;
    for(int i = 0; i<4; i++){
        flash_databuffer[4+i] = axis_Union.axis_val[i];
    }
    // printf(" %f servo_angle\n", axis_Union.axis_fval);

    // Note that a whole number of sectors must be erased at a time.
    printf("Erasing and writing new values...\n");
    busy_wait_ms(50);
    status = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_PAGE_SIZE);
    busy_wait_ms(50);
    flash_range_program(FLASH_TARGET_OFFSET, flash_databuffer, FLASH_PAGE_SIZE);
    busy_wait_ms(50);
    restore_interrupts(status);

    bool mismatch = false;
    for (int i = 0; i < 16; ++i) {
        if (flash_databuffer[i] != flash_target_contents[i])
            mismatch = true;
    }
    if (mismatch)
        printf("Programming failed!\n");
    else
        printf("Programming successful!\n");
    
    return 0;
}