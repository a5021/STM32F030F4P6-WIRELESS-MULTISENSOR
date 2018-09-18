#ifndef __FLASH_INCLUDED
#define __FLASH_INCLUDED

#define FLASH_PAGE_SIZE               ((uint32_t)0x00000400)   /* FLASH Page Size */
#define FLASH_DATA_ADDR               ((uint32_t)0x08007C00)   /* Flash area used */

#define FLASH_ERROR_ERASE             ((uint16_t)0x01)
#define FLASH_ERROR_PROG              ((uint16_t)0x02)
#define FLASH_ERROR_PROG_FLAG         ((uint16_t)0x04)
#define FLASH_ERROR_WRITE_PROT        ((uint16_t)0x08)
#define FLASH_ERROR_UNKNOWN           ((uint16_t)0x10)

extern uint16_t flash_status;

__STATIC_INLINE bool write2flash(uint16_t src[], uint8_t len) {
  __IO uint16_t* p = (__IO uint16_t*) FLASH_DATA_ADDR;

  /***************************************
  *
  *           UNLOCK FLASH
  *
  ***************************************/
  
  while ((FLASH->SR & FLASH_SR_BSY) != 0);           // Wait till no operation is on going
  
  if ((FLASH->CR & FLASH_CR_LOCK) != 0) {            // Check the Flash is locked
    FLASH->KEYR = FLASH_KEY1;                        // Perform unlock sequence 
    FLASH->KEYR = FLASH_KEY2;
  }

  /**************************************
  *
  *          ERASE DATA AREA
  *
  **************************************/
  
  FLASH->CR |= FLASH_CR_PER;                         // enable page erasing mode
  FLASH->AR = FLASH_DATA_ADDR;                       // set page address
  FLASH->CR |= FLASH_CR_STRT;                        // start the erasing
  while ((FLASH->SR & FLASH_SR_BSY) != 0);           // Wait till page be erased
  FLASH->CR &= ~FLASH_CR_PER;                        // reset page erasing flag
  
  if ((FLASH->SR & FLASH_SR_EOP) != 0) {             // Check for End of Operation flag
    FLASH->SR |= FLASH_SR_EOP;                       // Reset flag
  } else {
    if ((FLASH->SR & FLASH_SR_WRPERR) != 0) {        // Check Write protection error 
      flash_status |= FLASH_ERROR_WRITE_PROT;        // Report the error to the main progran
      FLASH->SR |= FLASH_SR_WRPERR;                  // Clear the error flag
    } else {
      flash_status |= FLASH_ERROR_UNKNOWN;           // Report the error
    }
    return false;                                    // break flashing
  }

  /***************************************
  *
  *     CHECK THE DATA AREA CLEARED
  *
  ***************************************/
  
  for (uint32_t i = FLASH_PAGE_SIZE; i > 0; i -= 4) { // Check the erasing of the page by reading all the page value
    if (*(uint32_t *)(FLASH_DATA_ADDR + i - 4) != (uint32_t)0xFFFFFFFF) { // compare with erased value, all bits at 1
      flash_status |= FLASH_ERROR_ERASE;             // report the error to the main progran
      return false;                                  // break flashing
    }
  }

  /***************************************
  *
  *         WRITE DATA TO FLASH
  *
  ***************************************/
  
  FLASH->CR |= FLASH_CR_PG;                          // enable programming mode
  for (uint8_t i = 0; i < len; i++) {                // start writing cycle
    *p = src[i];                                     // write 16 bits data
    while ((FLASH->SR & FLASH_SR_BSY) != 0);         // wait till data to be written
    if ((FLASH->SR & FLASH_SR_EOP) != 0) {           // Check the EOP flag
      FLASH->SR |= FLASH_SR_EOP;                     // Clear it
    } else {
      if ((FLASH->SR & FLASH_SR_PGERR) != 0) {       // Check Programming error
        flash_status = FLASH_ERROR_PROG_FLAG;        // save error code
        FLASH->SR |= FLASH_SR_PGERR;                 // Clear programming error flag
      } else if ((FLASH->SR & FLASH_SR_WRPERR) != 0) { // Check write protection
        flash_status = FLASH_ERROR_WRITE_PROT;       // save error code
        FLASH->SR |= FLASH_SR_WRPERR;                // Clear write protection error flag
      } else {
        flash_status = FLASH_ERROR_UNKNOWN;          // save error code
      }
      FLASH->CR &= ~FLASH_CR_PG;                     // disable programming      
      return false;                                  // break flashing
    }
    p++;                                             // next address to write data
  }
  
  FLASH->CR &= ~FLASH_CR_PG;                         // disable programming
  return true;
}

#endif
