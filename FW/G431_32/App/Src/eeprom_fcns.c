#include "eeprom_fcns.h"
#include "eeprom_emul.h"
#include "defines.h"
#include <stdint.h>

void eeprom_unlock()
{
    HAL_FLASH_Unlock();
    return;
}

uint8_t eeprom_erase()
{
    uint8_t retVal = 1;

    EE_Status eeprom_status = EE_Init(EE_FORCED_ERASE);
    if (eeprom_status != EE_OK)
    {
    	retVal = 0;
    }

    //write default values to NVM variables
    if(retVal)
    {
        eeprom_status = EE_WriteVariable32bits(ELLIPSE_CAL_T00_ADDR, ELLIPSE_CAL_T00_DEFAULT);
        if(EE_OK != eeprom_status  && EE_CLEANUP_REQUIRED != eeprom_status)
        {
          retVal = 0;
        }
    }
    
    if(retVal)
    {
        eeprom_status = EE_WriteVariable32bits(ELLIPSE_CAL_T01_ADDR, ELLIPSE_CAL_T01_DEFAULT);
        if(EE_OK != eeprom_status  && EE_CLEANUP_REQUIRED != eeprom_status)
        {
          retVal = 0;
        }
    }

    if(retVal)
    {
        eeprom_status = EE_WriteVariable32bits(ELLIPSE_CAL_T10_ADDR, ELLIPSE_CAL_T10_DEFAULT);
        if(EE_OK != eeprom_status  && EE_CLEANUP_REQUIRED != eeprom_status)
        {
          retVal = 0;
        }
    }

    if(retVal)
    {
        eeprom_status = EE_WriteVariable32bits(ELLIPSE_CAL_T11_ADDR, ELLIPSE_CAL_T11_DEFAULT);
        if(EE_OK != eeprom_status  && EE_CLEANUP_REQUIRED != eeprom_status)
        {
          retVal = 0;
        }
    }

    if(retVal)
    {
        eeprom_status = EE_WriteVariable32bits(ELLIPSE_CAL_X0_ADDR, ELLIPSE_CAL_X0_DEFAULT);
        if(EE_OK != eeprom_status  && EE_CLEANUP_REQUIRED != eeprom_status)
        {
          retVal = 0;
        }
    }

    if(retVal)
    {
        eeprom_status = EE_WriteVariable32bits(ELLIPSE_CAL_Y0_ADDR, ELLIPSE_CAL_Y0_DEFAULT);
        if(EE_OK != eeprom_status  && EE_CLEANUP_REQUIRED != eeprom_status)
        {
          retVal = 0;
        }
    }

    if(retVal)
    {
        if(EE_CLEANUP_REQUIRED == eeprom_status)
        {
            eeprom_status = EE_CleanUp();
            if(EE_OK != eeprom_status)
            {
                retVal = 0;
            }
        }
    }
    
    return retVal;
}

uint8_t eeprom_read_params(uint32_t* _eeprom_params, uint8_t _n_params, uint16_t _start_addr)
{
    uint8_t ii;
    uint8_t retVal = 1;

    //init eeprom
    EE_Status eeprom_status = EE_Init(EE_CONDITIONAL_ERASE);
    if (eeprom_status != EE_OK)
    {
    	retVal = 0;
    }

    for(ii = 0; ii < _n_params; ii++)
    {
        if(retVal)
        {
            eeprom_status = EE_ReadVariable32bits(_start_addr + ii, &_eeprom_params[ii]);
            if(EE_OK != eeprom_status)
            {
                retVal = 0;
            }
        }
    }

    return retVal;
}

uint8_t eeprom_write_params(uint32_t* _eeprom_params, uint8_t _n_params, uint16_t _start_addr)
{
    uint8_t retVal = 1;
    uint8_t ii;
    EE_Status eeprom_status;
    
    //store coefficients inside NVM parameters
    for(ii = 0; ii < _n_params; ii++)
    {
        if(retVal)
        {
            eeprom_status = EE_WriteVariable32bits(_start_addr + ii, _eeprom_params[ii]);
            if(EE_OK != eeprom_status  && EE_CLEANUP_REQUIRED != eeprom_status)
            {
                retVal = 0;
            }
        }
    }
    
    if(retVal && EE_CLEANUP_REQUIRED == eeprom_status)
    {
        eeprom_status = EE_CleanUp();
        if(EE_OK != eeprom_status)
        {
            retVal = 0;
        }
    }

    return retVal;
}