/***************************************************************************/
#include <stdint.h>
#include "efm32.h"

/*******************************************************************************
 ******************************   DEFINES   ************************************
 ******************************************************************************/

#define EFM32_LFRCO_FREQ  (32768)

/*******************************************************************************
 **************************   LOCAL VARIABLES   ********************************
 ******************************************************************************/

/* System oscillator frequencies. These frequencies are normally constant */
/* for a target, but they are made configurable in order to allow run-time */
/* handling of different boards. The crystal oscillator clocks can be set */
/* compile time to a non-default value by defining respective EFM32_nFXO_FREQ */
/* values according to board design. By defining the EFM32_nFXO_FREQ to 0, */
/* one indicates that the oscillator is not present, in order to save some */
/* SW footprint. */

#ifndef EFM32_HFXO_FREQ
#define EFM32_HFXO_FREQ (32000000)
#endif
/* Do not define variable if HF crystal oscillator not present */
#if (EFM32_HFXO_FREQ > 0)

static uint32_t SystemHFXOClock = EFM32_HFXO_FREQ;
#endif

#ifndef EFM32_LFXO_FREQ 
#define EFM32_LFXO_FREQ (EFM32_LFRCO_FREQ)
#endif
/* Do not define variable if LF crystal oscillator not present */
#if (EFM32_LFXO_FREQ > 0)

static uint32_t SystemLFXOClock = 32768;
#endif


/*******************************************************************************
 **************************   GLOBAL VARIABLES   *******************************
 ******************************************************************************/

uint32_t SystemCoreClock;

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

/***************************************************************************/
uint32_t SystemCoreClockGet(void)
{
  uint32_t ret;
  
  ret = SystemHFClockGet();
  ret >>= (CMU->HFCORECLKDIV & _CMU_HFCORECLKDIV_HFCORECLKDIV_MASK) >> 
          _CMU_HFCORECLKDIV_HFCORECLKDIV_SHIFT;

  /* Keep CMSIS variable up-to-date just in case */
  SystemCoreClock = ret;

  return ret;
}


/***************************************************************************/
uint32_t SystemHFClockGet(void)
{
  uint32_t ret;
  
  switch (CMU->STATUS & (CMU_STATUS_HFRCOSEL | CMU_STATUS_HFXOSEL |
                         CMU_STATUS_LFRCOSEL | CMU_STATUS_LFXOSEL))
  {
    case CMU_STATUS_LFXOSEL:
#if (EFM32_LFXO_FREQ > 0)
      ret = SystemLFXOClock;
#else
      /* We should not get here, since core should not be clocked. May */
      /* be caused by a misconfiguration though. */
      ret = 0;
#endif
      break;
      
    case CMU_STATUS_LFRCOSEL:
      ret = EFM32_LFRCO_FREQ;
      break;
      
    case CMU_STATUS_HFXOSEL:
#if (EFM32_HFXO_FREQ > 0)
      ret = SystemHFXOClock;
#else
      /* We should not get here, since core should not be clocked. May */
      /* be caused by a misconfiguration though. */
      ret = 0;
#endif
      break;
      
    default: /* CMU_STATUS_HFRCOSEL */
      switch (CMU->HFRCOCTRL & _CMU_HFRCOCTRL_BAND_MASK)
      {
      case CMU_HFRCOCTRL_BAND_28MHZ:
        ret = 28000000;
        break;

      case CMU_HFRCOCTRL_BAND_21MHZ:
        ret = 21000000;
        break;

      case CMU_HFRCOCTRL_BAND_14MHZ:
        ret = 14000000;
        break;

      case CMU_HFRCOCTRL_BAND_11MHZ:
        ret = 11000000;
        break;

      case CMU_HFRCOCTRL_BAND_7MHZ:
        ret = 7000000;
        break;

      case CMU_HFRCOCTRL_BAND_1MHZ:
        ret = 1000000;
        break;

      default:
        ret = 0;
        break;
      }
      break;
  }

  return ret;
}


/**************************************************************************/
uint32_t SystemHFXOClockGet(void)
{
  /* External crystal oscillator present? */
#if (EFM32_HFXO_FREQ > 0)
  return SystemHFXOClock;
#else
  return 0;
#endif
}


/**************************************************************************/
void SystemHFXOClockSet(uint32_t freq)
{
  /* External crystal oscillator present? */
#if (EFM32_HFXO_FREQ > 0)
  SystemHFXOClock = freq;

  /* Update core clock frequency if HFXO is used to clock core */
  if (CMU->STATUS & CMU_STATUS_HFXOSEL)
  {
    /* The function will update the global variable */
    SystemCoreClockGet();
  }
#else
  (void)freq; /* Unused parameter */
#endif
}


/**************************************************************************/
void SystemInit(void)
{
}


/**************************************************************************/
uint32_t SystemLFRCOClockGet(void)
{
  /* Currently we assume that this frequency is properly tuned during */
  /* manufacturing and is not changed after reset. If future requirements */
  /* for re-tuning by user, we can add support for that. */
  return EFM32_LFRCO_FREQ;
}


/**************************************************************************/
uint32_t SystemLFXOClockGet(void)
{
  /* External crystal oscillator present? */
#if (EFM32_LFXO_FREQ > 0)
  return SystemLFXOClock;
#else
  return 0;
#endif
}


/**************************************************************************/
void SystemLFXOClockSet(uint32_t freq)
{
  /* External crystal oscillator present? */
#if (EFM32_LFXO_FREQ > 0)
  SystemLFXOClock = freq;

  /* Update core clock frequency if LFXO is used to clock core */
  if (CMU->STATUS & CMU_STATUS_LFXOSEL)
  {
    /* The function will update the global variable */
    SystemCoreClockGet();
  }
#else
  (void)freq; /* Unused parameter */
#endif
}
