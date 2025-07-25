/***************************************************************************
 *   Copyright (C) 2020 - 2025 by Federico Amedeo Izzo IU2NUO,             *
 *                                Niccolò Izzo IU2KIN,                     *
 *                                Silvano Seva IU2KWO                      *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/

#ifndef HWCONFIG_H
#define HWCONFIG_H

#include <stm32f4xx.h>
#include "pinmap.h"

#ifdef __cplusplus

// Export the HR_C6000 driver only for C++ sources
#include <HR_C6000.h>

extern HR_C6000 C6000;

extern "C" {
#endif

enum AdcChannel
{
    ADC_VOL_CH   = 0,
    ADC_VBAT_CH  = 1,
    ADC_MIC_CH   = 3,
};

extern const struct spiCustomDevice c6000_spi;
extern const struct spiDevice nvm_spi;
extern const struct Adc adc1;

/* Device has a working real time clock */
#define CONFIG_RTC

/* Device supports an optional GPS chip */
#define CONFIG_GPS

/* Screen dimensions */
#define CONFIG_SCREEN_WIDTH 160
#define CONFIG_SCREEN_HEIGHT 128

/* Screen pixel format */
#define CONFIG_PIX_FMT_RGB565

/* Battery type */
#define CONFIG_BAT_LIION
#define CONFIG_BAT_NCELLS 2

/* Device supports M17 mode */
#define CONFIG_M17

/*
 * To enable pwm for display backlight dimming uncomment this directive.
 *
 * WARNING: backlight pwm is disabled by default because it generates a
 * continuous tone in the speaker and headphones.
 *
 * This issue cannot be solved in any way because it derives from how the
 * MD-UV380 mcu pins are used: to have a noiseless backlight pwm, the control
 * pin has to be connected to a mcu pin having between its alternate functions
 * an output compare channel of one of the timers. With this configuration, the
 * pwm signal can completely generated in hardware and its frequency can be well
 * above 22kHz, which is the upper limit for human ears.
 *
 * In the MD-UV380 radio, display backlight is connected to PD8, which is not
 * connected to any of the available output compare channels. Thus, the pwm
 * signal generation is managed inside the TIM11 ISR by toggling the backlight
 * pin and its frequency has to be low (~250Hz) to not put too much overehad on
 * the processor due to timer ISR triggering at an high rate.
 *
 * #define CONFIG_SCREEN_BRIGHTNESS
 */

#ifdef __cplusplus
}
#endif

#endif /* HWCONFIG_H */
