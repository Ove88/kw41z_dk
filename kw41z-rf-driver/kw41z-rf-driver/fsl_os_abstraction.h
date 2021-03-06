/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file fsl_os_abstraction.h
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* o Redistributions of source code must retain the above copyright notice, this list
*   of conditions and the following disclaimer.
*
* o Redistributions in binary form must reproduce the above copyright notice, this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
*
* o Neither the name of Freescale Semiconductor, Inc. nor the names of its
*   contributors may be used to endorse or promote products derived from this
*   software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef _FSL_OS_ABSTRACTION_H_
#define _FSL_OS_ABSTRACTION_H_

#include "EmbeddedTypes.h"

#ifdef  __cplusplus
extern "C"
{
#endif

/*
 * @brief Enable all interrupts.
*/
void OSA_InterruptEnable(void);

/*!
 * @brief Disable all interrupts.
*/
void OSA_InterruptDisable(void);

/*!
 * @brief Installs the interrupt handler.
 *
 * @param IRQNumber IRQ number of the interrupt.
 * @param handler The interrupt handler to install.
 */
void OSA_InstallIntHandler(uint32_t IRQNumber, void (*handler)(void));

#ifdef  __cplusplus
}
#endif

#endif  
