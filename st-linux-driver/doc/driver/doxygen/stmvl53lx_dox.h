/*
* Copyright (c) 2020, STMicroelectronics - All Rights Reserved
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permissivl53lx_mod_dbgon.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 *  @file stmvl53lx_dox.h - STM VL53LX doxygen documentation
 *
 *  @warning file use for doxygen doc generation do not use in source !
 */

/**
 * @mainpage
 * @section ctrl_data_paths	Control and data path
 * Dual @ref vl53lx_ioctl  and @ref sysfs_attrib control and data path co-exist
 * in the driver.
 * @subsection ctrl_paths Control path
 * Control over Sysfs or ioctl are assumed mutual exclusive. Control from ioctl
 * executes with no consideration of sysfs path and so on.
 * Thus only one control path shall be used at a time.
 *
 * @subsection data_paths Data paths
 * The two data path can be used safely in a concurrent way.
 *
 *
 * Input subsystem is fully buffered and support most linux i/o file operation
 * (poll,select ...).\n
 * Multiple events are fetch in non atomic way to collect a full range data.
 *
 * The EV_SYN bound events group of one range data. \n
 * Specific care and re-synch is needed to ensure proper data accumulation in
 * case of buffer overrun (SYN_DROPPED event)
 *
 * Like @ref sysfs_attrib the /dev/input/eventx is the device to read input
 * from and retrieved data.
 *
 * flush over sysfs is usable to control fifo draining.
 *
 * ioctl path is not buffered and can retrieve range data structure in single
 * call atomic way. It support blocking or non blocking data access.
 *
 * @sa sysfs_attrib vl53lx_ioctl
 *
 * @section known_issues known issues
 * @subsection ipp_abort Abort,stop during ipp call
 *  Device is  not locked "while ipp fly" between kernel and user daemon,
 *  so we accept and executed stop/abort and flush (android), and any other
 *  command in a say "no block" or reasonable "no wait" time while ranging.
 *
 *  This ensure if anything goes wrong in user daemon (slow, dies) driver
 *  is not maintained in a "dead locked" state for ever\n
 *  Even module unload would not be possible.
 *
 *  In case range is stopped/aborted while an ipp fly their's a rare but
 *  possible scenarios where a races with a new start and ranging can occur
 *  causing possibly a deadlock,bad-handling\n
 *
 *  @note This can be eliminated by putting some requirement constrains on
 *  back to back execution of stop re-start or implementing a wait queue for
 *  start to wait until flying ipp.
 *  Such "start" (blocking) would eliminated this bad situation.
 *
 *  @warning beware that is not handled rigth now !
 */

/**
 * select i2c/cci and h/w control  module interface used in driver
 *
 * CAMERA_CCI defined select msm cci interface MODULE\n
 * when not defined linux native i2c interface is used
 *
 * module interface see @a stmvl53lx_module_fn_t
 */
#define CAMERA_CCI


/** enable poll timing to be logged via dbgmsg
  * default to off when not defined set to 1 to enable
  */
#define STMVL53LX_LOG_POLL_TIMING	0

/** enable or not cci access timing to be logged dbgmsg
 * default to off when not defined set to 1 to enable
 */
#define STMVL53LX_LOG_CCI_TIMING	0

/**
 * configure usage of regulator to enable/disable sensor power
 *
 * @note the regulator information come from device tree
 * see stmvl53lx_module-i2c or stmvl53lx_module-i2c file
 */
#define CFG_STMVL53LX_HAVE_REGULATOR


/**
 * @defgroup vl53lx_mod_dbg  debugging
 *
 */

/**
 * @defgroup vl53lx_ioctl  IOCTL interface commands
 *
 * @brief Ioctl commands for vl53Lx
 *
 * ioctl are done across misc device @ref VL53LX_MISC_DEV_NAME
 */


/**
 *@defgroup sysfs_attrib sysfs attribute
 *
 * stmvl53lx can be controled via a set of exposed as sysf attribute
 * under /sys/class/inputx/attribe_name where x is the input device number
 * associated with the vl53lx device .
 * X is a dynamic number that depend on input device present in the system
 * it can changes, depending  order of driver loaded, hot plug device
 * like usb mice and keyboard.
 */

/**@defgroup drv_port vl53lx interface module porting and customization
 *
 * this i2c + gpio interface module is use  when @ref CAMERA_CCI isn't set
 */


#error "This file must not be included in source used for doc purpose !!!"
