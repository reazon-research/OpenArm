/*
 * Copyright (c) 2016-2021 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Name:    EventRecorderConf.h
 * Purpose: Event Recorder software component configuration options
 * Rev.:    V1.1.0
 */

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

// <h>Event Recorder

//   <o>Number of Records
//     <8=>8 <16=>16 <32=>32 <64=>64 <128=>128 <256=>256 <512=>512 <1024=>1024
//     <2048=>2048 <4096=>4096 <8192=>8192 <16384=>16384 <32768=>32768
//     <65536=>65536
//   <i>Configures size of Event Record Buffer (each record is 16 bytes)
//   <i>Must be 2^n (min=8, max=65536)
#define EVENT_RECORD_COUNT      64U

//   <o>Time Stamp Source
//      <0=> DWT Cycle Counter  <1=> SysTick  <2=> CMSIS-RTOS2 System Timer
//      <3=> User Timer (Normal Reset)  <4=> User Timer (Power-On Reset)
//   <i>Selects source for 32-bit time stamp
#define EVENT_TIMESTAMP_SOURCE  0

//   <o>Time Stamp Clock Frequency [Hz] <0-1000000000>
//   <i>Defines initial time stamp clock frequency (0 when not used)
#define EVENT_TIMESTAMP_FREQ    0U

// </h>

//------------- <<< end of configuration section >>> ---------------------------
