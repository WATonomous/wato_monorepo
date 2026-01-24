// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file vehicles.h
 * @brief List of vehicle headers.
 *
 */

#ifndef _OSCC_VEHICLES_H_
#define _OSCC_VEHICLES_H_

#if defined(KIA_SOUL)
  #include "vehicles/kia_soul_petrol.h"
#elif defined(KIA_SOUL_EV)
  #include "vehicles/kia_soul_ev.h"
#elif defined(KIA_NIRO)
  #include "vehicles/kia_niro.h"
#endif

#define CONSTRAIN(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#endif /* _OSCC_VEHICLES_H_ */
