// Copyright 2026 StableKite contributors
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

#ifndef RCL__TIMER_CALLBACK_MUTEX_H_
#define RCL__TIMER_CALLBACK_MUTEX_H_

#ifdef _WIN32
#include <windows.h>
typedef CRITICAL_SECTION rcl_timer_callback_mutex_t;

static int rcl_timer_callback_mutex_init(rcl_timer_callback_mutex_t * mutex)
{
  return InitializeCriticalSectionEx(mutex, 0, 0) ? 0 : -1;
}

static int rcl_timer_callback_mutex_lock(rcl_timer_callback_mutex_t * mutex)
{
  EnterCriticalSection(mutex);
  return 0;
}

static int rcl_timer_callback_mutex_unlock(rcl_timer_callback_mutex_t * mutex)
{
  LeaveCriticalSection(mutex);
  return 0;
}

static void rcl_timer_callback_mutex_fini(rcl_timer_callback_mutex_t * mutex)
{
  DeleteCriticalSection(mutex);
}
#else
#include <pthread.h>
typedef pthread_mutex_t rcl_timer_callback_mutex_t;

static int rcl_timer_callback_mutex_init(rcl_timer_callback_mutex_t * mutex)
{
  pthread_mutexattr_t attributes;
  int ret = pthread_mutexattr_init(&attributes);
  if (ret != 0) {
    return ret;
  }
  ret = pthread_mutexattr_settype(&attributes, PTHREAD_MUTEX_RECURSIVE);
  if (ret == 0) {
    ret = pthread_mutex_init(mutex, &attributes);
  }
  pthread_mutexattr_destroy(&attributes);
  return ret;
}

static int rcl_timer_callback_mutex_lock(rcl_timer_callback_mutex_t * mutex)
{
  return pthread_mutex_lock(mutex);
}

static int rcl_timer_callback_mutex_unlock(rcl_timer_callback_mutex_t * mutex)
{
  return pthread_mutex_unlock(mutex);
}

static void rcl_timer_callback_mutex_fini(rcl_timer_callback_mutex_t * mutex)
{
  pthread_mutex_destroy(mutex);
}
#endif

#endif  // RCL__TIMER_CALLBACK_MUTEX_H_
