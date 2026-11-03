// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include "rcl/timer.h"

#include "rcl/rcl.h"

#include "osrf_testing_tools_cpp/scope_exit.hpp"
#include "rcl/error_handling.h"

#include "./allocator_testing_utils.h"

class TestTimerFixture : public ::testing::Test
{
public:
  rcl_context_t * context_ptr;
  rcl_node_t * node_ptr;
  void SetUp()
  {
    rcl_ret_t ret;
    {
      rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
      ret = rcl_init_options_init(&init_options, rcl_get_default_allocator());
      ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
      OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
      {
        EXPECT_EQ(RCL_RET_OK, rcl_init_options_fini(&init_options)) << rcl_get_error_string().str;
      });
      this->context_ptr = new rcl_context_t;
      *this->context_ptr = rcl_get_zero_initialized_context();
      ret = rcl_init(0, nullptr, &init_options, this->context_ptr);
      ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    }
    this->node_ptr = new rcl_node_t;
    *this->node_ptr = rcl_get_zero_initialized_node();
    const char * name = "test_timer_node";
    rcl_node_options_t node_options = rcl_node_get_default_options();
    ret = rcl_node_init(this->node_ptr, name, "", this->context_ptr, &node_options);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  }

  void TearDown()
  {
    rcl_ret_t ret = rcl_node_fini(this->node_ptr);
    delete this->node_ptr;
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ret = rcl_shutdown(this->context_ptr);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ret = rcl_context_fini(this->context_ptr);
    delete this->context_ptr;
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  }
};

static uint8_t times_called = 0;
static void callback_function(rcl_timer_t * timer, int64_t last_call)
{
  (void) timer;
  (void) last_call;
  times_called++;
}
static void callback_function_changed(rcl_timer_t * timer, int64_t last_call)
{
  (void) timer;
  (void) last_call;
  times_called--;
}

static size_t times_reset = 0;
static void on_reset_callback_function(const void * timer, size_t n)
{
  (void) timer;
  times_reset += n;
}
static void on_reset_callback_function_changed(const void * timer, size_t n)
{
  (void) timer;
  times_reset -= n;
}

class TestPreInitTimer : public TestTimerFixture
{
public:
  rcl_clock_t clock;
  rcl_allocator_t allocator;
  rcl_timer_t timer;
  rcl_timer_callback_t timer_callback_test = &callback_function;
  rcl_timer_callback_t timer_callback_changed = &callback_function_changed;

  void SetUp() override
  {
    TestTimerFixture::SetUp();
    rcl_ret_t ret;
    allocator = rcl_get_default_allocator();
    timer = rcl_get_zero_initialized_timer();
    ASSERT_EQ(
      RCL_RET_OK,
      rcl_clock_init(RCL_ROS_TIME, &clock, &allocator)) << rcl_get_error_string().str;

    ret = rcl_timer_init2(
      &timer, &clock, this->context_ptr, RCL_S_TO_NS(1), timer_callback_test,
      rcl_get_default_allocator(), true);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  }

  void TearDown() override
  {
    EXPECT_EQ(RCL_RET_OK, rcl_timer_fini(&timer)) << rcl_get_error_string().str;
    EXPECT_EQ(RCL_RET_OK, rcl_clock_fini(&clock)) << rcl_get_error_string().str;
    TestTimerFixture::TearDown();
  }
};

TEST_F(TestTimerFixture, test_timer_init_with_invalid_arguments) {
  rcl_clock_t clock;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_clock_init(RCL_STEADY_TIME, &clock, &allocator);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  rcl_timer_t timer = rcl_get_zero_initialized_timer();

  ret = rcl_timer_init2(
    nullptr, &clock, this->context_ptr, RCL_MS_TO_NS(50), nullptr, allocator, true);
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, ret);
  rcl_reset_error();

  ret = rcl_timer_init2(
    &timer, nullptr, this->context_ptr, RCL_MS_TO_NS(50), nullptr, allocator, true);
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, ret);
  rcl_reset_error();

  ret = rcl_timer_init2(
    &timer, &clock, nullptr, RCL_MS_TO_NS(50), nullptr, allocator, true);
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, ret);
  rcl_reset_error();

  ret = rcl_timer_init2(
    &timer, &clock, this->context_ptr, -1, nullptr, allocator, true);
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, ret);
  rcl_reset_error();

  rcl_allocator_t invalid_allocator = rcutils_get_zero_initialized_allocator();
  ret = rcl_timer_init2(
    &timer, &clock, this->context_ptr, RCL_MS_TO_NS(50), nullptr, invalid_allocator, true);
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, ret);
  rcl_reset_error();
}

TEST_F(TestTimerFixture, test_timer_with_invalid_clock) {
  rcl_clock_t clock;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_clock_init(RCL_CLOCK_UNINITIALIZED, &clock, &allocator);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  rcl_timer_t timer = rcl_get_zero_initialized_timer();
  ret = rcl_timer_init2(
    &timer, &clock, this->context_ptr, 0, nullptr, allocator, true);
  EXPECT_EQ(RCL_RET_ERROR, ret);
  rcl_reset_error();

  ret = rcl_clock_init(RCL_ROS_TIME, &clock, &allocator);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    rcl_ret_t ret = rcl_clock_fini(&clock);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });

  ret = rcl_timer_init2(
    &timer, &clock, this->context_ptr, 0, nullptr, allocator, true);
  ASSERT_EQ(RCL_RET_OK, ret);
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    rcl_ret_t ret = rcl_timer_fini(&timer);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });

  rcl_clock_t * timer_clock;
  ret = rcl_timer_clock(&timer, &timer_clock);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  timer_clock->get_now = nullptr;

  // Trigger clock jump callbacks
  ret = rcl_enable_ros_time_override(timer_clock);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  ret = rcl_timer_call(&timer);
  EXPECT_EQ(RCL_RET_ERROR, ret);
  rcl_reset_error();

  int64_t time_until_next_call;
  ret = rcl_timer_get_time_until_next_call(&timer, &time_until_next_call);
  EXPECT_EQ(RCL_RET_ERROR, ret);
  rcl_reset_error();

  bool ready;
  ret = rcl_timer_is_ready(&timer, &ready);
  EXPECT_EQ(RCL_RET_ERROR, ret);
  rcl_reset_error();

  rcl_time_point_value_t time_since_last_call;
  ret = rcl_timer_get_time_since_last_call(&timer, &time_since_last_call);
  EXPECT_EQ(RCL_RET_ERROR, ret);
  rcl_reset_error();

  ret = rcl_timer_reset(&timer);
  EXPECT_EQ(RCL_RET_ERROR, ret);
  rcl_reset_error();
}

TEST_F(TestTimerFixture, test_two_timers) {
  rcl_ret_t ret;

  rcl_clock_t clock;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  ret = rcl_clock_init(RCL_STEADY_TIME, &clock, &allocator);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  rcl_timer_t timer = rcl_get_zero_initialized_timer();
  rcl_timer_t timer2 = rcl_get_zero_initialized_timer();

  ret = rcl_timer_init2(
    &timer, &clock, this->context_ptr, RCL_MS_TO_NS(50), nullptr, rcl_get_default_allocator(),
    true);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  ret = rcl_timer_init2(
    &timer2, &clock, this->context_ptr, RCL_MS_TO_NS(1000), nullptr, rcl_get_default_allocator(),
    true);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  ret = rcl_wait_set_init(&wait_set, 0, 0, 2, 0, 0, 0, context_ptr, rcl_get_default_allocator());
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  ret = rcl_wait_set_add_timer(&wait_set, &timer, NULL);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  ret = rcl_wait_set_add_timer(&wait_set, &timer2, NULL);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    rcl_ret_t ret = rcl_timer_fini(&timer);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ret = rcl_timer_fini(&timer2);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ret = rcl_wait_set_fini(&wait_set);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });
  // The loop is needed because the rcl_wait_set might suffer spurious
  // awakes when timers are involved.
  // The loop can be removed if spurious awakes are fixed in the future.
  // This issue particularly happens on Windows.
  uint8_t nonnull_timers = 0;
  auto start = std::chrono::system_clock::now();
  do {
    ret = rcl_wait(&wait_set, RCL_MS_TO_NS(100));
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    for (uint8_t i = 0; i < wait_set.size_of_timers; i++) {
      if (wait_set.timers[i] != NULL) {
        nonnull_timers++;
      }
    }
  } while (
    nonnull_timers == 0u ||
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now() - start).count() > 100u);
  bool is_ready = false;
  ret = rcl_timer_is_ready(&timer, &is_ready);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  EXPECT_TRUE(is_ready);
  ret = rcl_timer_is_ready(&timer2, &is_ready);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  EXPECT_FALSE(is_ready);
  ASSERT_EQ(1, nonnull_timers);

  ret = rcl_clock_fini(&clock);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
}

TEST_F(TestTimerFixture, test_two_timers_ready_before_timeout) {
  rcl_ret_t ret;

  rcl_clock_t clock;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  ret = rcl_clock_init(RCL_STEADY_TIME, &clock, &allocator);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  rcl_timer_t timer = rcl_get_zero_initialized_timer();
  rcl_timer_t timer2 = rcl_get_zero_initialized_timer();

  // Keep the first timer period low enough so that rcl_wait() doesn't timeout too early.
  ret = rcl_timer_init2(
    &timer, &clock, this->context_ptr, RCL_MS_TO_NS(10), nullptr, rcl_get_default_allocator(),
    true);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  ret = rcl_timer_init2(
    &timer2, &clock, this->context_ptr, RCL_MS_TO_NS(1000), nullptr, rcl_get_default_allocator(),
    true);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  ret = rcl_wait_set_init(&wait_set, 0, 0, 2, 0, 0, 0, context_ptr, rcl_get_default_allocator());
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  ret = rcl_wait_set_add_timer(&wait_set, &timer, NULL);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  ret = rcl_wait_set_add_timer(&wait_set, &timer2, NULL);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    rcl_ret_t ret = rcl_timer_fini(&timer);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ret = rcl_timer_fini(&timer2);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ret = rcl_wait_set_fini(&wait_set);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });
  // The loop is needed because the rcl_wait_set might suffer spurious
  // awakes when timers are involved.
  // The loop can be removed if spurious awakes are fixed in the future.
  // This issue particularly happens on Windows.
  uint8_t nonnull_timers = 0u;
  auto start = std::chrono::system_clock::now();
  do {
    ret = rcl_wait(&wait_set, RCL_MS_TO_NS(100));
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    for (uint8_t i = 0; i < wait_set.size_of_timers; i++) {
      if (wait_set.timers[i] != NULL) {
        nonnull_timers++;
      }
    }
  } while (
    nonnull_timers == 0u ||
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now() - start).count() > 100u);
  bool is_ready = false;
  ret = rcl_timer_is_ready(&timer, &is_ready);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  EXPECT_TRUE(is_ready);
  ret = rcl_timer_is_ready(&timer2, &is_ready);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  EXPECT_FALSE(is_ready);
  ASSERT_EQ(1, nonnull_timers);

  ret = rcl_clock_fini(&clock);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
}

TEST_F(TestTimerFixture, test_timer_not_ready) {
  rcl_ret_t ret;

  rcl_clock_t clock;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  ret = rcl_clock_init(RCL_STEADY_TIME, &clock, &allocator);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  rcl_timer_t timer = rcl_get_zero_initialized_timer();

  ret = rcl_timer_init2(
    &timer, &clock, this->context_ptr, RCL_MS_TO_NS(1000), nullptr, rcl_get_default_allocator(),
    true);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  ret = rcl_wait_set_init(&wait_set, 0, 0, 1, 0, 0, 0, context_ptr, rcl_get_default_allocator());
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  ret = rcl_wait_set_add_timer(&wait_set, &timer, NULL);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    rcl_ret_t ret = rcl_timer_fini(&timer);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ret = rcl_wait_set_fini(&wait_set);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });
  ret = rcl_wait(&wait_set, RCL_MS_TO_NS(100));
  EXPECT_EQ(RCL_RET_TIMEOUT, ret) << rcl_get_error_string().str;
  uint8_t nonnull_timers = 0;
  for (uint8_t i = 0; i < wait_set.size_of_timers; i++) {
    if (wait_set.timers[i] != NULL) {
      nonnull_timers++;
    }
  }
  bool is_ready = false;
  ret = rcl_timer_is_ready(&timer, &is_ready);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  EXPECT_FALSE(is_ready);
  ASSERT_EQ(0, nonnull_timers);

  ret = rcl_clock_fini(&clock);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
}

TEST_F(TestTimerFixture, test_timer_overrun) {
  rcl_clock_t clock;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_clock_init(RCL_STEADY_TIME, &clock, &allocator);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    rcl_ret_t ret = rcl_clock_fini(&clock);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });

  rcl_timer_t timer = rcl_get_zero_initialized_timer();
  ret = rcl_timer_init2(
    &timer, &clock, this->context_ptr, RCL_MS_TO_NS(200), nullptr, rcl_get_default_allocator(),
    true);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    rcl_ret_t ret = rcl_timer_fini(&timer);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  ret = rcl_wait_set_init(&wait_set, 0, 0, 1, 0, 0, 0, context_ptr, rcl_get_default_allocator());
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    rcl_ret_t ret = rcl_wait_set_fini(&wait_set);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });

  // Force multiple timer timeouts.
  ret = rcl_wait(&wait_set, RCL_MS_TO_NS(500));
  EXPECT_EQ(RCL_RET_TIMEOUT, ret) << rcl_get_error_string().str;
  rcl_reset_error();

  bool is_ready = false;
  ret = rcl_timer_is_ready(&timer, &is_ready);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  EXPECT_TRUE(is_ready);

  EXPECT_EQ(RCL_RET_OK, rcl_timer_call(&timer)) << rcl_get_error_string().str;

  ret = rcl_wait_set_add_timer(&wait_set, &timer, NULL);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  // Ensure period is re-aligned.
  ret = rcl_wait(&wait_set, RCL_MS_TO_NS(10));
  EXPECT_EQ(RCL_RET_TIMEOUT, ret) << rcl_get_error_string().str;
  rcl_reset_error();

  ret = rcl_timer_is_ready(&timer, &is_ready);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  EXPECT_FALSE(is_ready);
}

TEST_F(TestTimerFixture, test_timer_with_zero_period) {
  rcl_clock_t clock;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_clock_init(RCL_STEADY_TIME, &clock, &allocator);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    rcl_ret_t ret = rcl_clock_fini(&clock);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });

  rcl_timer_t timer = rcl_get_zero_initialized_timer();
  ret = rcl_timer_init2(
    &timer, &clock, this->context_ptr, 0, nullptr, rcl_get_default_allocator(),
    true);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    rcl_ret_t ret = rcl_timer_fini(&timer);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });

  bool is_ready = false;
  ret = rcl_timer_is_ready(&timer, &is_ready);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  EXPECT_TRUE(is_ready) << rcl_get_error_string().str;

  int64_t time_until_next_call = 0;
  ret = rcl_timer_get_time_until_next_call(&timer, &time_until_next_call);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  EXPECT_LE(time_until_next_call, 0);

  EXPECT_EQ(RCL_RET_OK, rcl_timer_call(&timer)) << rcl_get_error_string().str;
}

TEST_F(TestTimerFixture, test_timer_init_state) {
  rcl_ret_t ret;

  rcl_clock_t clock;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  ret = rcl_clock_init(RCL_STEADY_TIME, &clock, &allocator);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  rcl_timer_t timer = rcl_get_zero_initialized_timer();

  ret = rcl_timer_init2(
    &timer, &clock, this->context_ptr, RCL_S_TO_NS(1), nullptr, rcl_get_default_allocator(),
    false);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  bool is_canceled = false;
  ret = rcl_timer_is_canceled(&timer, &is_canceled);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  EXPECT_TRUE(is_canceled);

  ret = rcl_timer_init2(
    &timer, &clock, this->context_ptr, RCL_S_TO_NS(1), nullptr, rcl_get_default_allocator(),
    true);
  ASSERT_EQ(RCL_RET_ALREADY_INIT, ret) << rcl_get_error_string().str;
  rcl_reset_error();

  timer = rcl_get_zero_initialized_timer();

  ret = rcl_timer_init2(
    &timer, &clock, this->context_ptr, RCL_S_TO_NS(1), nullptr, rcl_get_default_allocator(),
    true);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  ret = rcl_timer_is_canceled(&timer, &is_canceled);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  EXPECT_FALSE(is_canceled);
}

TEST_F(TestTimerFixture, test_canceled_timer) {
  rcl_ret_t ret;

  rcl_clock_t clock;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  ret = rcl_clock_init(RCL_STEADY_TIME, &clock, &allocator);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  rcl_timer_t timer = rcl_get_zero_initialized_timer();

  ret = rcl_timer_init2(
    &timer, &clock, this->context_ptr, 500, nullptr, rcl_get_default_allocator(),
    true);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  ret = rcl_timer_cancel(&timer);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  int64_t time_until_next_call = 0;
  ret = rcl_timer_get_time_until_next_call(&timer, &time_until_next_call);
  EXPECT_EQ(RCL_RET_TIMER_CANCELED, ret) << rcl_get_error_string().str;

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  ret = rcl_wait_set_init(&wait_set, 0, 0, 1, 0, 0, 0, context_ptr, rcl_get_default_allocator());
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  ret = rcl_wait_set_add_timer(&wait_set, &timer, NULL);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    rcl_ret_t ret = rcl_timer_fini(&timer);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ret = rcl_wait_set_fini(&wait_set);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });
  ret = rcl_wait(&wait_set, RCL_MS_TO_NS(1));
  EXPECT_EQ(RCL_RET_TIMEOUT, ret) << rcl_get_error_string().str;
  uint8_t nonnull_timers = 0;
  for (uint8_t i = 0; i < wait_set.size_of_timers; i++) {
    if (wait_set.timers[i] != NULL) {
      nonnull_timers++;
    }
  }
  bool is_ready = false;
  ret = rcl_timer_is_ready(&timer, &is_ready);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  EXPECT_FALSE(is_ready);
  ASSERT_EQ(0, nonnull_timers);

  ret = rcl_clock_fini(&clock);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
}

TEST_F(TestTimerFixture, test_rostime_time_until_next_call) {
  rcl_ret_t ret;
  const int64_t sec_5 = RCL_S_TO_NS(5);
  int64_t time_until = 0;

  rcl_clock_t clock;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  ret = rcl_clock_init(RCL_ROS_TIME, &clock, &allocator);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    EXPECT_EQ(RCL_RET_OK, rcl_clock_fini(&clock)) << rcl_get_error_string().str;
  });
  ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(&clock)) << rcl_get_error_string().str;

  rcl_timer_t timer = rcl_get_zero_initialized_timer();
  ret = rcl_timer_init2(
    &timer, &clock, this->context_ptr, sec_5, nullptr, rcl_get_default_allocator(),
    true);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    EXPECT_EQ(RCL_RET_OK, rcl_timer_fini(&timer)) << rcl_get_error_string().str;
  });

  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(&clock, 1)) << rcl_get_error_string().str;
  ret = rcl_timer_get_time_until_next_call(&timer, &time_until);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  EXPECT_EQ(sec_5 - 1, time_until);

  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(&clock, sec_5)) << rcl_get_error_string().str;
  ret = rcl_timer_get_time_until_next_call(&timer, &time_until);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  EXPECT_EQ(0, time_until);

  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(&clock, sec_5 + 1)) <<
    rcl_get_error_string().str;
  ret = rcl_timer_get_time_until_next_call(&timer, &time_until);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  EXPECT_EQ(-1, time_until);
}

TEST_F(TestTimerFixture, test_system_time_to_ros_time) {
  rcl_ret_t ret;
  const int64_t sec_5 = RCL_S_TO_NS(5);

  rcl_clock_t clock;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  ret = rcl_clock_init(RCL_ROS_TIME, &clock, &allocator);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    EXPECT_EQ(RCL_RET_OK, rcl_clock_fini(&clock)) << rcl_get_error_string().str;
  });

  rcl_timer_t timer = rcl_get_zero_initialized_timer();
  ret = rcl_timer_init2(
    &timer, &clock, this->context_ptr, sec_5, nullptr, rcl_get_default_allocator(),
    true);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    EXPECT_EQ(RCL_RET_OK, rcl_timer_fini(&timer)) << rcl_get_error_string().str;
  });

  int64_t time_until_pre = 0;
  ASSERT_EQ(RCL_RET_OK, rcl_timer_get_time_until_next_call(&timer, &time_until_pre)) <<
    rcl_get_error_string().str;
  ASSERT_LT(0, time_until_pre);
  ASSERT_GT(sec_5, time_until_pre);

  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(&clock, 1)) << rcl_get_error_string().str;
  ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(&clock)) << rcl_get_error_string().str;

  int64_t time_until = 0;
  ASSERT_EQ(RCL_RET_OK, rcl_timer_get_time_until_next_call(&timer, &time_until)) <<
    rcl_get_error_string().str;
  // Because of time credit the time until next call should be less than before
  EXPECT_GT(time_until_pre, time_until);
  EXPECT_LT(0, time_until);
}

TEST_F(TestTimerFixture, test_ros_time_to_system_time) {
  rcl_ret_t ret;
  const int64_t sec_5 = RCL_S_TO_NS(5);
  const int64_t sec_1 = RCL_S_TO_NS(1);

  rcl_clock_t clock;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  ret = rcl_clock_init(RCL_ROS_TIME, &clock, &allocator);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    EXPECT_EQ(RCL_RET_OK, rcl_clock_fini(&clock)) << rcl_get_error_string().str;
  });

  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(&clock, 1)) << rcl_get_error_string().str;
  ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(&clock)) << rcl_get_error_string().str;

  rcl_timer_t timer = rcl_get_zero_initialized_timer();
  ret = rcl_timer_init2(
    &timer, &clock, this->context_ptr, sec_5, nullptr, rcl_get_default_allocator(),
    true);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    EXPECT_EQ(RCL_RET_OK, rcl_timer_fini(&timer)) << rcl_get_error_string().str;
  });

  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(&clock, sec_1)) << rcl_get_error_string().str;
  ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(&clock)) << rcl_get_error_string().str;

  int64_t time_until_pre = 0;
  ASSERT_EQ(RCL_RET_OK, rcl_timer_get_time_until_next_call(&timer, &time_until_pre)) <<
    rcl_get_error_string().str;
  ASSERT_EQ(sec_5 - (sec_1 - 1), time_until_pre);

  ASSERT_EQ(RCL_RET_OK, rcl_disable_ros_time_override(&clock)) << rcl_get_error_string().str;

  int64_t time_until = 0;
  ASSERT_EQ(RCL_RET_OK, rcl_timer_get_time_until_next_call(&timer, &time_until)) <<
    rcl_get_error_string().str;
  // Because of time credit the time until next call should be less than before
  EXPECT_GT(time_until_pre, time_until);
  EXPECT_LT(0, time_until);
}

TEST_F(TestTimerFixture, test_ros_time_backwards_jump) {
  rcl_ret_t ret;
  const int64_t sec_5 = RCL_S_TO_NS(5);
  const int64_t sec_3 = RCL_S_TO_NS(3);
  const int64_t sec_2 = RCL_S_TO_NS(2);
  const int64_t sec_1 = RCL_S_TO_NS(1);

  rcl_clock_t clock;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  ret = rcl_clock_init(RCL_ROS_TIME, &clock, &allocator);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    EXPECT_EQ(RCL_RET_OK, rcl_clock_fini(&clock)) << rcl_get_error_string().str;
  });

  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(&clock, sec_2)) << rcl_get_error_string().str;
  ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(&clock)) << rcl_get_error_string().str;

  rcl_timer_t timer = rcl_get_zero_initialized_timer();
  ret = rcl_timer_init2(
    &timer, &clock, this->context_ptr, sec_5, nullptr, rcl_get_default_allocator(),
    true);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    EXPECT_EQ(RCL_RET_OK, rcl_timer_fini(&timer)) << rcl_get_error_string().str;
  });

  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(&clock, sec_3)) << rcl_get_error_string().str;
  {
    // Moved forward a little bit, timer should be closer to being ready
    int64_t time_until = 0;
    ASSERT_EQ(RCL_RET_OK, rcl_timer_get_time_until_next_call(&timer, &time_until)) <<
      rcl_get_error_string().str;
    EXPECT_EQ(sec_5 - (sec_3 - sec_2), time_until);
  }
  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(&clock, sec_1)) << rcl_get_error_string().str;
  {
    // Jumped back before timer was created, so last_call_time should be 1 period
    int64_t time_until = 0;
    ASSERT_EQ(RCL_RET_OK, rcl_timer_get_time_until_next_call(&timer, &time_until)) <<
      rcl_get_error_string().str;
    EXPECT_EQ(sec_5, time_until);
  }
}

TEST_F(TestTimerFixture, test_ros_time_wakes_wait) {
  const int64_t sec_5 = RCL_S_TO_NS(5);
  const int64_t sec_1 = RCL_S_TO_NS(1);
  const int64_t sec_1_5 = RCL_S_TO_NS(3) / 2;

  rcl_ret_t ret;
  rcl_clock_t clock;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  ASSERT_EQ(RCL_RET_OK, rcl_clock_init(RCL_ROS_TIME, &clock, &allocator)) <<
    rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    EXPECT_EQ(RCL_RET_OK, rcl_clock_fini(&clock)) << rcl_get_error_string().str;
  });
  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(&clock, sec_1)) << rcl_get_error_string().str;
  ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(&clock)) << rcl_get_error_string().str;

  rcl_timer_t timer = rcl_get_zero_initialized_timer();
  ret = rcl_timer_init2(
    &timer, &clock, this->context_ptr, sec_1, nullptr, rcl_get_default_allocator(),
    true);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    EXPECT_EQ(RCL_RET_OK, rcl_timer_fini(&timer)) << rcl_get_error_string().str;
  });

  bool timer_was_ready = false;

  std::thread wait_thr([&](void) {
      rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
      ret = rcl_wait_set_init(
        &wait_set,
        0, 0, 1, 0, 0, 0,
        context_ptr,
        rcl_get_default_allocator());
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

      ASSERT_EQ(RCL_RET_OK, rcl_wait_set_add_timer(&wait_set, &timer, NULL)) <<
        rcl_get_error_string().str;
      // *INDENT-OFF* (Uncrustify wants strange un-indentation here)
      OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT({
        EXPECT_EQ(RCL_RET_OK, rcl_wait_set_fini(&wait_set)) <<
          rcl_get_error_string().str;
      });
      // *INDENT-ON*

      ret = rcl_wait(&wait_set, sec_5);
      // set some flag indicating wait was exited
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
      if (wait_set.timers[0] != NULL) {
        timer_was_ready = true;
      }
    });

  // Timer not exceeded, should not wake
  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(&clock, sec_1_5)) <<
    rcl_get_error_string().str;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_FALSE(timer_was_ready);

  // Timer exceeded, should wake
  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(&clock, sec_5)) << rcl_get_error_string().str;
  auto start = std::chrono::steady_clock::now();
  wait_thr.join();
  auto finish = std::chrono::steady_clock::now();
  EXPECT_TRUE(timer_was_ready);
  EXPECT_LT(finish - start, std::chrono::milliseconds(100));
}

TEST_F(TestPreInitTimer, test_timer_get_allocator) {
  const rcl_allocator_t * allocator_returned = rcl_timer_get_allocator(&timer);
  EXPECT_TRUE(rcutils_allocator_is_valid(allocator_returned));

  EXPECT_EQ(NULL, rcl_timer_get_allocator(nullptr));
  rcl_reset_error();
}

TEST_F(TestPreInitTimer, test_timer_clock) {
  rcl_clock_t * clock_impl = nullptr;
  EXPECT_EQ(RCL_RET_OK, rcl_timer_clock(&timer, &clock_impl)) << rcl_get_error_string().str;
  EXPECT_EQ(clock_impl, &clock);
}

TEST_F(TestPreInitTimer, test_timer_call) {
  int64_t next_call_start = 0;
  int64_t next_call_end = 0;
  int64_t old_period = 0;
  times_called = 0;

  EXPECT_EQ(RCL_RET_OK, rcl_timer_get_time_until_next_call(&timer, &next_call_start));
  ASSERT_EQ(RCL_RET_OK, rcl_timer_call(&timer)) << rcl_get_error_string().str;
  EXPECT_EQ(times_called, 1);

  ASSERT_EQ(RCL_RET_OK, rcl_timer_call(&timer)) << rcl_get_error_string().str;
  ASSERT_EQ(RCL_RET_OK, rcl_timer_call(&timer)) << rcl_get_error_string().str;
  EXPECT_EQ(times_called, 3);
  EXPECT_EQ(RCL_RET_OK, rcl_timer_get_time_until_next_call(&timer, &next_call_end));
  EXPECT_GT(next_call_end, next_call_start);

  next_call_start = next_call_end;
  ASSERT_EQ(RCL_RET_OK, rcl_timer_exchange_period(&timer, 0, &old_period));
  EXPECT_EQ(RCL_S_TO_NS(1), old_period);
  ASSERT_EQ(RCL_RET_OK, rcl_timer_call(&timer)) << rcl_get_error_string().str;
  EXPECT_EQ(times_called, 4);
  EXPECT_EQ(RCL_RET_OK, rcl_timer_get_time_until_next_call(&timer, &next_call_end));
  EXPECT_GT(next_call_start, next_call_end);

  EXPECT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(&this->clock)) << rcl_get_error_string().str;
  EXPECT_EQ(RCL_RET_OK, rcl_set_ros_time_override(&this->clock, -1)) << rcl_get_error_string().str;
  EXPECT_EQ(RCL_RET_ERROR, rcl_timer_call(&timer));
  rcl_reset_error();
  EXPECT_EQ(times_called, 4);

  EXPECT_EQ(RCL_RET_OK, rcl_timer_cancel(&timer)) << rcl_get_error_string().str;
  EXPECT_EQ(RCL_RET_TIMER_CANCELED, rcl_timer_call(&timer));
  rcl_reset_error();
  EXPECT_EQ(times_called, 4);
}

TEST_F(TestPreInitTimer, test_get_callback) {
  ASSERT_EQ(timer_callback_test, rcl_timer_get_callback(&timer)) << rcl_get_error_string().str;
}

TEST_F(TestPreInitTimer, test_timer_reset) {
  int64_t next_call_start = 0;
  int64_t next_call_end = 0;
  times_called = 0;

  ASSERT_EQ(RCL_RET_OK, rcl_timer_call(&timer)) << rcl_get_error_string().str;
  ASSERT_EQ(RCL_RET_OK, rcl_timer_call(&timer)) << rcl_get_error_string().str;
  EXPECT_EQ(times_called, 2);
  EXPECT_EQ(RCL_RET_OK, rcl_timer_get_time_until_next_call(&timer, &next_call_start));

  ASSERT_EQ(RCL_RET_OK, rcl_timer_reset(&timer));
  EXPECT_EQ(RCL_RET_OK, rcl_timer_get_time_until_next_call(&timer, &next_call_end));
  EXPECT_GT(next_call_start, next_call_end);

  ASSERT_EQ(RCL_RET_OK, rcl_timer_cancel(&timer)) << rcl_get_error_string().str;
  EXPECT_EQ(RCL_RET_TIMER_CANCELED, rcl_timer_call(&timer));
  rcl_reset_error();
  EXPECT_EQ(times_called, 2);
  ASSERT_EQ(RCL_RET_OK, rcl_timer_reset(&timer));
  EXPECT_EQ(RCL_RET_OK, rcl_timer_call(&timer)) << rcl_get_error_string().str;
  EXPECT_EQ(times_called, 3);
}

TEST_F(TestPreInitTimer, test_timer_exchange_callback) {
  times_called = 0;
  ASSERT_EQ(RCL_RET_OK, rcl_timer_call(&timer)) << rcl_get_error_string().str;
  EXPECT_EQ(times_called, 1);
  ASSERT_EQ(
    timer_callback_test, rcl_timer_exchange_callback(
      &timer, timer_callback_changed)) << rcl_get_error_string().str;

  ASSERT_EQ(RCL_RET_OK, rcl_timer_call(&timer)) << rcl_get_error_string().str;
  EXPECT_EQ(times_called, 0);
}

TEST_F(TestPreInitTimer, test_on_reset_timer_callback) {
  // Set callback to an invalid timer
  EXPECT_EQ(
    RCL_RET_INVALID_ARGUMENT,
    rcl_timer_set_on_reset_callback(nullptr, nullptr, nullptr));
  rcl_reset_error();

  // Set a null on reset callback to a valid timer
  ASSERT_EQ(
    RCL_RET_OK, rcl_timer_set_on_reset_callback(
      &timer, nullptr, nullptr)) << rcl_get_error_string().str;

  // Reset 2 times, then set callback and check times_reset
  times_reset = 0;
  ASSERT_EQ(RCL_RET_OK, rcl_timer_reset(&timer)) << rcl_get_error_string().str;
  ASSERT_EQ(RCL_RET_OK, rcl_timer_reset(&timer)) << rcl_get_error_string().str;
  ASSERT_EQ(
    RCL_RET_OK, rcl_timer_set_on_reset_callback(
      &timer, on_reset_callback_function, nullptr)) << rcl_get_error_string().str;
  EXPECT_EQ(times_reset, static_cast<size_t>(2));

  // Assign a new on_reset callback
  ASSERT_EQ(
    RCL_RET_OK, rcl_timer_set_on_reset_callback(
      &timer, on_reset_callback_function_changed, nullptr)) << rcl_get_error_string().str;
  ASSERT_EQ(RCL_RET_OK, rcl_timer_reset(&timer)) << rcl_get_error_string().str;
  EXPECT_EQ(times_reset, static_cast<size_t>(1));
}

TEST_F(TestPreInitTimer, test_invalid_get_guard) {
  ASSERT_EQ(NULL, rcl_timer_get_guard_condition(nullptr));
  rcl_reset_error();
}

TEST_F(TestPreInitTimer, test_invalid_init_fini) {
  rcl_allocator_t bad_allocator = get_failing_allocator();
  rcl_timer_t timer_fail = rcl_get_zero_initialized_timer();

  EXPECT_EQ(
    RCL_RET_ALREADY_INIT, rcl_timer_init2(
      &timer, &clock, this->context_ptr, 500, nullptr,
      rcl_get_default_allocator(), true)) << rcl_get_error_string().str;
  rcl_reset_error();

  ASSERT_EQ(
    RCL_RET_BAD_ALLOC, rcl_timer_init2(
      &timer_fail, &clock, this->context_ptr, RCL_S_TO_NS(1), timer_callback_test,
      bad_allocator, true)) << rcl_get_error_string().str;
  rcl_reset_error();

  EXPECT_EQ(RCL_RET_OK, rcl_timer_fini(nullptr)) << rcl_get_error_string().str;
}

TEST_F(TestPreInitTimer, test_timer_get_period) {
  int64_t period = 0;
  ASSERT_EQ(RCL_RET_OK, rcl_timer_get_period(&timer, &period));
  EXPECT_EQ(RCL_S_TO_NS(1), period);

  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, rcl_timer_get_period(nullptr, &period));
  rcl_reset_error();
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, rcl_timer_get_period(&timer, nullptr));
  rcl_reset_error();
}

TEST_F(TestPreInitTimer, test_timer_info) {
  int64_t next_call_start = 0;
  int64_t old_period = 0;
  times_called = 0;
  rcl_timer_call_info_t call_info;
  call_info.actual_call_time = 0;
  call_info.expected_call_time = 0;
  int64_t period = RCL_MS_TO_NS(100);

  ASSERT_EQ(RCL_RET_OK, rcl_timer_exchange_period(&timer, period, &old_period));
  ASSERT_EQ(RCL_RET_OK, rcl_timer_reset(&timer));
  ASSERT_EQ(RCL_RET_OK, rcl_timer_call_with_info(&timer, &call_info)) << rcl_get_error_string().str;
  EXPECT_EQ(times_called, 1);

  int64_t next_expected_call_time = call_info.expected_call_time + period;

  EXPECT_EQ(RCL_RET_OK, rcl_timer_get_time_until_next_call(&timer, &next_call_start));
  std::this_thread::sleep_for(std::chrono::nanoseconds(next_call_start));

  ASSERT_EQ(RCL_RET_OK, rcl_timer_call_with_info(&timer, &call_info)) << rcl_get_error_string().str;
  ASSERT_EQ(next_expected_call_time, call_info.expected_call_time);
  ASSERT_GE(call_info.actual_call_time, call_info.expected_call_time);
  EXPECT_EQ(times_called, 2);

  next_expected_call_time = call_info.expected_call_time + period;

  EXPECT_EQ(RCL_RET_OK, rcl_timer_get_time_until_next_call(&timer, &next_call_start));
  std::this_thread::sleep_for(std::chrono::nanoseconds(next_call_start));

  ASSERT_EQ(RCL_RET_OK, rcl_timer_call_with_info(&timer, &call_info)) << rcl_get_error_string().str;
  ASSERT_EQ(next_expected_call_time, call_info.expected_call_time);
  ASSERT_GE(call_info.actual_call_time, call_info.expected_call_time);
  EXPECT_EQ(times_called, 3);

  next_expected_call_time = call_info.expected_call_time + period;

  EXPECT_EQ(RCL_RET_OK, rcl_timer_cancel(&timer)) << rcl_get_error_string().str;
  EXPECT_EQ(RCL_RET_TIMER_CANCELED, rcl_timer_call(&timer));
  rcl_reset_error();
  EXPECT_EQ(times_called, 3);
}

TEST_F(TestPreInitTimer, test_timer_info_detect_overrun) {
  int64_t next_call_start = 0;
  int64_t old_period = 0;
  times_called = 0;
  rcl_timer_call_info_t call_info;
  call_info.actual_call_time = 0;
  call_info.expected_call_time = 0;
  int64_t period = RCL_MS_TO_NS(100);

  ASSERT_EQ(RCL_RET_OK, rcl_timer_exchange_period(&timer, period, &old_period));
  ASSERT_EQ(RCL_RET_OK, rcl_timer_reset(&timer));
  ASSERT_EQ(RCL_RET_OK, rcl_timer_call_with_info(&timer, &call_info)) << rcl_get_error_string().str;
  EXPECT_EQ(times_called, 1);

  int64_t next_expected_call_time = call_info.expected_call_time + period;

  EXPECT_EQ(RCL_RET_OK, rcl_timer_get_time_until_next_call(&timer, &next_call_start));
  std::this_thread::sleep_for(std::chrono::nanoseconds(next_call_start + period));

  ASSERT_EQ(RCL_RET_OK, rcl_timer_call_with_info(&timer, &call_info)) << rcl_get_error_string().str;
  ASSERT_EQ(next_expected_call_time, call_info.expected_call_time);
  ASSERT_GE(call_info.actual_call_time, call_info.expected_call_time);
  // check, if we can detect a timer overrun
  ASSERT_GE(call_info.actual_call_time - call_info.expected_call_time, period);
  EXPECT_EQ(times_called, 2);

  // check, if the expected_call_time for next call is as expected, and skips a period
  next_expected_call_time = call_info.expected_call_time + period + period;

  EXPECT_EQ(RCL_RET_OK, rcl_timer_get_time_until_next_call(&timer, &next_call_start));
  std::this_thread::sleep_for(std::chrono::nanoseconds(next_call_start));

  ASSERT_EQ(RCL_RET_OK, rcl_timer_call_with_info(&timer, &call_info)) << rcl_get_error_string().str;
  ASSERT_EQ(next_expected_call_time, call_info.expected_call_time);
  ASSERT_GE(call_info.actual_call_time, call_info.expected_call_time);
  EXPECT_EQ(times_called, 3);

  EXPECT_EQ(RCL_RET_OK, rcl_timer_cancel(&timer)) << rcl_get_error_string().str;
  EXPECT_EQ(RCL_RET_TIMER_CANCELED, rcl_timer_call(&timer));
  rcl_reset_error();
  EXPECT_EQ(times_called, 3);
}

TEST_F(TestPreInitTimer, test_time_since_last_call) {
  rcl_time_point_value_t time_sice_next_call_start = 0u;
  rcl_time_point_value_t time_sice_next_call_end = 0u;

  ASSERT_EQ(RCL_RET_OK, rcl_timer_get_time_since_last_call(&timer, &time_sice_next_call_start));
  // Cope with coarse system time resolution.
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ASSERT_EQ(RCL_RET_OK, rcl_timer_get_time_since_last_call(&timer, &time_sice_next_call_end));
  EXPECT_GT(time_sice_next_call_end, time_sice_next_call_start);
}

namespace
{
struct ResetCount
{
  int tag;
  std::atomic<size_t> * total;
  std::atomic<size_t> * mismatches;
};

void count_reset_one(const void * user_data, size_t count)
{
  const auto & state = *static_cast<const ResetCount *>(user_data);
  if (state.tag != 1) {
    state.mismatches->fetch_add(1);
  }
  state.total->fetch_add(count);
}

void count_reset_two(const void * user_data, size_t count)
{
  const auto & state = *static_cast<const ResetCount *>(user_data);
  if (state.tag != 2) {
    state.mismatches->fetch_add(1);
  }
  state.total->fetch_add(count);
}
}  // namespace

TEST_F(TestPreInitTimer, reset_callback_pending_count_is_not_lost_on_reentry)
{
  ASSERT_EQ(RCL_RET_OK, rcl_timer_reset(&timer));
  struct ReentrantReset
  {
    rcl_timer_t * timer;
    rcl_ret_t clear_result = RCL_RET_ERROR;
    rcl_ret_t reset_result = RCL_RET_ERROR;
    size_t notifications = 0;
  } state{&timer};
  const auto callback = [](const void * data, size_t count) {
      auto & state = *const_cast<ReentrantReset *>(static_cast<const ReentrantReset *>(data));
      state.notifications += count;
      state.clear_result = rcl_timer_set_on_reset_callback(state.timer, nullptr, nullptr);
      state.reset_result = rcl_timer_reset(state.timer);
    };
  ASSERT_EQ(RCL_RET_OK, rcl_timer_set_on_reset_callback(&timer, callback, &state));
  EXPECT_EQ(RCL_RET_OK, state.clear_result);
  EXPECT_EQ(RCL_RET_OK, state.reset_result);
  EXPECT_EQ(1u, state.notifications);
  std::atomic<size_t> total{0};
  std::atomic<size_t> mismatches{0};
  const ResetCount counter{1, &total, &mismatches};
  ASSERT_EQ(RCL_RET_OK, rcl_timer_set_on_reset_callback(&timer, count_reset_one, &counter));
  EXPECT_EQ(1u, total.load());
  EXPECT_EQ(0u, mismatches.load());
  EXPECT_EQ(RCL_RET_OK, rcl_timer_set_on_reset_callback(&timer, nullptr, nullptr));
}

TEST_F(TestPreInitTimer, concurrent_resets_preserve_every_pending_notification)
{
  std::atomic<size_t> errors{0};
  std::vector<std::thread> workers;
  for (size_t worker = 0; worker < 8; ++worker) {
    workers.emplace_back([&]() {
        for (size_t round = 0; round < 512; ++round) {
          if (RCL_RET_OK != rcl_timer_reset(&timer)) {
            errors.fetch_add(1);
          }
        }
      });
  }
  for (auto & worker : workers) {
    worker.join();
  }
  std::atomic<size_t> total{0};
  std::atomic<size_t> mismatches{0};
  const ResetCount counter{1, &total, &mismatches};
  ASSERT_EQ(RCL_RET_OK, rcl_timer_set_on_reset_callback(&timer, count_reset_one, &counter));
  EXPECT_EQ(0u, errors.load());
  EXPECT_EQ(4096u, total.load());
  EXPECT_EQ(0u, mismatches.load());
  EXPECT_EQ(RCL_RET_OK, rcl_timer_set_on_reset_callback(&timer, nullptr, nullptr));
}

TEST_F(TestPreInitTimer, concurrent_reset_callback_replacement_preserves_userdata_pair)
{
  std::atomic<size_t> total{0};
  std::atomic<size_t> mismatches{0};
  std::atomic<size_t> errors{0};
  const ResetCount first{1, &total, &mismatches};
  const ResetCount second{2, &total, &mismatches};
  ASSERT_EQ(RCL_RET_OK, rcl_timer_set_on_reset_callback(&timer, count_reset_one, &first));
  std::atomic<bool> start{false};
  std::atomic<size_t> ready{0};
  std::vector<std::thread> workers;
  for (size_t worker = 0; worker < 8; ++worker) {
    workers.emplace_back([&, worker]() {
        ready.fetch_add(1);
        while (!start.load()) {
          std::this_thread::yield();
        }
        for (size_t round = 0; round < 2048; ++round) {
          rcl_ret_t ret;
          if (worker < 4) {
            ret = rcl_timer_reset(&timer);
          } else if ((worker + round) % 2 == 0) {
            ret = rcl_timer_set_on_reset_callback(&timer, count_reset_one, &first);
          } else {
            ret = rcl_timer_set_on_reset_callback(&timer, count_reset_two, &second);
          }
          if (ret != RCL_RET_OK) {
            errors.fetch_add(1);
          }
        }
      });
  }
  while (ready.load() != 8) {
    std::this_thread::yield();
  }
  start.store(true);
  for (auto & worker : workers) {
    worker.join();
  }
  EXPECT_EQ(RCL_RET_OK, rcl_timer_set_on_reset_callback(&timer, nullptr, nullptr));
  EXPECT_EQ(0u, errors.load());
  EXPECT_EQ(0u, mismatches.load());
  EXPECT_EQ(8192u, total.load());
}

TEST_F(TestPreInitTimer, removing_reset_callback_waits_for_its_inflight_userdata)
{
  struct BlockingReset
  {
    std::mutex mutex;
    std::condition_variable cv;
    bool entered = false;
    bool released = false;
    bool callback_timed_out = false;
    std::atomic<bool> removing{false};
    std::atomic<bool> removed{false};
  } state;
  const auto callback = [](const void * data, size_t) {
      auto & state = *const_cast<BlockingReset *>(static_cast<const BlockingReset *>(data));
      std::unique_lock<std::mutex> lock(state.mutex);
      state.entered = true;
      state.cv.notify_all();
      if (!state.cv.wait_for(lock, std::chrono::seconds(5), [&]() {return state.released;})) {
        state.callback_timed_out = true;
      }
    };
  ASSERT_EQ(RCL_RET_OK, rcl_timer_set_on_reset_callback(&timer, callback, &state));
  rcl_ret_t reset_result = RCL_RET_ERROR;
  rcl_ret_t remove_result = RCL_RET_ERROR;
  std::thread resetter([&]() {reset_result = rcl_timer_reset(&timer);});
  bool entered;
  {
    std::unique_lock<std::mutex> lock(state.mutex);
    entered = state.cv.wait_for(lock, std::chrono::seconds(5), [&]() {return state.entered;});
  }
  std::thread remover([&]() {
      state.removing.store(true);
      state.cv.notify_all();
      remove_result = rcl_timer_set_on_reset_callback(&timer, nullptr, nullptr);
      state.removed.store(true);
      state.cv.notify_all();
    });
  bool removing;
  bool removed_early;
  {
    std::unique_lock<std::mutex> lock(state.mutex);
    removing = state.cv.wait_for(lock, std::chrono::seconds(5), [&]() {return state.removing.load();});
    removed_early = state.cv.wait_for(lock, std::chrono::milliseconds(25), [&]() {return state.removed.load();});
    state.released = true;
    state.cv.notify_all();
  }
  resetter.join();
  remover.join();
  EXPECT_TRUE(entered);
  EXPECT_TRUE(removing);
  EXPECT_FALSE(removed_early);
  EXPECT_FALSE(state.callback_timed_out);
  EXPECT_TRUE(state.removed.load());
  EXPECT_EQ(RCL_RET_OK, reset_result);
  EXPECT_EQ(RCL_RET_OK, remove_result);
}

TEST_F(TestTimerFixture, reset_callback_rejects_uninitialized_timer)
{
  const rcl_timer_t uninitialized = rcl_get_zero_initialized_timer();
  EXPECT_EQ(RCL_RET_TIMER_INVALID, rcl_timer_set_on_reset_callback(&uninitialized, nullptr, nullptr));
  rcl_reset_error();
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, rcl_timer_set_on_reset_callback(nullptr, nullptr, nullptr));
  rcl_reset_error();
}
