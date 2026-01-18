#include "RobotApp/Platform/heap_guard.hpp"

#include <atomic>
#include <cstdlib>
#include <new>

namespace robotapp::platform {

namespace {
std::atomic<bool> g_started{false};
}  // namespace

[[noreturn]] void heap_guard_trap() noexcept
{
#if defined(__GNUC__) && (defined(__arm__) || defined(__thumb__) || defined(__ARM_ARCH))
  __asm volatile("bkpt 0");
#endif
  for (;;)
  {
  }
}

void heap_guard_mark_started() noexcept
{
  g_started.store(true, std::memory_order_relaxed);
  g_robotapp_heap_started = 1U;
}

bool heap_guard_started() noexcept { return g_started.load(std::memory_order_relaxed); }

}  // namespace robotapp::platform

extern "C" {

volatile uint32_t g_robotapp_heap_started = 0;
volatile uint32_t g_robotapp_heap_malloc_calls = 0;
volatile uint32_t g_robotapp_heap_free_calls = 0;
volatile uint32_t g_robotapp_heap_calloc_calls = 0;
volatile uint32_t g_robotapp_heap_realloc_calls = 0;
volatile uint32_t g_robotapp_heap_calls_after_start = 0;
volatile uint32_t g_robotapp_heap_alloc_fail = 0;

// Linker --wrap() entry points (enabled via target_link_options in CMakeLists.txt).
void* __real_malloc(size_t size);
void __real_free(void* ptr);
void* __real_calloc(size_t n, size_t size);
void* __real_realloc(void* ptr, size_t size);

void* __wrap_malloc(size_t size)
{
  g_robotapp_heap_malloc_calls += 1U;
  if (robotapp::platform::heap_guard_started())
  {
    g_robotapp_heap_calls_after_start += 1U;
    robotapp::platform::heap_guard_trap();
  }
  void* p = __real_malloc(size);
  if (p == nullptr) g_robotapp_heap_alloc_fail += 1U;
  return p;
}

void __wrap_free(void* ptr)
{
  g_robotapp_heap_free_calls += 1U;
  if (robotapp::platform::heap_guard_started())
  {
    g_robotapp_heap_calls_after_start += 1U;
    robotapp::platform::heap_guard_trap();
  }
  __real_free(ptr);
}

void* __wrap_calloc(size_t n, size_t size)
{
  g_robotapp_heap_calloc_calls += 1U;
  if (robotapp::platform::heap_guard_started())
  {
    g_robotapp_heap_calls_after_start += 1U;
    robotapp::platform::heap_guard_trap();
  }
  void* p = __real_calloc(n, size);
  if (p == nullptr) g_robotapp_heap_alloc_fail += 1U;
  return p;
}

void* __wrap_realloc(void* ptr, size_t size)
{
  g_robotapp_heap_realloc_calls += 1U;
  if (robotapp::platform::heap_guard_started())
  {
    g_robotapp_heap_calls_after_start += 1U;
    robotapp::platform::heap_guard_trap();
  }
  void* p = __real_realloc(ptr, size);
  if (p == nullptr) g_robotapp_heap_alloc_fail += 1U;
  return p;
}

}  // extern "C"

// Optional: override global new/delete for clearer behaviour on embedded targets.
void* operator new(std::size_t size)
{
  void* p = std::malloc(size);
#if defined(__cpp_exceptions)
  if (p == nullptr) throw std::bad_alloc();
#else
  if (p == nullptr)
  {
    g_robotapp_heap_alloc_fail += 1U;
    for (;;)
    {
    }
  }
#endif
  return p;
}

void* operator new[](std::size_t size) { return ::operator new(size); }

void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }

void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

void* operator new(std::size_t size, const std::nothrow_t&) noexcept { return std::malloc(size); }
void* operator new[](std::size_t size, const std::nothrow_t&) noexcept { return std::malloc(size); }
void operator delete(void* p, const std::nothrow_t&) noexcept { std::free(p); }
void operator delete[](void* p, const std::nothrow_t&) noexcept { std::free(p); }
