#pragma once

#include <cstddef>
#include <cstdint>

namespace robotapp::platform {

// Marks "scheduler started" from any task context. After this, any heap operation
// (malloc/free/new/...) is considered a design bug and will trap.
void heap_guard_mark_started() noexcept;

// Best-effort indicator used by malloc/new wrappers.
bool heap_guard_started() noexcept;

}  // namespace robotapp::platform

extern "C" {

// LiveWatch-friendly counters.
extern volatile uint32_t g_robotapp_heap_started;
extern volatile uint32_t g_robotapp_heap_malloc_calls;
extern volatile uint32_t g_robotapp_heap_free_calls;
extern volatile uint32_t g_robotapp_heap_calloc_calls;
extern volatile uint32_t g_robotapp_heap_realloc_calls;
extern volatile uint32_t g_robotapp_heap_calls_after_start;
extern volatile uint32_t g_robotapp_heap_alloc_fail;

}

