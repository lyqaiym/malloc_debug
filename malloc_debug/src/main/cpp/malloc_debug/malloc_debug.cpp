/*
 * Copyright (C) 2012 The Android Open Source Project
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <errno.h>
#include <inttypes.h>
#include <malloc.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/cdefs.h>
#include <sys/param.h>
#include <unistd.h>

#include <mutex>
#include <vector>

//#include <android-base/file.h>
#include <android-base/stringprintf.h>
#include <private/bionic_malloc_dispatch.h>
#include <private/MallocXmlElem.h>
#include <sys/system_properties.h>

#include <android-base/unique_fd.h>
#include <fcntl.h>
#include <sys/stat.h>

#include "Config.h"
#include "DebugData.h"
#include "backtrace.h"
#include "debug_disable.h"
#include "debug_log.h"
#include "malloc_debug.h"
#include "UnwindBacktrace.h"
#include "ReadFileToString.h"
#include <jni.h>

#define TAG "malloc_debug_cpp"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)

// ------------------------------------------------------------------------
// Global Data
// ------------------------------------------------------------------------
DebugData* g_debug;

bool* g_zygote_child;

const MallocDispatch* g_dispatch;
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// Use C style prototypes for all exported functions. This makes it easy
// to do dlsym lookups during libc initialization when malloc debug
// is enabled.
// ------------------------------------------------------------------------
__BEGIN_DECLS

bool debug_initialize(const MallocDispatch* malloc_dispatch, bool* malloc_zygote_child,
                      const char* options);
void debug_finalize();
void extralogLeaks(int minsize);
void debug_dump_heap(const char* file_name);
void debug_get_malloc_leak_info(uint8_t** info, size_t* overall_size, size_t* info_size,
                                size_t* total_memory, size_t* backtrace_size);
bool debug_write_malloc_leak_info(FILE* fp);
ssize_t debug_malloc_backtrace(void* pointer, uintptr_t* frames, size_t frame_count);
void debug_free_malloc_leak_info(uint8_t* info);
size_t debug_malloc_usable_size(void* pointer);
void* debug_malloc(size_t size);
void debug_free(void* pointer);
void* debug_aligned_alloc(size_t alignment, size_t size);
void* debug_memalign(size_t alignment, size_t bytes);
void* debug_realloc(void* pointer, size_t bytes);
void* debug_calloc(size_t nmemb, size_t bytes);
struct mallinfo debug_mallinfo();
int debug_mallopt(int param, int value);
int debug_malloc_info(int options, FILE* fp);
int debug_posix_memalign(void** memptr, size_t alignment, size_t size);
int debug_iterate(uintptr_t base, size_t size,
                  void (*callback)(uintptr_t base, size_t size, void* arg), void* arg);
void debug_malloc_disable();
void debug_malloc_enable();

#if defined(HAVE_DEPRECATED_MALLOC_FUNCS)
void* debug_pvalloc(size_t bytes);
void* debug_valloc(size_t size);
#endif

__END_DECLS
// ------------------------------------------------------------------------

class ScopedConcurrentLock {
 public:
  ScopedConcurrentLock() {
    pthread_rwlock_rdlock(&lock_);
  }
  ~ScopedConcurrentLock() {
    pthread_rwlock_unlock(&lock_);
  }

  static void Init() {
    pthread_rwlockattr_t attr;
    // Set the attribute so that when a write lock is pending, read locks are no
    // longer granted.
    pthread_rwlockattr_setkind_np(&attr, PTHREAD_RWLOCK_PREFER_WRITER_NONRECURSIVE_NP);
    pthread_rwlock_init(&lock_, &attr);
  }

  static void BlockAllOperations() {
    pthread_rwlock_wrlock(&lock_);
  }

 private:
  static pthread_rwlock_t lock_;
};
pthread_rwlock_t ScopedConcurrentLock::lock_;

static void InitAtfork() {
  static pthread_once_t atfork_init = PTHREAD_ONCE_INIT;
  pthread_once(&atfork_init, []() {
    pthread_atfork(
        []() {
          if (g_debug != nullptr) {
            g_debug->PrepareFork();
          }
        },
        []() {
          if (g_debug != nullptr) {
            g_debug->PostForkParent();
          }
        },
        []() {
          if (g_debug != nullptr) {
            g_debug->PostForkChild();
          }
        });
  });
}

void BacktraceAndLog() {
  if (g_debug->config().options() & BACKTRACE_FULL) {
    std::vector<uintptr_t> frames;
    std::vector<unwindstack::LocalFrameData> frames_info;
    if (!Unwind(&frames, &frames_info, 256)) {
      error_log("  Backtrace failed to get any frames.");
    } else {
      UnwindLog(frames_info);
    }
  } else {
    std::vector<uintptr_t> frames(256);
    size_t num_frames = backtrace_get(frames.data(), frames.size());
    if (num_frames == 0) {
      error_log("  Backtrace failed to get any frames.");
    } else {
      backtrace_log(frames.data(), num_frames);
    }
  }
}

static void LogError(const void* pointer, const char* error_str) {
  error_log(LOG_DIVIDER);
  error_log("+++ ALLOCATION %p %s", pointer, error_str);

  // If we are tracking already freed pointers, check to see if this is
  // one so we can print extra information.
  if (g_debug->config().options() & FREE_TRACK) {
    PointerData::LogFreeBacktrace(pointer);
  }

  error_log("Backtrace at time of failure:");
  BacktraceAndLog();
  error_log(LOG_DIVIDER);
  if (g_debug->config().options() & ABORT_ON_ERROR) {
    abort();
  }
}

static bool VerifyPointer(const void* pointer, const char* function_name) {
  if (g_debug->HeaderEnabled()) {
    Header* header = g_debug->GetHeader(pointer);
    if (header->tag != DEBUG_TAG) {
      std::string error_str;
      if (header->tag == DEBUG_FREE_TAG) {
        error_str = std::string("USED AFTER FREE (") + function_name + ")";
      } else {
        error_str = android::base::StringPrintf("HAS INVALID TAG %" PRIx32 " (%s)", header->tag,
                                                function_name);
      }
      LogError(pointer, error_str.c_str());
      return false;
    }
  }

//  if (g_debug->TrackPointers()) {
//    if (!PointerData::Exists(pointer)) {
//      std::string error_str(std::string("UNKNOWN POINTER (") + function_name + ")");
//      LogError(pointer, error_str.c_str());
//      return false;
//    }
//  }
  return true;
}

static size_t InternalMallocUsableSize(void* pointer) {
  if (g_debug->HeaderEnabled()) {
    return g_debug->GetHeader(pointer)->usable_size;
  } else {
    return g_dispatch->malloc_usable_size(pointer);
  }
}

static void* InitHeader(Header* header, void* orig_pointer, size_t size) {
  header->tag = DEBUG_TAG;
  header->orig_pointer = orig_pointer;
  header->size = size;
  header->usable_size = g_dispatch->malloc_usable_size(orig_pointer);
  if (header->usable_size == 0) {
    g_dispatch->free(orig_pointer);
    return nullptr;
  }
  header->usable_size -= g_debug->pointer_offset() + reinterpret_cast<uintptr_t>(header) -
                         reinterpret_cast<uintptr_t>(orig_pointer);

  if (g_debug->config().options() & FRONT_GUARD) {
    uint8_t* guard = g_debug->GetFrontGuard(header);
    memset(guard, g_debug->config().front_guard_value(), g_debug->config().front_guard_bytes());
  }

  if (g_debug->config().options() & REAR_GUARD) {
    uint8_t* guard = g_debug->GetRearGuard(header);
    memset(guard, g_debug->config().rear_guard_value(), g_debug->config().rear_guard_bytes());
    // If the rear guard is enabled, set the usable size to the exact size
    // of the allocation.
    header->usable_size = header->size;
  }

  return g_debug->GetPointer(header);
}

bool debug_initialize(const MallocDispatch* malloc_dispatch, bool* zygote_child,
                      const char* options) {
  if (zygote_child == nullptr || options == nullptr) {
    return false;
  }

  InitAtfork();

  g_zygote_child = zygote_child;

  g_dispatch = malloc_dispatch;

  if (!DebugDisableInitialize()) {
    return false;
  }

  DebugData* debug = new DebugData();
  if (!debug->Initialize(options)) {
    delete debug;
    DebugDisableFinalize();
    return false;
  }
  g_debug = debug;

  // Always enable the backtrace code since we will use it in a number
  // of different error cases.
  backtrace_startup();

  if (g_debug->config().options() & VERBOSE) {
    info_log("%s: malloc debug enabled", getprogname());
  }

  ScopedConcurrentLock::Init();

  return true;
}

void extralogLeaks(int minsize){
  if (g_debug->config().options() & LEAK_TRACK) {
    PointerData::LogLeaks(minsize);
  }
}

void debug_finalize() {
  if (g_debug == nullptr) {
    return;
  }

  // Make sure that there are no other threads doing debug allocations
  // before we kill everything.
  ScopedConcurrentLock::BlockAllOperations();

  // Turn off capturing allocations calls.
  DebugDisableSet(true);

  if (g_debug->config().options() & FREE_TRACK) {
    PointerData::VerifyAllFreed();
  }

  if (g_debug->config().options() & LEAK_TRACK) {
    PointerData::LogLeaks();
  }

  if ((g_debug->config().options() & BACKTRACE) && g_debug->config().backtrace_dump_on_exit()) {
    debug_dump_heap(android::base::StringPrintf("%s.%d.exit.txt",
                                                g_debug->config().backtrace_dump_prefix().c_str(),
                                                getpid()).c_str());
  }

  backtrace_shutdown();

  delete g_debug;
  g_debug = nullptr;

  DebugDisableFinalize();
}

void debug_get_malloc_leak_info(uint8_t** info, size_t* overall_size, size_t* info_size,
                                size_t* total_memory, size_t* backtrace_size) {
  LOGD("debug_get_malloc_leak_info start");
  ScopedConcurrentLock lock;

  ScopedDisableDebugCalls disable;

  // Verify the arguments.
  if (info == nullptr || overall_size == nullptr || info_size == nullptr || total_memory == nullptr ||
      backtrace_size == nullptr) {
    error_log("get_malloc_leak_info: At least one invalid parameter.");
    return;
  }

  *info = nullptr;
  *overall_size = 0;
  *info_size = 0;
  *total_memory = 0;
  *backtrace_size = 0;

  if (!(g_debug->config().options() & BACKTRACE)) {
    error_log(
        "get_malloc_leak_info: Allocations not being tracked, to enable "
        "set the option 'backtrace'.");
    return;
  }
  LOGD("debug_get_malloc_leak_info GetInfo");
  PointerData::GetInfo(info, overall_size, info_size, total_memory, backtrace_size);
  LOGD("debug_get_malloc_leak_info end");
}

void debug_free_malloc_leak_info(uint8_t* info) {
  g_dispatch->free(info);
}

size_t debug_malloc_usable_size(void* pointer) {
  if (DebugCallsDisabled() || pointer == nullptr) {
    return g_dispatch->malloc_usable_size(pointer);
  }
  ScopedConcurrentLock lock;
  ScopedDisableDebugCalls disable;

  if (!VerifyPointer(pointer, "malloc_usable_size")) {
    return 0;
  }

  return InternalMallocUsableSize(pointer);
}

static void* InternalMalloc(size_t size) {
//  LOGD("InternalMalloc:size=%d",size);
  if ((g_debug->config().options() & BACKTRACE) && g_debug->pointer->ShouldDumpAndReset()) {
    debug_dump_heap(android::base::StringPrintf(
                        "%s.%d.txt", g_debug->config().backtrace_dump_prefix().c_str(), getpid())
                        .c_str());
  }

  if (size == 0) {
    size = 1;
  }
//  LOGD("InternalMalloc:g_debug=%p",g_debug);
  size_t real_size = size + g_debug->extra_bytes();
  if (real_size < size) {
    // Overflow.
    errno = ENOMEM;
    LOGD("InternalMalloc:errno=ENOMEM");
    return nullptr;
  }

  if (size > PointerInfoType::MaxSize()) {
    errno = ENOMEM;
    return nullptr;
  }

  void* pointer;
  if (g_debug->HeaderEnabled()) {
    Header* header =
        reinterpret_cast<Header*>(g_dispatch->memalign(MINIMUM_ALIGNMENT_BYTES, real_size));
      LOGD("InternalMalloc:header=%p",header);
    if (header == nullptr) {
      return nullptr;
    }
    pointer = InitHeader(header, header, size);
//    LOGD("InternalMalloc:header_pointer=%p",header);
  } else {
//      LOGD("InternalMalloc:g_dispatch->malloc=%p",g_dispatch->malloc);
      pointer = g_dispatch->malloc(real_size);
  }

  if (pointer != nullptr) {
    if (g_debug->TrackPointers()) {
      PointerData::Add(pointer, size);
    }

    if (g_debug->config().options() & FILL_ON_ALLOC) {
      size_t bytes = InternalMallocUsableSize(pointer);
      size_t fill_bytes = g_debug->config().fill_on_alloc_bytes();
      bytes = (bytes < fill_bytes) ? bytes : fill_bytes;
      memset(pointer, g_debug->config().fill_alloc_value(), bytes);
    }
  }
//  LOGD("InternalMalloc:pointer=%p",pointer);
  return pointer;
}

void* debug_malloc(size_t size) {
//  LOGD("debug_malloc:size=%d",size);
  if (DebugCallsDisabled()) {
    return g_dispatch->malloc(size);
  }
  ScopedConcurrentLock lock;
  ScopedDisableDebugCalls disable;

  void* pointer = InternalMalloc(size);

  if (g_debug->config().options() & RECORD_ALLOCS) {
    g_debug->record->AddEntry(new MallocEntry(pointer, size));
  }

  return pointer;
}

static void InternalFree(void* pointer) {
  if ((g_debug->config().options() & BACKTRACE) && g_debug->pointer->ShouldDumpAndReset()) {
    debug_dump_heap(android::base::StringPrintf(
                        "%s.%d.txt", g_debug->config().backtrace_dump_prefix().c_str(), getpid())
                        .c_str());
  }

  void* free_pointer = pointer;
  size_t bytes;
  Header* header;
  if (g_debug->HeaderEnabled()) {
    header = g_debug->GetHeader(pointer);
    free_pointer = header->orig_pointer;

    if (g_debug->config().options() & FRONT_GUARD) {
      if (!g_debug->front_guard->Valid(header)) {
        g_debug->front_guard->LogFailure(header);
      }
    }
    if (g_debug->config().options() & REAR_GUARD) {
      if (!g_debug->rear_guard->Valid(header)) {
        g_debug->rear_guard->LogFailure(header);
      }
    }

    header->tag = DEBUG_FREE_TAG;

    bytes = header->usable_size;
  } else {
    bytes = g_dispatch->malloc_usable_size(pointer);
  }

  if (g_debug->config().options() & FILL_ON_FREE) {
    size_t fill_bytes = g_debug->config().fill_on_free_bytes();
    bytes = (bytes < fill_bytes) ? bytes : fill_bytes;
    memset(pointer, g_debug->config().fill_free_value(), bytes);
  }

  if (g_debug->TrackPointers()) {
    PointerData::Remove(pointer);
  }

  if (g_debug->config().options() & FREE_TRACK) {
    // Do not add the allocation until we are done modifying the pointer
    // itself. This avoids a race if a lot of threads are all doing
    // frees at the same time and we wind up trying to really free this
    // pointer from another thread, while still trying to free it in
    // this function.
    pointer = PointerData::AddFreed(pointer);
    if (pointer != nullptr) {
      if (g_debug->HeaderEnabled()) {
        pointer = g_debug->GetHeader(pointer)->orig_pointer;
      }
      g_dispatch->free(pointer);
    }
  } else {
    g_dispatch->free(free_pointer);
  }
}

void debug_free(void* pointer) {
  if (DebugCallsDisabled() || pointer == nullptr) {
    return g_dispatch->free(pointer);
  }
  ScopedConcurrentLock lock;
  ScopedDisableDebugCalls disable;

  if (g_debug->config().options() & RECORD_ALLOCS) {
    g_debug->record->AddEntry(new FreeEntry(pointer));
  }

  if (!VerifyPointer(pointer, "free")) {
    return;
  }

  InternalFree(pointer);
}

void* debug_memalign(size_t alignment, size_t bytes) {
  if (DebugCallsDisabled()) {
    return g_dispatch->memalign(alignment, bytes);
  }
  ScopedConcurrentLock lock;
  ScopedDisableDebugCalls disable;

  if (bytes == 0) {
    bytes = 1;
  }

  if (bytes > PointerInfoType::MaxSize()) {
    errno = ENOMEM;
    return nullptr;
  }

  void* pointer;
  if (g_debug->HeaderEnabled()) {
    // Make the alignment a power of two.
    if (!powerof2(alignment)) {
      alignment = BIONIC_ROUND_UP_POWER_OF_2(alignment);
    }
    // Force the alignment to at least MINIMUM_ALIGNMENT_BYTES to guarantee
    // that the header is aligned properly.
    if (alignment < MINIMUM_ALIGNMENT_BYTES) {
      alignment = MINIMUM_ALIGNMENT_BYTES;
    }

    // We don't have any idea what the natural alignment of
    // the underlying native allocator is, so we always need to
    // over allocate.
    size_t real_size = alignment + bytes + g_debug->extra_bytes();
    if (real_size < bytes) {
      // Overflow.
      errno = ENOMEM;
      return nullptr;
    }

    pointer = g_dispatch->malloc(real_size);
    if (pointer == nullptr) {
      return nullptr;
    }

    uintptr_t value = reinterpret_cast<uintptr_t>(pointer) + g_debug->pointer_offset();
    // Now align the pointer.
    value += (-value % alignment);

    Header* header = g_debug->GetHeader(reinterpret_cast<void*>(value));
    pointer = InitHeader(header, pointer, bytes);
  } else {
    size_t real_size = bytes + g_debug->extra_bytes();
    if (real_size < bytes) {
      // Overflow.
      errno = ENOMEM;
      return nullptr;
    }
    pointer = g_dispatch->memalign(alignment, real_size);
  }

  if (pointer != nullptr) {
    if (g_debug->TrackPointers()) {
      PointerData::Add(pointer, bytes);
    }

    if (g_debug->config().options() & FILL_ON_ALLOC) {
      size_t bytes = InternalMallocUsableSize(pointer);
      size_t fill_bytes = g_debug->config().fill_on_alloc_bytes();
      bytes = (bytes < fill_bytes) ? bytes : fill_bytes;
      memset(pointer, g_debug->config().fill_alloc_value(), bytes);
    }

    if (g_debug->config().options() & RECORD_ALLOCS) {
      g_debug->record->AddEntry(new MemalignEntry(pointer, bytes, alignment));
    }
  }

  return pointer;
}

void* debug_realloc(void* pointer, size_t bytes) {
//  LOGD("debug_realloc start,realloc=%p,malloc_usable_size=%p",g_dispatch->realloc,g_dispatch->malloc_usable_size);
  if (DebugCallsDisabled()) {
    return g_dispatch->realloc(pointer, bytes);
  }
  ScopedConcurrentLock lock;
  ScopedDisableDebugCalls disable;

  if (pointer == nullptr) {
    pointer = InternalMalloc(bytes);
    if (g_debug->config().options() & RECORD_ALLOCS) {
      g_debug->record->AddEntry(new ReallocEntry(pointer, bytes, nullptr));
    }
    return pointer;
  }
//  LOGD("debug_realloc 2");
  if (!VerifyPointer(pointer, "realloc")) {
    return nullptr;
  }

  if (bytes == 0) {
    if (g_debug->config().options() & RECORD_ALLOCS) {
      g_debug->record->AddEntry(new ReallocEntry(nullptr, bytes, pointer));
    }

    InternalFree(pointer);
    return nullptr;
  }
//    LOGD("debug_realloc 3");
  size_t real_size = bytes;
  if (g_debug->config().options() & EXPAND_ALLOC) {
    real_size += g_debug->config().expand_alloc_bytes();
    if (real_size < bytes) {
      // Overflow.
      errno = ENOMEM;
      return nullptr;
    }
  }
//    LOGD("debug_realloc 4");
  if (bytes > PointerInfoType::MaxSize()) {
    errno = ENOMEM;
    return nullptr;
  }

  void* new_pointer;
  size_t prev_size;
  if (g_debug->HeaderEnabled()) {
    // Same size, do nothing.
    Header* header = g_debug->GetHeader(pointer);
    if (real_size == header->size) {
      if (g_debug->TrackPointers()) {
        // Remove and re-add so that the backtrace is updated.
        PointerData::Remove(pointer);
        PointerData::Add(pointer, real_size);
      }
      return pointer;
    }
//      LOGD("debug_realloc 5");
    // Allocation is shrinking.
    if (real_size < header->usable_size) {
      header->size = real_size;
      if (g_debug->config().options() & REAR_GUARD) {
        // Don't bother allocating a smaller pointer in this case, simply
        // change the header usable_size and reset the rear guard.
        header->usable_size = header->size;
        memset(g_debug->GetRearGuard(header), g_debug->config().rear_guard_value(),
               g_debug->config().rear_guard_bytes());
      }
      if (g_debug->TrackPointers()) {
        // Remove and re-add so that the backtrace is updated.
        PointerData::Remove(pointer);
        PointerData::Add(pointer, real_size);
      }
      return pointer;
    }

//    LOGD("debug_realloc 6");
    // Allocate the new size.
    new_pointer = InternalMalloc(bytes);
    if (new_pointer == nullptr) {
      errno = ENOMEM;
      return nullptr;
    }

    prev_size = header->usable_size;
    memcpy(new_pointer, pointer, prev_size);
    InternalFree(pointer);
  } else {
    if (g_debug->TrackPointers()) {
      PointerData::Remove(pointer);
    }

    prev_size = g_dispatch->malloc_usable_size(pointer);
    new_pointer = g_dispatch->realloc(pointer, real_size);
    if (new_pointer == nullptr) {
      return nullptr;
    }

    if (g_debug->TrackPointers()) {
      PointerData::Add(new_pointer, real_size);
    }
  }

  if (g_debug->config().options() & FILL_ON_ALLOC) {
    size_t bytes = InternalMallocUsableSize(new_pointer);
    if (bytes > g_debug->config().fill_on_alloc_bytes()) {
      bytes = g_debug->config().fill_on_alloc_bytes();
    }
    if (bytes > prev_size) {
      memset(reinterpret_cast<void*>(reinterpret_cast<uintptr_t>(new_pointer) + prev_size),
             g_debug->config().fill_alloc_value(), bytes - prev_size);
    }
  }

  if (g_debug->config().options() & RECORD_ALLOCS) {
    g_debug->record->AddEntry(new ReallocEntry(new_pointer, bytes, pointer));
  }
//  LOGD("debug_realloc end");
  return new_pointer;
}

void* debug_calloc(size_t nmemb, size_t bytes) {
  if (DebugCallsDisabled()) {
    return g_dispatch->calloc(nmemb, bytes);
  }
  ScopedConcurrentLock lock;
  ScopedDisableDebugCalls disable;

  size_t size;
  if (__builtin_mul_overflow(nmemb, bytes, &size)) {
    // Overflow
    errno = ENOMEM;
    return nullptr;
  }

  if (size == 0) {
    size = 1;
  }

  size_t real_size;
  if (__builtin_add_overflow(size, g_debug->extra_bytes(), &real_size)) {
    // Overflow.
    errno = ENOMEM;
    return nullptr;
  }

  if (real_size > PointerInfoType::MaxSize()) {
    errno = ENOMEM;
    return nullptr;
  }

  void* pointer;
  if (g_debug->HeaderEnabled()) {
    // Need to guarantee the alignment of the header.
    Header* header =
        reinterpret_cast<Header*>(g_dispatch->memalign(MINIMUM_ALIGNMENT_BYTES, real_size));
    if (header == nullptr) {
      return nullptr;
    }
    memset(header, 0, g_dispatch->malloc_usable_size(header));
    pointer = InitHeader(header, header, size);
  } else {
    pointer = g_dispatch->calloc(1, real_size);
  }

  if (g_debug->config().options() & RECORD_ALLOCS) {
    g_debug->record->AddEntry(new CallocEntry(pointer, bytes, nmemb));
  }

  if (pointer != nullptr && g_debug->TrackPointers()) {
    PointerData::Add(pointer, size);
  }
  return pointer;
}

struct mallinfo debug_mallinfo() {
  return g_dispatch->mallinfo();
}

int debug_mallopt(int param, int value) {
  return g_dispatch->mallopt(param, value);
}

int debug_malloc_info(int options, FILE* fp) {
  if (DebugCallsDisabled() || !g_debug->TrackPointers()) {
    return g_dispatch->malloc_info(options, fp);
  }
  ScopedConcurrentLock lock;
  ScopedDisableDebugCalls disable;

  MallocXmlElem root(fp, "malloc", "version=\"debug-malloc-1\"");
  std::vector<ListInfoType> list;
  PointerData::GetAllocList(&list);

  size_t alloc_num = 0;
  for (size_t i = 0; i < list.size(); i++) {
    MallocXmlElem alloc(fp, "allocation", "nr=\"%zu\"", alloc_num);

    size_t total = 1;
    size_t size = list[i].size;
    while (i < list.size() - 1 && list[i + 1].size == size) {
      i++;
      total++;
    }
    MallocXmlElem(fp, "size").Contents("%zu", list[i].size);
    MallocXmlElem(fp, "total").Contents("%zu", total);
    alloc_num++;
  }
  return 0;
}

void* debug_aligned_alloc(size_t alignment, size_t size) {
  if (DebugCallsDisabled()) {
    return g_dispatch->aligned_alloc(alignment, size);
  }
  if (!powerof2(alignment) || (size % alignment) != 0) {
    errno = EINVAL;
    return nullptr;
  }
  return debug_memalign(alignment, size);
}

int debug_posix_memalign(void** memptr, size_t alignment, size_t size) {
  if (DebugCallsDisabled()) {
    return g_dispatch->posix_memalign(memptr, alignment, size);
  }

  if (alignment < sizeof(void*) || !powerof2(alignment)) {
    return EINVAL;
  }
  int saved_errno = errno;
  *memptr = debug_memalign(alignment, size);
  errno = saved_errno;
  return (*memptr != nullptr) ? 0 : ENOMEM;
}

int debug_iterate(uintptr_t base, size_t size, void (*callback)(uintptr_t, size_t, void*),
                  void* arg) {
  ScopedConcurrentLock lock;
  if (g_debug->TrackPointers()) {
    // Since malloc is disabled, don't bother acquiring any locks.
    for (auto it = PointerData::begin(); it != PointerData::end(); ++it) {
      callback(it->first, InternalMallocUsableSize(reinterpret_cast<void*>(it->first)), arg);
    }
    return 0;
  }

  // An option that adds a header will add pointer tracking, so no need to
  // check if headers are enabled.
  return g_dispatch->iterate(base, size, callback, arg);
}

void debug_malloc_disable() {
  ScopedConcurrentLock lock;
  g_dispatch->malloc_disable();
  if (g_debug->pointer) {
    g_debug->pointer->PrepareFork();
  }
}

void debug_malloc_enable() {
  ScopedConcurrentLock lock;
  if (g_debug->pointer) {
    g_debug->pointer->PostForkParent();
  }
  g_dispatch->malloc_enable();
}

ssize_t debug_malloc_backtrace(void* pointer, uintptr_t* frames, size_t max_frames) {
  if (DebugCallsDisabled() || pointer == nullptr) {
    return 0;
  }
  ScopedConcurrentLock lock;
  ScopedDisableDebugCalls disable;

  if (!(g_debug->config().options() & BACKTRACE)) {
    return 0;
  }
  return PointerData::GetFrames(pointer, frames, max_frames);
}

#if defined(HAVE_DEPRECATED_MALLOC_FUNCS)
void* debug_pvalloc(size_t bytes) {
  if (DebugCallsDisabled()) {
    return g_dispatch->pvalloc(bytes);
  }

  size_t pagesize = getpagesize();
  size_t size = __BIONIC_ALIGN(bytes, pagesize);
  if (size < bytes) {
    // Overflow
    errno = ENOMEM;
    return nullptr;
  }
  return debug_memalign(pagesize, size);
}

void* debug_valloc(size_t size) {
  if (DebugCallsDisabled()) {
    return g_dispatch->valloc(size);
  }
  return debug_memalign(getpagesize(), size);
}
#endif

static std::mutex g_dump_lock;

std::string GetProperty(const std::string& key, const std::string& default_value) {
  std::string property_value;
#if defined(__BIONIC__)
  const prop_info* pi = __system_property_find(key.c_str());
  if (pi == nullptr) return default_value;

//  __system_property_read_callback(pi,
//                                  [](void* cookie, const char*, const char* value, unsigned) {
//                                    auto property_value = reinterpret_cast<std::string*>(cookie);
//                                    *property_value = value;
//                                  },
//                                  &property_value);
#else
  auto it = g_properties.find(key);
  if (it == g_properties.end()) return default_value;
  property_value = it->second;
#endif
  // If the property exists but is empty, also return the default value.
  // Since we can't remove system properties, "empty" is traditionally
  // the same as "missing" (this was true for cutils' property_get).
  return property_value.empty() ? default_value : property_value;
}

static void write_dump(FILE* fp) {
  fprintf(fp, "Android Native Heap Dump v1.2\n\n");

  std::string fingerprint = GetProperty("ro.build.fingerprint", "unknown");
  fprintf(fp, "Build fingerprint: '%s'\n\n", fingerprint.c_str());

  PointerData::DumpLiveToFile(fp);

  fprintf(fp, "MAPS\n");
  std::string content;
  if (!ReadFileToString("/proc/self/maps", &content, false)) {
    fprintf(fp, "Could not open /proc/self/maps\n");
  } else {
    fprintf(fp, "%s", content.c_str());
  }
  fprintf(fp, "END\n");
}

bool debug_write_malloc_leak_info(FILE* fp) {
  ScopedConcurrentLock lock;
  ScopedDisableDebugCalls disable;

  std::lock_guard<std::mutex> guard(g_dump_lock);

  if (!(g_debug->config().options() & BACKTRACE)) {
    return false;
  }

  write_dump(fp);
  return true;
}

void debug_dump_heap(const char* file_name) {
  ScopedConcurrentLock lock;
  ScopedDisableDebugCalls disable;

  std::lock_guard<std::mutex> guard(g_dump_lock);

  FILE* fp = fopen(file_name, "w+e");
  if (fp == nullptr) {
    error_log("Unable to create file: %s", file_name);
    return;
  }

  error_log("Dumping to file: %s\n", file_name);
  write_dump(fp);
  fclose(fp);
}

bool debuginit;
const char *save;

extern "C" JNIEXPORT void JNICALL
Java_com_malloc_test_MallocInit_init(JNIEnv *env, jclass clazz,jstring path) {
  struct MallocDispatch *malloc_dispatch = static_cast<MallocDispatch *>(malloc(
          sizeof(struct MallocDispatch)));

  malloc_dispatch->calloc = &calloc;
  malloc_dispatch->free = &free;
  malloc_dispatch->malloc = &malloc;
  malloc_dispatch->realloc = &realloc;
  LOGD("debug_initialize:realloc=%p,%p", malloc_dispatch->realloc, &realloc);
  malloc_dispatch->memalign = &memalign;
  malloc_dispatch->posix_memalign = &posix_memalign;
  malloc_dispatch->malloc_usable_size = &malloc_usable_size;
//    malloc_dispatchmy->aligned_alloc = &aligned_alloc;

//    memcpy(malloc_dispatch,malloc_dispatchmy,sizeof(struct MallocDispatchMy));
//    memset(malloc_dispatch,0,sizeof(struct MallocDispatchMy));
//    malloc_dispatch->malloc = &malloc;
  bool zygote_child = true;
  char *options = static_cast<char *>(malloc(100));
//    sprintf(options, "libc.debug.malloc.options %s", "backtrace");
//    sprintf(options, "LIBC_DEBUG_MALLOC_OPTIONS=%s", "backtrace");
//    sprintf(options, "LIBC_DEBUG_MALLOC_OPTIONS %s", "backtrace");
//    sprintf(options, "libc.debug.malloc.options=%s", "1");
//    sprintf(options, "libc.debug.malloc.options=%d", 1);
//    sprintf(options, "libc.debug.malloc.env_enabled %s", "1");
//    sprintf(options, "libc.debug.malloc.program %s", "1");
//    sprintf(options, "%s", "backtrace");
  sprintf(options, "%s", "leak_track backtrace backtrace_full record_allocs");
//    sprintf(options, "%s", "verbose");
//    sprintf(options, "libc.debug.malloc %d", 1);
//    sprintf(options, "libc.debug.malloc %d", 1);
//    sprintf(options, "%s", "backtrace=4 backtrace_dump_on_exit");
  options[84] = '\0';
  LOGD("debug_initialize:options=%s", options);
  debuginit = debug_initialize(malloc_dispatch, &zygote_child, options);
  LOGD("debug_initialize:debuginit=%d", debuginit);
  if (debuginit) {
    save = env->GetStringUTFChars(path, nullptr);
  }
}

void test4(){
    void* p = debug_malloc(11);
//    debug_free(p);
}

void test3(){
    test4();
}

void test2(){
    test3();
}

void test1(){
    test2();
}

extern "C"
JNIEXPORT void JNICALL
Java_com_malloc_test_MallocInit_test(JNIEnv *env, jclass clazz) {
    test1();
}

extern "C"
JNIEXPORT void JNICALL
Java_com_malloc_test_MallocInit_printf(JNIEnv *env, jclass clazz,jstring path) {
    uint8_t *info = 0;
    size_t overall_size;
    size_t info_size;
    size_t total_memory;
    size_t backtrace_size;
    debug_get_malloc_leak_info(&info, &overall_size, &info_size, &total_memory, &backtrace_size);
    LOGD("MallocInit_stop:info=%p", &info);
    const char *c = env->GetStringUTFChars(path, nullptr);
    LOGD("MallocInit_stop:c=%s", c);
    debug_dump_heap(c);
//    extralogLeaks(1);
}