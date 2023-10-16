#define NOMINMAX
#include <Windows.h>
#include <x86intrin.h>

#include <cassert>
#include <cstdio>

using u64 = unsigned long long;

namespace profiling {
    static u64 read_os_timer_frequency() {
        LARGE_INTEGER frequency = {};
        const BOOL success = QueryPerformanceFrequency(&frequency);
        assert(success != 0);
        return static_cast<u64>(frequency.QuadPart);
    }

    static u64 read_os_timer() {
        LARGE_INTEGER tick_count = {};
        const BOOL success = QueryPerformanceCounter(&tick_count);
        assert(success != 0);
        return static_cast<u64>(tick_count.QuadPart);
    }

    static u64 read_cpu_timer() {
        return __rdtsc();
    }

    static u64 estimate_cpu_frequency() {
        const u64 os_timer_frequency = read_os_timer_frequency();

        const u64 cpu_start = read_cpu_timer();
        const u64 os_start = read_os_timer();

        u64 os_elapsed = 0;
        while (os_elapsed < os_timer_frequency) {
            const u64 os_end = read_os_timer();
            os_elapsed = os_end - os_start;
        }

        const u64 cpu_end = read_cpu_timer();
        const u64 cpu_elapsed = cpu_end - cpu_start;

        return os_timer_frequency * cpu_elapsed / os_elapsed;
    }

    struct ProfilingData {
        const char* name;
        u64 exclusive_duration; // does not include children
        u64 inclusive_duration; // does include children
        u64 hit_count;
    };

    static constexpr u64 PROFILING_DATA_COUNT = 4096;
    static ProfilingData PROFILING_DATA_ARRAY[PROFILING_DATA_COUNT] = {};
    static u64 CURRENT_PROFILING_DATA_INDEX = 0;

    struct BlockTimer {
        BlockTimer(const char* const timer_name, const u64 timer_index) {
            name = timer_name;
            index = timer_index;
            parent_index = CURRENT_PROFILING_DATA_INDEX;
            CURRENT_PROFILING_DATA_INDEX = index;
            old_inclusive_duration = PROFILING_DATA_ARRAY[index].inclusive_duration;
            start_time = read_cpu_timer();
        }

        ~BlockTimer() {
            const u64 end_time = read_cpu_timer();
            const u64 duration = end_time - start_time;

            ProfilingData& profiling_data = PROFILING_DATA_ARRAY[index];
            profiling_data.name = name;
            profiling_data.exclusive_duration += duration;
            profiling_data.inclusive_duration = old_inclusive_duration + duration;
            profiling_data.hit_count += 1;

            ProfilingData& parent_profiling_data = PROFILING_DATA_ARRAY[parent_index];
            parent_profiling_data.exclusive_duration -= duration;

            CURRENT_PROFILING_DATA_INDEX = parent_index;
        }

        const char* name;
        u64 index;
        u64 parent_index;
        u64 old_inclusive_duration;
        u64 start_time;
    };

    static u64 PROFILING_START_TIME = 0;

    static void initialise() {
        PROFILING_START_TIME = read_cpu_timer();
    }

    static void print_collected_data() {
        const u64 profiling_end_time = read_cpu_timer();
        const u64 cpu_frequency = estimate_cpu_frequency();

        const u64 profiling_duration = profiling_end_time - PROFILING_START_TIME;
        const double profiling_time = static_cast<double>(profiling_duration) / static_cast<double>(cpu_frequency) * 1000.0;

        std::printf("total time: %gms (cpu frequency: %llu)\n", profiling_time, cpu_frequency);

        for (u64 i = 1; i < PROFILING_DATA_COUNT; ++i) {
            const ProfilingData& profiling_data = PROFILING_DATA_ARRAY[i];
            const u64 exclusive_duration = profiling_data.exclusive_duration;
            if (exclusive_duration > 0) {
                const double percentage = static_cast<double>(exclusive_duration) / static_cast<double>(profiling_duration) * 100.0;
                std::printf("%s[%llu]: %llu, (%.2f%%", profiling_data.name, profiling_data.hit_count, exclusive_duration, percentage);

                const u64 inclusive_duration = profiling_data.inclusive_duration;
                if (inclusive_duration != exclusive_duration) {
                    const double with_children_percentage = static_cast<double>(inclusive_duration) / static_cast<double>(profiling_duration) * 100.0;
                    std::printf(", %.2f%% w/ children", with_children_percentage);
                }

                std::puts(")");
            }
        }
    }
}

#ifdef PROFILING

#define NAME_CONCAT2(s1, s2) s1##s2
#define NAME_CONCAT(s1, s2) NAME_CONCAT2(s1, s2)
#define TIME_BLOCK(block_name) const profiling::BlockTimer NAME_CONCAT(timer, __LINE__){block_name, __COUNTER__ + 1}

#else

#define TIME_BLOCK(block_name)

#endif
