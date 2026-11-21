/**
 * @file embedded_demo.cpp
 * @brief Embedded Firmware Architecture Demo in C++20: Ring Buffer,
 *        Sensor Filter, Fault State Machine, Watchdog, and Scheduler.
 */

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string_view>

namespace embedded_demo {

/* =========================================================================
 * 1. HARDWARE / PLATFORM CONFIGURATION CONSTANTS
 * ========================================================================= */

inline constexpr std::uint32_t SYS_TICK_HZ              = 1000U; /* 1 ms tick */
inline constexpr std::size_t   UART_RX_BUF_SIZE         = 64U;
inline constexpr std::size_t   UART_TX_BUF_SIZE         = 128U;
inline constexpr std::uint16_t ADC_MAX_COUNTS           = 4095U; /* 12-bit ADC */
inline constexpr std::uint32_t ADC_VREF_MV              = 3300U; /* 3.300 V */
inline constexpr std::size_t   SENSOR_FILTER_WINDOW     = 8U;
inline constexpr std::uint32_t SENSOR_LOW_THRESHOLD_MV  = 200U;
inline constexpr std::uint32_t SENSOR_HIGH_THRESHOLD_MV = 3100U;
inline constexpr std::uint32_t WATCHDOG_TIMEOUT_MS      = 250U;
inline constexpr std::size_t   MAX_TASKS                = 6U;
inline constexpr std::uint32_t DEMO_SIMULATION_TICKS    = 1200U;

/* =========================================================================
 * 2. RING BUFFER (CIRCULAR FIFO) FOR BYTE STREAMS
 * ========================================================================= */

template <std::size_t Capacity>
class RingBuffer {
    static_assert(Capacity >= 2U, "RingBuffer capacity must be at least 2");

public:
    RingBuffer() noexcept { reset(); }

    void reset() noexcept
    {
        head_ = 0U;
        tail_ = 0U;
        drop_count_ = 0U;
    }

    [[nodiscard]] std::size_t count() const noexcept
    {
        if (head_ >= tail_) {
            return head_ - tail_;
        }
        return (Capacity - tail_) + head_;
    }

    [[nodiscard]] bool is_empty() const noexcept { return head_ == tail_; }

    [[nodiscard]] bool is_full() const noexcept
    {
        return next_index(head_) == tail_;
    }

    [[nodiscard]] std::uint32_t drop_count() const noexcept { return drop_count_; }

    bool push(std::uint8_t byte) noexcept
    {
        const std::size_t next = next_index(head_);
        if (next == tail_) {
            ++drop_count_;
            return false;
        }
        buffer_[head_] = byte;
        head_ = next;
        return true;
    }

    bool pop(std::uint8_t& out_byte) noexcept
    {
        if (is_empty()) {
            return false;
        }
        out_byte = buffer_[tail_];
        tail_ = next_index(tail_);
        return true;
    }

    std::size_t write(const std::uint8_t* data, std::size_t len) noexcept
    {
        std::size_t written = 0U;
        for (std::size_t i = 0U; i < len; ++i) {
            if (push(data[i])) {
                ++written;
            } else {
                break;
            }
        }
        return written;
    }

    std::size_t write_str(std::string_view sv) noexcept
    {
        return write(reinterpret_cast<const std::uint8_t*>(sv.data()), sv.size());
    }

private:
    static constexpr std::size_t next_index(std::size_t idx) noexcept
    {
        return (idx + 1U) % Capacity;
    }

    std::array<std::uint8_t, Capacity> buffer_{};
    volatile std::size_t head_{0U};
    volatile std::size_t tail_{0U};
    volatile std::uint32_t drop_count_{0U};
};

/* =========================================================================
 * 3. SENSOR ACQUISITION, FIXED-POINT FILTER & FAULT DETECTOR
 * ========================================================================= */

enum class SensorStatus : std::uint32_t {
    Ok = 0U,
    FaultUndervoltage = 0x02U,
    FaultOvervoltage  = 0x04U,
    FaultStale        = 0x08U
};

class AnalogSensor {
public:
    explicit AnalogSensor(std::uint32_t now_tick = 0U) noexcept
        : last_update_tick_(now_tick)
    {
    }

    static constexpr std::uint32_t counts_to_mv(std::uint16_t counts) noexcept
    {
        const std::uint16_t clamped =
            (counts > ADC_MAX_COUNTS) ? ADC_MAX_COUNTS : counts;
        return (static_cast<std::uint32_t>(clamped) * ADC_VREF_MV) / ADC_MAX_COUNTS;
    }

    void update_sample(std::uint16_t raw_counts, std::uint32_t now_tick) noexcept
    {
        raw_counts_ = raw_counts;
        raw_mv_ = counts_to_mv(raw_counts);
        last_update_tick_ = now_tick;

        if (sample_count_ == SENSOR_FILTER_WINDOW) {
            accumulator_ -= samples_[sample_idx_];
        } else {
            ++sample_count_;
        }

        samples_[sample_idx_] = raw_mv_;
        accumulator_ += raw_mv_;
        sample_idx_ = (sample_idx_ + 1U) % SENSOR_FILTER_WINDOW;
        filtered_mv_ = accumulator_ / static_cast<std::uint32_t>(sample_count_);

        const bool low_fault  = (filtered_mv_ < SENSOR_LOW_THRESHOLD_MV);
        const bool high_fault = (filtered_mv_ > SENSOR_HIGH_THRESHOLD_MV);

        if (low_fault || high_fault) {
            if (fault_debounce_cnt_ < 5U) {
                ++fault_debounce_cnt_;
            }
            if (fault_debounce_cnt_ >= 3U) {
                status_ = low_fault ? SensorStatus::FaultUndervoltage
                                    : SensorStatus::FaultOvervoltage;
            }
        } else {
            if (fault_debounce_cnt_ > 0U) {
                --fault_debounce_cnt_;
            }
            if (fault_debounce_cnt_ == 0U && status_ != SensorStatus::FaultStale) {
                status_ = SensorStatus::Ok;
            }
        }
    }

    void check_staleness(std::uint32_t now_tick, std::uint32_t max_age_ms) noexcept
    {
        if ((now_tick - last_update_tick_) > max_age_ms) {
            status_ = SensorStatus::FaultStale;
        }
    }

    [[nodiscard]] std::uint16_t raw_counts() const noexcept { return raw_counts_; }
    [[nodiscard]] std::uint32_t raw_mv() const noexcept { return raw_mv_; }
    [[nodiscard]] std::uint32_t filtered_mv() const noexcept { return filtered_mv_; }
    [[nodiscard]] SensorStatus status() const noexcept { return status_; }

private:
    std::uint16_t raw_counts_{0U};
    std::uint32_t raw_mv_{0U};
    std::uint32_t filtered_mv_{0U};
    std::array<std::uint32_t, SENSOR_FILTER_WINDOW> samples_{};
    std::uint32_t accumulator_{0U};
    std::size_t sample_idx_{0U};
    std::size_t sample_count_{0U};
    std::uint32_t last_update_tick_{0U};
    std::uint16_t fault_debounce_cnt_{0U};
    SensorStatus status_{SensorStatus::Ok};
};

/* =========================================================================
 * 4. SYSTEM STATE MACHINE & WATCHDOG SUPERVISOR
 * ========================================================================= */

enum class SystemState : std::uint8_t {
    Boot = 0U,
    Calibrate,
    Run,
    Degraded,
    Fault
};

constexpr std::string_view to_string(SystemState state) noexcept
{
    switch (state) {
        case SystemState::Boot:      return "BOOT";
        case SystemState::Calibrate: return "CALIBRATE";
        case SystemState::Run:       return "RUN";
        case SystemState::Degraded:  return "DEGRADED";
        case SystemState::Fault:     return "FAULT";
        default:                     return "UNKNOWN";
    }
}

class SoftwareWatchdog {
public:
    SoftwareWatchdog(std::uint32_t timeout_ms, std::uint32_t now_tick) noexcept
        : timeout_ms_(timeout_ms), last_kick_tick_(now_tick)
    {
    }

    void kick(std::uint32_t now_tick) noexcept
    {
        last_kick_tick_ = now_tick;
        ++kick_count_;
    }

    bool check(std::uint32_t now_tick) noexcept
    {
        if ((now_tick - last_kick_tick_) >= timeout_ms_) {
            expired_ = true;
        }
        return expired_;
    }

    [[nodiscard]] std::uint32_t kick_count() const noexcept { return kick_count_; }
    [[nodiscard]] bool is_expired() const noexcept { return expired_; }

private:
    std::uint32_t timeout_ms_{WATCHDOG_TIMEOUT_MS};
    std::uint32_t last_kick_tick_{0U};
    std::uint32_t kick_count_{0U};
    bool expired_{false};
};

class SystemController {
public:
    explicit SystemController(std::uint32_t now_tick = 0U) noexcept
        : state_enter_tick_(now_tick),
          watchdog_(WATCHDOG_TIMEOUT_MS, now_tick)
    {
    }

    template <std::size_t TxCap>
    void step(const AnalogSensor& sensor, const RingBuffer<TxCap>& tx_buf,
              std::uint32_t now_tick) noexcept
    {
        if (watchdog_.check(now_tick)) {
            fault_code_ |= 0x01U;
            transition_to(SystemState::Fault, now_tick);
            return;
        }

        const SensorStatus s_stat = sensor.status();

        switch (state_) {
            case SystemState::Boot:
                if ((now_tick - state_enter_tick_) >= 50U) {
                    transition_to(SystemState::Calibrate, now_tick);
                }
                break;

            case SystemState::Calibrate:
                if ((now_tick - state_enter_tick_) >= 100U) {
                    calibration_offset_mv_ = (sensor.filtered_mv() > 1650U)
                                               ? (sensor.filtered_mv() - 1650U)
                                               : 0U;
                    transition_to(SystemState::Run, now_tick);
                }
                break;

            case SystemState::Run:
                if (s_stat == SensorStatus::FaultOvervoltage ||
                    s_stat == SensorStatus::FaultStale) {
                    fault_code_ |= static_cast<std::uint32_t>(s_stat);
                    transition_to(SystemState::Fault, now_tick);
                } else if (s_stat == SensorStatus::FaultUndervoltage ||
                           tx_buf.drop_count() > 0U) {
                    fault_code_ |= 0x10U;
                    transition_to(SystemState::Degraded, now_tick);
                }
                break;

            case SystemState::Degraded:
                if (s_stat == SensorStatus::FaultOvervoltage ||
                    s_stat == SensorStatus::FaultStale) {
                    fault_code_ |= static_cast<std::uint32_t>(s_stat);
                    transition_to(SystemState::Fault, now_tick);
                } else if (s_stat == SensorStatus::Ok &&
                           (now_tick - state_enter_tick_) >= 150U) {
                    fault_code_ &= ~0x10U;
                    transition_to(SystemState::Run, now_tick);
                }
                break;

            case SystemState::Fault:
                /* Safe hold state until external reset or fault clear */
                break;

            default:
                transition_to(SystemState::Fault, now_tick);
                break;
        }
    }

    void kick_watchdog_if_healthy(std::uint32_t now_tick) noexcept
    {
        if (state_ != SystemState::Fault) {
            watchdog_.kick(now_tick);
        }
    }

    [[nodiscard]] SystemState state() const noexcept { return state_; }
    [[nodiscard]] std::uint32_t fault_code() const noexcept { return fault_code_; }
    [[nodiscard]] std::uint32_t calibration_offset_mv() const noexcept
    {
        return calibration_offset_mv_;
    }
    [[nodiscard]] const SoftwareWatchdog& watchdog() const noexcept { return watchdog_; }

private:
    void transition_to(SystemState next, std::uint32_t now_tick) noexcept
    {
        if (state_ != next) {
            const SystemState prev = state_;
            state_ = next;
            state_enter_tick_ = now_tick;
            const std::string_view prev_sv = to_string(prev);
            const std::string_view next_sv = to_string(next);
            std::printf("[%04lu ms] STATE: %.*s -> %.*s (fault_code=0x%02lX)\n",
                        static_cast<unsigned long>(now_tick),
                        static_cast<int>(prev_sv.size()), prev_sv.data(),
                        static_cast<int>(next_sv.size()), next_sv.data(),
                        static_cast<unsigned long>(fault_code_));
        }
    }

    SystemState state_{SystemState::Boot};
    std::uint32_t state_enter_tick_{0U};
    std::uint32_t calibration_offset_mv_{0U};
    std::uint32_t fault_code_{0U};
    SoftwareWatchdog watchdog_;
};

/* =========================================================================
 * 5. COOPERATIVE TASK SCHEDULER
 * ========================================================================= */

using TaskCallback = void (*)(void* ctx, std::uint32_t now_tick);

struct ScheduledTask {
    std::string_view name{};
    TaskCallback fn{nullptr};
    void* ctx{nullptr};
    std::uint32_t period_ms{0U};
    std::uint32_t last_run_tick{0U};
    std::uint32_t run_count{0U};
    bool enabled{false};
};

class CooperativeScheduler {
public:
    bool add_task(std::string_view name, TaskCallback fn, void* ctx,
                  std::uint32_t period_ms, std::uint32_t start_tick) noexcept
    {
        if (task_count_ >= MAX_TASKS || fn == nullptr || period_ms == 0U) {
            return false;
        }
        tasks_[task_count_++] = ScheduledTask{
            .name = name,
            .fn = fn,
            .ctx = ctx,
            .period_ms = period_ms,
            .last_run_tick = start_tick,
            .run_count = 0U,
            .enabled = true,
        };
        return true;
    }

    void tick(std::uint32_t now_tick) noexcept
    {
        for (std::size_t i = 0U; i < task_count_; ++i) {
            ScheduledTask& t = tasks_[i];
            if (!t.enabled) {
                continue;
            }
            if ((now_tick - t.last_run_tick) >= t.period_ms) {
                t.last_run_tick = now_tick;
                ++t.run_count;
                t.fn(t.ctx, now_tick);
            }
        }
    }

private:
    std::array<ScheduledTask, MAX_TASKS> tasks_{};
    std::size_t task_count_{0U};
};

/* =========================================================================
 * 6. APPLICATION CONTEXT, TASKS & SIMULATED HARDWARE STUBS
 * ========================================================================= */

struct AppContext {
    std::uint32_t tick_ms{0U};
    RingBuffer<UART_RX_BUF_SIZE> rx_buf{};
    RingBuffer<UART_TX_BUF_SIZE> tx_buf{};
    AnalogSensor sensor{0U};
    SystemController sys{0U};
    CooperativeScheduler sched{};
};

static std::uint16_t hal_adc_read_sim(std::uint32_t tick_ms) noexcept
{
    if (tick_ms >= 450U && tick_ms <= 520U) {
        return 120U; /* ~96 mV undervoltage dip */
    }
    if (tick_ms >= 900U && tick_ms <= 940U) {
        return 4050U; /* ~3261 mV overvoltage spike */
    }
    const std::int32_t wave = static_cast<std::int32_t>(tick_ms % 100U) - 50;
    std::int32_t counts = 2048 + (wave * 6);
    if (counts < 0) {
        counts = 0;
    }
    if (counts > static_cast<std::int32_t>(ADC_MAX_COUNTS)) {
        counts = static_cast<std::int32_t>(ADC_MAX_COUNTS);
    }
    return static_cast<std::uint16_t>(counts);
}

template <std::size_t TxCap>
static void hal_uart_tx_drain_sim(RingBuffer<TxCap>& tx_buf) noexcept
{
    std::uint8_t ch = 0U;
    std::size_t drained = 0U;
    while (drained < 16U && tx_buf.pop(ch)) {
        ++drained;
    }
}

static void task_sensor_sample(void* ctx, std::uint32_t now_tick)
{
    auto* app = static_cast<AppContext*>(ctx);
    const std::uint16_t counts = hal_adc_read_sim(now_tick);
    app->sensor.update_sample(counts, now_tick);
}

static void task_supervisor_wdg(void* ctx, std::uint32_t now_tick)
{
    auto* app = static_cast<AppContext*>(ctx);
    app->sensor.check_staleness(now_tick, 100U);
    app->sys.step(app->sensor, app->tx_buf, now_tick);
    app->sys.kick_watchdog_if_healthy(now_tick);
}

static void task_telemetry_pack(void* ctx, std::uint32_t now_tick)
{
    auto* app = static_cast<AppContext*>(ctx);
    const std::string_view st = to_string(app->sys.state());
    char frame[96];
    const int len = std::snprintf(
        frame, sizeof(frame),
        "T=%lu,ST=%.*s,RAW=%lu,FILT=%lu,RX=%zu,TX=%zu\r\n",
        static_cast<unsigned long>(now_tick),
        static_cast<int>(st.size()), st.data(),
        static_cast<unsigned long>(app->sensor.raw_mv()),
        static_cast<unsigned long>(app->sensor.filtered_mv()),
        app->rx_buf.count(),
        app->tx_buf.count());
    if (len > 0) {
        (void)app->tx_buf.write_str(
            std::string_view(frame, static_cast<std::size_t>(len)));
    }
    hal_uart_tx_drain_sim(app->tx_buf);
}

static void task_cli_poll(void* ctx, std::uint32_t now_tick)
{
    auto* app = static_cast<AppContext*>(ctx);
    (void)now_tick;

    (void)app->rx_buf.push(static_cast<std::uint8_t>('?'));

    std::uint8_t byte = 0U;
    while (app->rx_buf.pop(byte)) {
        if (byte == static_cast<std::uint8_t>('?')) {
            (void)app->tx_buf.write_str("OK\r\n");
        }
    }
}

static void task_periodic_summary(void* ctx, std::uint32_t now_tick)
{
    auto* app = static_cast<AppContext*>(ctx);
    const std::string_view st = to_string(app->sys.state());
    std::printf("[%04lu ms] SUMMARY | state=%-9.*s | raw=%4lumV | filt=%4lumV | "
                "wdg_kicks=%lu | tx_drops=%lu\n",
                static_cast<unsigned long>(now_tick),
                static_cast<int>(st.size()), st.data(),
                static_cast<unsigned long>(app->sensor.raw_mv()),
                static_cast<unsigned long>(app->sensor.filtered_mv()),
                static_cast<unsigned long>(app->sys.watchdog().kick_count()),
                static_cast<unsigned long>(app->tx_buf.drop_count()));
}

/* =========================================================================
 * 7. ENTRY POINT & MAIN LOOP
 * ========================================================================= */

static void app_init(AppContext& app) noexcept
{
    app.tick_ms = 0U;
    app.rx_buf.reset();
    app.tx_buf.reset();
    app.sensor = AnalogSensor(app.tick_ms);
    app.sys = SystemController(app.tick_ms);
    app.sched = CooperativeScheduler{};

    (void)app.sched.add_task("sensor_10ms", task_sensor_sample,
                             &app, 10U, app.tick_ms);
    (void)app.sched.add_task("wdg_20ms", task_supervisor_wdg,
                             &app, 20U, app.tick_ms);
    (void)app.sched.add_task("cli_50ms", task_cli_poll,
                             &app, 50U, app.tick_ms);
    (void)app.sched.add_task("telem_100ms", task_telemetry_pack,
                             &app, 100U, app.tick_ms);
    (void)app.sched.add_task("summary_200ms", task_periodic_summary,
                             &app, 200U, app.tick_ms);
}

} /* namespace embedded_demo */

int main()
{
    using namespace embedded_demo;

    AppContext app{};
    app_init(app);

    std::printf("Starting C++20 Embedded Firmware Demo (%u ms simulation)...\n",
                DEMO_SIMULATION_TICKS);

    for (std::uint32_t t = 0U; t <= DEMO_SIMULATION_TICKS; ++t) {
        app.tick_ms = t;
        app.sched.tick(app.tick_ms);
    }

    const std::string_view final_st = to_string(app.sys.state());
    std::printf("Simulation complete. Final state: %.*s, WD kicks: %lu\n",
                static_cast<int>(final_st.size()), final_st.data(),
                static_cast<unsigned long>(app.sys.watchdog().kick_count()));

    return EXIT_SUCCESS;
}
