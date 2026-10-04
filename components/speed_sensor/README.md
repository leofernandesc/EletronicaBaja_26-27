# Speed sensor

Reusable ESP-IDF component for a digital pulse input with a fixed relationship
to wheel travel. The intended prototype sensor is a VW Gol G5 Hall effect speed
sensor; no vehicle-specific pulse count or electrical pinout is assumed.

## API and independent instances

```c
#include "speed_sensor.h"

speed_sensor_config_t first_config = SPEED_SENSOR_CONFIG_DEFAULT();
first_config.gpio = GPIO_NUM_27;
first_config.distance_per_pulse_m = 0.0f; // Set after calibration.
speed_sensor_handle_t first = NULL;
esp_err_t err = speed_sensor_init(&first_config, &first);

speed_sensor_config_t second_config = SPEED_SENSOR_CONFIG_DEFAULT();
second_config.gpio = GPIO_NUM_26; // Example only; verify board assignments.
second_config.distance_per_pulse_m = 0.0f;
speed_sensor_handle_t second = NULL;
esp_err_t second_err = speed_sensor_init(&second_config, &second);

speed_sensor_reading_t reading;
if (err == ESP_OK && speed_sensor_read(first, &reading) == ESP_OK) {
    // Raw pulse_count, last_pulse_us and period_us are available even without
    // calibration. Use speed_mps only when reading.available is true.
    if (reading.available) {
        float speed_mps = reading.speed_mps;
        // Pass speed_mps to the application's logger or consumer.
        (void)speed_mps;
    }
}

// After stopping every reader of the handles, on the lifecycle task/core:
if (err == ESP_OK) speed_sensor_deinit(first);
if (second_err == ESP_OK) speed_sensor_deinit(second);
```

The component has no application-specific sensor count. Each instance uses an
exclusive GPIO, one allocated state object, and one handler in the shared GPIO
interrupt service. Available pins, memory, and aggregate interrupt load limit
the number of sensors. It creates no sensor tasks and uses no PCNT units.
Duplicate assignments within the component return `ESP_ERR_INVALID_STATE`;
the application must also prevent conflicts with other components.

Initialization and cleanup are serialized internally and must run in a task
pinned to the core that first initialized the component. ESP-IDF's default main
task is pinned and can create all instances. Reads may run from other tasks or
cores; their state snapshot is protected by a short critical section. Stop all
readers before deleting a handle. Handles become invalid after successful cleanup.

The first initialization installs the global GPIO ISR service with
`ESP_INTR_FLAG_IRAM`. A service already installed by another component is
rejected because its IRAM configuration cannot be verified. Initialize this
component first; subsequent GPIO clients may add their own handlers. The service
and lifecycle mutex remain installed after the last instance is removed. Other
clients must not uninstall the service while this component is in use.
`CONFIG_ESP_TIMER_IN_IRAM=y` is required and is already enabled in this project.
ISR code and instance state reside in internal memory. The ISR only records
integer timing/count data; float conversion runs in the reader.

## Calibration and timing

Define:

- `C`: distance travelled for one wheel revolution, in metres.
- `P`: selected-edge pulses per sensor shaft revolution.
- `R`: sensor shaft revolutions per wheel revolution.

Then `distance_per_pulse_m = C / (P * R)` and
`speed_mps = distance_per_pulse_m * 1,000,000 / period_us`.
Alternatively, calibrate by counting pulses over a measured travel distance:
`distance_per_pulse_m = measured_distance_m / pulse_count`.
Use one edge per pulse. Switching from rising to falling edge does not double
the pulse count. A single pulse input supplies speed magnitude, not direction.

The first instance's menuconfig distance is expressed in integer micrometres;
`main/main.c` converts it to metres. Zero leaves speed unavailable even after
the stop timeout; raw pulse data remains readable. Negative/nonfinite calibration,
zero timeout, unsupported edges, invalid GPIOs and a rejection interval at least
as long as the timeout are rejected during initialization. Classic ESP32 flash
GPIOs 6–11 are also rejected.

With calibration configured:

| Input state | Reading |
| --- | --- |
| Startup before the timeout, no pulses | Unavailable |
| First pulse in a movement sequence | Unavailable until a second pulse or timeout |
| Two pulses less than the timeout apart | Available speed from their interval |
| No accepted pulse for at least the timeout | Available zero |
| First pulse following a timeout | Unavailable; old stationary interval is discarded |

Stop detection works even if the logger makes no reads during the pause.
Repeated reads never clear the count or change the measured period. Between
pulses, speed retains the last measured value until the timeout; no smoothing
or extrapolation is applied. `period_us` retains the last measured interval on
timeout, so `available` and `speed_mps` determine the current speed result.
`last_pulse_us` is a monotonic microsecond timestamp, meaningful only after
`pulse_count > 0`.

The one-second timeout is provisional. Steady motion needs pulse spacing shorter
than the timeout, giving a lower measurable speed approximately
`distance_per_pulse_m / timeout_seconds`. Increase the timeout if the final
mounting produces sparse pulses at slow speeds. This also increases stop latency.

`min_pulse_interval_us` rejects an edge arriving too soon after the last accepted
edge without resetting the stop timer. It defaults to zero (disabled). Choose a
threshold below the shortest legitimate interval at maximum speed. This is an
interval check, not a pulse-width filter or replacement for signal conditioning.
GPIO interrupts can lose edges if input rates exceed the system's interrupt
capacity; validate the final speed range and aggregate sensor load on hardware.

## Prototype logging

`main/main.c` initializes the first instance before starting the 5 Hz car sampler.
Its settings come from menuconfig; additional instances can receive independent
runtime configuration structs. The application rejects GPIO conflicts with its
configured GPS and SD pins.

`speed1` in `car_data_v2.csv` now records speed in m/s. `speed2` is empty until a
second instance is integrated. Initialization failures and unavailable readings
produce empty fields, while an available stopped reading is `0.00`. RPM and
pressure placeholders continue to be logged. The CSV column layout is unchanged;
historical rows from earlier firmware may contain the old mock speed values.

## Electrical interface

The exact sensor part number, connector orientation, supply voltage, output
topology, output voltage, and pulse count must be verified for the physical unit.
No wiring or sensor supply pin assignment is inferred from the vehicle model.
Provide an externally conditioned digital signal within the ESP32 GPIO limits
and a suitable common reference; GPIO27 is a configurable software default.
Internal pull-up and pull-down resistors are disabled. Depending on the verified
sensor output, the external interface may require a pull-up, level conversion,
and noise protection. Verify the waveform and edge count before calibration.

GPIO34–39 lack internal pull resistors. Avoid module flash/PSRAM pins and account
for boot strap, UART and other board assignments. GPIO36/39 have documented
interrupt restrictions with ADC/radio sleep. Consult the official
[ESP32 GPIO documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/gpio.html)
and the installed ESP-IDF headers for the target board.

## Validation

The host test in `tests/test_speed_measurement.c` feeds synthetic edge timestamps
through the same pulse model used by the ISR. It covers conversion, independent
instances, startup, timeout boundaries, restart without intervening reads,
uncalibrated data, rejected edges, slow pulses, long uptime, and a pulse count
crossing 32 bits. See the root README for its compilation command.

For hardware validation, feed a known conditioned pulse train and compare the
reading against `frequency_hz * distance_per_pulse_m`. Exercise startup, stopping,
restart, input noise, simultaneous instances, and acquisition during SD activity.
Software tests do not establish the physical pinout, electrical interface,
calibration, or maximum sustainable pulse rate. Pulse absence alone cannot
distinguish a stopped vehicle from a disconnected or failed sensor.
