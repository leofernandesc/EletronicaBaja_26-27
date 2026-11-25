# Component provenance

These source files are a standalone copy of `components/speed_sensor` from
branch `feat/speed-sensor`, commit
`434ddd1ff0eba950d8a9e82c242244f1edec58cf`. The diagnostic project does not
modify the driver. T6 uses its public init/read/deinit API; the other modes use
their own ISR capture while sharing the GPIO ISR service installed by this
component. The temporary service bootstrap handler is removed before a test
claims its GPIO.

Dependencies: ESP-IDF `esp_driver_gpio`, `esp_timer`, `heap`, and `freertos`.
`CONFIG_ESP_TIMER_IN_IRAM=y` is required. Init/deinit must run on the core that
installed the service; this project keeps mode lifecycle work on the main task
affined to CPU0. The global GPIO ISR service remains installed between modes,
while the active test handler and pin configuration are removed/reset before
another mode starts.
