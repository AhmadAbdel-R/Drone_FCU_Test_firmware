# Performance Results

Date: 2026-06-20

No hardware performance measurements have been captured for the Windows configurator path yet.

Current verified results are build/test results only:

- `esp32-s3-mini` firmware builds successfully.
- `esp32-s3-mini-pidweb` bench firmware builds successfully.
- `esp32-s3-mini-configurator` firmware builds successfully with the bench-only USB endpoint enabled.
- Node protocol and mock-client tests cover parser correctness and desktop request flow.
- Vue production build completes and the mock dashboard renders with one active Three.js canvas.
- Portable ZIP generation completes at `configurator/release/FCU-Configurator-Portable-x64.zip`.

Measurements still required before any flight-release claim:

- Average and worst-case flight-loop time while USB configurator polling is active.
- Loop jitter.
- USB parser service time.
- Queue high-water marks.
- Dropped telemetry frames.
- Protocol CRC/version/length error counts under noisy input.
- Graph/UI frame rate during 30-60 Hz attitude display and 50-100 Hz PID telemetry.
- Motor-test stop latency after release, disconnect-by-timeout, unsafe state, and explicit stop.
