# GUI Development Checklist

**Project:** 2-DOF Ball and Plate Balancing Platform
**Framework:** PyQt6
**Current phase:** UI development only

## Phase 1 — Main Window

* [ ] Initialize `BallPlateWindow` using `QMainWindow`.
* [ ] Set the application title and initial window dimensions.
* [ ] Create the central widget and main layout.
* [ ] Define minimum window dimensions.
* [ ] Establish consistent margins, spacing, and alignment.
* [ ] Apply the global stylesheet and typography.

## Phase 2 — Header and System Status

* [ ] Display the project name in the header.
* [ ] Add a system state indicator: Idle, Running, or Stopped.
* [ ] Add placeholder connection status for the ESP32.
* [ ] Add placeholder camera status.
* [ ] Ensure status indicators have distinct visual states.

## Phase 3 — Camera and Ball Tracking Panel

* [ ] Create a camera preview placeholder.
* [ ] Maintain the camera preview's aspect ratio when resizing.
* [ ] Display placeholder values for the measured ball X/Y coordinates.
* [ ] Display placeholder values for the target X/Y coordinates.
* [ ] Add visual indicators for the ball position and target position.
* [ ] Clearly label all measurements and units.

## Phase 4 — Setpoint Controls

* [ ] Add a numeric input for target X in millimeters.
* [ ] Add a numeric input for target Y in millimeters.
* [ ] Define suitable input ranges and decimal precision.
* [ ] Add an Apply Setpoint button.
* [ ] Display the currently selected target.
* [ ] Validate input values before applying them.
* [ ] Provide visible feedback when a setpoint is applied.

## Phase 5 — PID Tuning Panel

* [ ] Add separate PID controls for the X-axis.
* [ ] Add separate PID controls for the Y-axis.
* [ ] Add numeric inputs for Kp, Ki, and Kd.
* [ ] Define initial values and appropriate input precision.
* [ ] Group parameters by control axis.
* [ ] Add a Reset PID Values button.
* [ ] Display the current configured PID values.

## Phase 6 — Control Buttons

* [ ] Add a Start Control button.
* [ ] Add a Stop button with a prominent warning style.
* [ ] Add a Reset button.
* [ ] Define UI-only state transitions for each button.
* [ ] Prevent invalid UI transitions, such as starting while already running.
* [ ] Make it clear that hardware control is not yet implemented.

## Phase 7 — Telemetry Panel

* [ ] Display plate pitch in degrees.
* [ ] Display plate roll in degrees.
* [ ] Display X-axis position error in millimeters.
* [ ] Display Y-axis position error in millimeters.
* [ ] Display elapsed time.
* [ ] Use placeholder values until telemetry is implemented.
* [ ] Distinguish unavailable data from actual zero measurements.

## Phase 8 — Plotting Panel

* [ ] Create a placeholder for position-versus-time plots.
* [ ] Create a placeholder for tracking-error plots.
* [ ] Label the future axes and measurement units.
* [ ] Define how the plotting area should resize.
* [ ] Keep plotting logic separate from the main window.

## Phase 9 — Layout and Usability

* [ ] Organize the interface using nested Qt layouts.
* [ ] Group related controls into clearly labeled sections.
* [ ] Ensure the interface remains usable at smaller window sizes.
* [ ] Prevent widgets from overlapping or clipping.
* [ ] Verify tab order and keyboard navigation.
* [ ] Check visual consistency across all panels.

## Phase 10 — UI Testing

* [ ] Verify the application launches successfully.
* [ ] Verify all controls are visible and accessible.
* [ ] Verify the layout responds correctly to resizing.
* [ ] Verify setpoint inputs accept and reject appropriate values.
* [ ] Verify PID reset restores the configured defaults.
* [ ] Verify Start, Stop, and Reset update the displayed UI state.
* [ ] Verify placeholders are not presented as live hardware data.
* [ ] Verify the application closes cleanly.

## Deferred — Hardware Integration

* [ ] Implement serial communication with the ESP32.
* [ ] Implement live camera capture.
* [ ] Integrate ball detection and coordinate calibration.
* [ ] Receive and display actual telemetry.
* [ ] Send setpoints and PID parameters to the controller.
* [ ] Implement hardware-safe stopping and communication-loss handling.
