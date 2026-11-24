# Product

<!-- impeccable:product-schema 1 -->

## Platform

web

## Users
- Primary: members of environmental capstone design team 1 (university students). They run bench and chamber tests on a laptop at the lab bench: watch the sensor values, force states, inject test values, find servo angles, calibrate, and save the logs.
- Secondary: the professor and audience at the final presentation. The same page is shown on a projector or large monitor while the device is demonstrated live.

## Product Purpose
A control panel for a stormwater catch-basin prototype that separates out the first flush. An Arduino UNO reads turbidity (DFRobot SEN0189) and water level (HC-SR04 ultrasonic) in a measurement chamber, then drives a servo to slide a perforated plate. The plate opens (TREAT): water goes down through the microfilter and biochar. The plate stays closed (BYPASS): water leaves through the side overflow. The page lets anyone run, test, and demonstrate the device without the Arduino IDE or typing serial commands. Success means a teammate can plug in, understand the current state at a glance, and make the plate move on purpose within a minute.

## Positioning
It shows the controller's real decision logic live, not just raw numbers: the current state, why it got there, and which condition triggers the next transition. Values can be injected so the automatic judgment can be demonstrated on demand.

## Operating Context
- Chrome or Edge on a Mac or Windows laptop, connected over USB with Web Serial at 115200 baud. Opened as a local file or from GitHub (repo Dovcl/Stormwater-Gate).
- Firmware RAIN/RAIN.ino sends a CSV row every second (`ms,state,test,mem,raw_adc,avg_adc,turb_index,dist_mm,level_mm,open,servo_deg,reason,sim,warn`) plus `#` event lines.
- Two settings: lab bench testing (frequent, hands busy with water and wires) and the final presentation (projector, viewed from a distance).
- Everything is in Korean.

## Capabilities and Constraints
- States: FILL 채움, MEASURE 측정, TREAT 처리 (plate open), BYPASS 우회 (plate closed), ALARM 경보 (currently disabled in firmware), MANUAL 수동.
- Controls:
  - Force a state (test mode), or return to auto.
  - Inject a turbidity % or level mm value (t/h commands), or go back to the real sensor.
  - Manual servo mode with an angle slider.
  - Clear-water calibration.
  - Save the CSV.
- Thresholds come from firmware config lines:
  - Turbidity 15/25%.
  - Level 20/40/80 mm. H_MM is 125.
- Single self-contained HTML file. It must work offline with no external dependencies, because a lab laptop may have no network.
- Servo angles and thresholds cannot be changed from the page; they are set in the firmware.

## Brand Commitments
None. Team project, no logo or brand. The user explicitly rejects interfaces that look AI-generated.

## Evidence on Hand
Real bench log captured from the board (input-value and test-mode runs). There are no photos of the device yet, so none may be fabricated.

## Product Principles
1. The state and the plate position are readable from across a room.
2. Every number has a reason: show the threshold and the next transition rule next to the live value.
3. Testing actions are safe and reversible, with an obvious way back to auto or the real sensors.
4. Works offline from one file; nothing to install.
