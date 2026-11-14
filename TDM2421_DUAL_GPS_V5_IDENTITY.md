# TDM2421 Dual GPS V5 - Remote ID Identity

V5 extends V4 with configurable Remote ID identity and deterministic MQTT object identity.

## Identity policy

- OpenDroneID Basic ID / UAS ID is configurable from the local dashboard.
- Saving a non-empty Basic ID sets UAS_TYPE to multirotor and UAS_ID_TYPE to serial number.
- MQTT `objectId` priority: valid OpenDroneID Basic ID -> configured Device ID fallback.
- MQTT `extension.uavSN` priority: configured UAV Serial -> Basic ID -> Device ID fallback.
- Device ID remains the identity of the TDM2421/MQTT sender.

## Dashboard

The UAV / MAVLink / Remote ID card now shows Object ID, Basic ID/UAS ID, Basic ID type/source, UAV Serial and Device ID.
The Configuration page has a Remote ID Identity section for Basic ID/UAS ID and UAV Serial.
