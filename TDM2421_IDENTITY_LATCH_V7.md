# TDM2421 sticky identity latch v7

Changes relative to v6:
- BASIC_ID and OPERATOR_ID latches update only when a new MAVLink packet timestamp arrives.
- A stale/empty Transport cache cannot clear or replace an already accepted Operator ID.
- Last-good SYSTEM/operator location is latched only on a newly accepted SYSTEM packet.
- Loss of MAVLink affects position independently: last-good MAVLink position is held for the configured timeout, then EG800K GNSS becomes RID/MQTT position.
- EG800K GNSS is never used as pilot/operator location.
- MQTT receives the sticky BasicID and OperatorID explicitly.
- Added `[RID ID LATCH]` diagnostics.

Important: the firmware preserves the last identity actually received. It does not invent BID/OID strings. If MAVLink sends `...BIV` as the latest valid BasicID and never sends an OperatorID, the latch will correctly contain BIV and no OperatorID.
