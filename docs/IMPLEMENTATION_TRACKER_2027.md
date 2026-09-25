# 2027 implementation tracker

This records progress against the [2027 Mentor Recommendations](REUSE_RECOMMENDATIONS_2027.md). The original recommendations and acceptance criteria remain unchanged. The [Delivery Plan](DELIVERY_PLAN_2027.md) sets the order of work.

| Package / finding | Status | Change | Evidence | Next step |
| --- | --- | --- | --- | --- |
| [P3.1](DELIVERY_PLAN_2027.md#phase-3--repair-runtime-foundations-and-verification-infrastructure) / [H1](REUSE_RECOMMENDATIONS_2027.md#acceptance-h1) | Ready for review | Removed duplicate shooter-child `periodic()` calls from [Shooter.java](../src/main/java/frc/robot/subsystems/shooter/Shooter.java). | Four SIM scenarios passed the 100-cycle count check; the final probe-free run showed `OPEN_LOOP` → `CLOSED_LOOP` → `OPEN_LOOP`; `spotlessCheck build` passed. [Run results](shooter-sim-lifecycle/RESULTS_2026-09-25.md). | Mentor reviews the code and evidence. Physical robot behavior awaits hardware. |

`Ready for review` means the change and evidence are available, not that they are approved. Only a reviewer can mark a package `Accepted`.

## P3.1 / H1: one shooter-child update per robot cycle

- **Acceptance:** [H1 finding and checks](REUSE_RECOMMENDATIONS_2027.md#h1-give-each-subsystem-exactly-one-update-per-cycle). The SIM measurements are in the [run record](shooter-sim-lifecycle/RESULTS_2026-09-25.md); the reusable procedure is in the [diagnostic guide](shooter-sim-lifecycle/README.md).
- **Boundary:** the changed callback path is shared by REAL and SIM, but physical behavior has not been tested. Record robot bring-up separately when hardware is available.

For each later package, record its finding, change, evidence, review candidate, hardware status, and reviewer decision. Keep failed or deferred checks visible.
