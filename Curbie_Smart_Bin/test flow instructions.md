# Curbie UI Test Flow (Current Curbie_Smart_Bin UI)

This file matches the current `Curbie Smart Bin Operator` app in `Curbie_Smart_Bin`.

Important:
- There is no bin-registration screen in this UI.
- Markers are triggered by event buttons during recording.
- On replay, those saved events run again automatically.

## 0) What each tab is for

- `Drive`: manual movement + hook controls + activity log.
- `Paths`: record path, insert pickup/dropoff events, replay/rename/delete saved paths.
- `Seek + Sonar`: run pickup/dropoff manually and view the five sonars (telemetry only, no obstacle avoidance).

Important:
- There is no camera preview tab in this UI by design.
- Camera topics are still used internally by marker-seek on the robot.

## 1) Pre-check before recording

1. Open `Curbie Smart Bin Operator`.
2. Go to `Seek + Sonar` and click `Reset Collection Offset`.
3. Go to `Drive` and click `Release Hook (C)`.
4. In `Drive`, click `Stop / Cancel (X)`.
5. Put the robot at the exact home start pose.

## 2) Record one full two-bin demo path (exact steps)

Target sequence:
`home -> pickup 10 -> dropoff 40 -> pickup 11 -> dropoff 40 -> home`

1. Go to `Paths`.
2. In `Record New Path`, set:
`Path Name`: `two_bin_10_11_40`
`Description`: `home -> 10 -> 40 -> 11 -> 40 -> home`
3. Click `Start Recording`.
4. Drive the robot from home to bin-10 staging pose.
5. Click `Record + Run Pickup 10`.
6. Wait until seek finishes (Live Status `Seek` returns to idle/completed and hook is engaged).
7. Drive from bin 10 area to collection-marker staging.
8. Click `Record + Run Dropoff 40`.
9. Wait until seek finishes (hook is disengaged).
10. Drive from collection to bin-11 staging pose.
11. Click `Record + Run Pickup 11`.
12. Wait until seek finishes (hook is engaged).
13. Drive from bin 11 area back to collection-marker staging.
14. Click `Record + Run Dropoff 40`.
15. Wait until seek finishes (hook is disengaged).
16. Drive from collection back to home pose.
17. Click `Stop Recording`.
18. Click `Refresh Paths`.
19. In `Saved Paths`, confirm `two_bin_10_11_40` appears.
20. Check its `Steps` column shows `(... 4 events)` for the two pickups + two dropoffs.

## 3) Replay the saved two-bin demo

1. In `Paths`, click the `two_bin_10_11_40` row in `Saved Paths`.
2. Click `Replay Selected`.
3. Watch `Live Status` for:
`Replay` progress, `Seek` phase, `Hook` state, and `Collection Offset`.
4. If needed, stop immediately with `Cancel Replay / Seek` or keyboard `X`.
5. Sonar values are still shown live, but replay now follows the saved path directly without sonar-based steering.

## 4) Expected replay behavior

1. Robot follows recorded motion segments.
2. It runs pickup seek on marker `10`, then engages hook.
3. It runs dropoff seek on marker `40`, then disengages hook.
4. It runs pickup seek on marker `11`, then engages hook.
5. It runs dropoff seek on marker `40` again, with increased left offset.
6. It follows final recorded motion and ends at home.

## 5) Notes that avoid common confusion

- No bin registration is required in this UI.
- `Record + Run Pickup 10/11` and `Record + Run Dropoff 40` are the bin/collection logic.
- Keyboard shortcuts are ignored while typing in text boxes.
- After typing path fields, click outside the field before using movement keys.
