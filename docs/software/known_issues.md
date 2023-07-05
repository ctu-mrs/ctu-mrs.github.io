---
layout: default
title: Known Issues
parent: Software
---

## RTK Frame Issue

It has been noted at times that the UAV could find itself under the safety area's minimum-Z height specified. If a tmux flight session is started on the UAV before the RTK fix is fully acquired, it sets its local origin at this start time and can then end up finding itself below ground height even when it is using the height sensor. This issue will show up in trajectory reference as "Start point / pose of the trajectory is outside the safety area".

**Solution: **Restart the tmux session after the RTK and GPS fix has been acquired.
