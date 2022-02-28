---
layout: default
title: R81 receiver setup
parent: Hardware
---

# How to setup Radiomaster R81 receivers

### Binding:

Set the receiver mode on the Radiomaster RC to MULTI FrSky D8.
To bind with the receiver:
1. Hold the button on the receiver, then power the UAV on. 
2. The receiver LED should be solid red.
3. On the RC, make sure you have the correct mode selsected, and press Bind.
4. The receiver should start blinking, the RC should start beeping.
5. After the RC is finished with binding, power off the UAV, then power it back on.
6. You should now see the RC channels throught QGroundControl.

### Setting the failsafe:

Failsafe is here to let Pixhawk know that the RC is disconnected and that Pixhawk should take control and do something (land). This is the case mostly for manual flight, if we fly autonomously in the offboard mode, we want the NUC to retain the offboard control even in case that the RC is lost.
With the new R81 receivers, Pixhawk can detect the RC loss through the SBus protocol, without checking some predefined PWM values on specific channels. For this to work, you need to set these parameters:
* RC_MAP_FAILSAFE -> Unassigned
* RC_FAILS_THR -> 0us

These are the default values, so on a fresh PX4 you should not have to change anything. To check if everything is working, connect to PX4 through QGroundControl, and then turn off the RC. You should see a message saying 'Manual control lost'. This means PX4 detected the loss of RC, which is what we want. Turn the RC back on and you should see a message saying 'Manual control regained'.

When the receiver is in the failsafe mode (rc is lost), it is outputing some predefined values on its channels. We want to set these channels to specific values, e.g. keep the offboard mode on even when RC is lost. Follow these steps:
1. Turn off the UAV
2. Set the RC into a state to represent the desired failsafe output of the receiver:
  * Put all switches into default positions
  * Set the mode switch to Position (GPS)
  * Set the offboard switch to On
  * Set the throttle to minimum
3. Once the RC is in the desired configuration, turn on the UAV. 
4. Wait a second for the RC to connect (solid red light) and then press the button on the receiver. You have 10 seconds to do this after the receiver is powered on.
5. The receiver should start flashing, release the button.
6. Power cycle the receiver. The failsafe output is now set, when the RC is now powered off, the receiver will output these saved values.
7. Verifiy this in QGroundControl, put the RC into some random state, turn it off and you should see the channels go to the values for offboard and position mode, while QGroundControl will report 'manual control lost'
