**Basestation:**

Connect the com1 port to your computer, then turn on TersusGNSScenter.

**Initial Setup - do this only once**

Remove all previous outputs:
```
unlogall
```

Remove any previous fixed location:
```
fix none
```


Turn on COM1 and COM2 output:
```
interfacemode com1 automatic automatic on
interfacemode com2 automatic automatic on
```

Ignore satellites which are 15° or less above the horizon 
```
ecutoff bd2 15.0
ecutoff gps 15.0
ecutoff glonass 15.0
```

Turn on corrections output for com2 (basestation will output corrections only once the position is fixed, see before flight section):

rtcm1074 - GPS corrections

rtcm1084 - GLONASS corrections

rtcm1124 - Beidou corrections

rtcm1005 - Stationary RTK Reference Station ARP

ontime parameter is period, so 10 means once every 10 seconds, 0.05 means 20 Hz.
```
log com2 rtcm1074 ontime 1
log com2 rtcm1084 ontime 1
log com2 rtcm1124 ontime 1
log com2 rtcm1005 ontime 10
```
Save config, without this the config will reset after power loss
```
saveconfig
```

**Before flight - do this every time the basestation is moved**

Connect the com1 port to your computer, then turn on TersusGNSScenter.

Remove any previous fixed location:
```
fix none
```

Start position logging:

gpgga - position information

gpgsv - information about visible satellites
```
log com1 gpgga ontime 0.05
log com1 gpgsv ontime 1
```

Now, run position averaging for basestation in Tersus GNSS Center. Select number of samples - more samples ~ better absolute position. Then clic start and wait. After the averaging is finished, coordinates should appear in the window. Do not close the window for now, type into the console:

```
unlogall com1
```

Now, in the averaging window, click on fix position. This should send the fix command with appropriate coordinates. Make sure that the basestation replies with OK in the terminal. The basestation will only provide rtcm correction messages when the position is fixed. After the position is fixed, make sure to saveconfig:

```
saveconfig
```



**UAV:**

Connect the com1 port to your computer, then turn on TersusGNSScenter.

**Initial Setup - do this only once**

Remove all previous outputs:
```
unlogall
```

Remove any fixed location, location cannot be fixed for rover:
```
fix none
```

Turn on COM1 and COM2 output:
```
interfacemode com1 automatic automatic on
interfacemode com2 automatic automatic on
```

Turn on position output:

```
log com1 gpgga ontime 0.05
```

And save:


```
saveconfig
```

No additional setup is needed before flight.