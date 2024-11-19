---
title:Buildingimages
pagination_label:BuildingDockerimages
description:Howtobuilddockerimages.
---

#Buildingimages

Followthe[officialguide](https://docs.docker.com/build/)formorein-depthinformation.

###BuildingaDockerImagefromaDockerfile

ADockerfileisatextfilecontainingaseriesofinstructionstocreateaDockerimage.
Here'showtobuildanimageusingaDockerfile.

1.**CreateaDockerfile**:
Writea`Dockerfile`withthenecessaryinstructionsforyourapplication.Example:
```dockerfile
#UsetheMRSUAVSystemthebaseimage
FROMctumrs/mrs_uav_system:latest

#Installadditionalpackagesintotheimage
RUNsudoapt-get-yinstall<my_dependency>

#Specifythedefaultcommandtorun
CMD["/ros_entrypoint.sh"]
```
2.**SavetheDockerfile:**SavetheDockerfileintherootdirectoryofyourproject.
3.**BuildtheDockerImage:**Usethedockerbuildcommandtobuildtheimage.RunthiscommandinthesamedirectoryasyourDockerfile:
```bash
dockerbuild-t<image-name>:<tag>.
```
Example:
```bash
dockerbuild-tmy-app:latest.
```

*The`-t`flagassignsanameandtagtoyourimage(e.g.,my-app:latest).
*The`.`attheendspecifiesthebuildcontext(thecurrentdirectory).

##DockerfilefortheMRSUAVSystem

```dockerfile
FROMctumrs/ros:noetic

RUNapt-get-yupdate

#workaroundinterractivepromptsduringaptinstallations
RUNecho'debconfdebconf/frontendselectNoninteractive'|sudodebconf-set-selections
RUNDEBIAN_FRONTEND=noninteractiveapt-get-yinstallkeyboard-configuration

#INSTALLtheMRSUAVSystem

RUNapt-get-yinstallsoftware-properties-commoncurlbash

RUNcurlhttps://ctu-mrs.github.io/ppa-stable/add_ppa.sh|bash

RUNapt-get-yinstallros-noetic-mrs-uav-system-full

CMD["/ros_entrypoint.sh"]
```
