#!/bin/bash
# author: Robert Penicka
set -e

echo "Start getting readme of core things"
cd `pwd`/docs/software/
mkdir -p uav_core
cd uav_core

##########################
#   UAV CORE PART begin  #
##########################
README_FILE=~/uav_core/README.md
module_name="uav_core"
##### fig part begin
FIG_FOLDER=~/uav_core/.fig
if [ -d "$FIG_FOLDER" ]; then
    echo "copy figs from $FIG_FOLDER to fig/"
    mkdir mkdir -p fig/
    cp -r $FIG_FOLDER/* fig/
else
    echo "FIG_FOLDER $FIG_FOLDER does not exists"
fi
##### fig part end
echo "$README_FILE"
if [[ -f "$README_FILE" ]]; then
    echo "processing uav_core"
    echo -e "---\nlayout: default\ntitle: $module_name\nparent: Software\nhas_children: true\n---" > index.md
    cat $README_FILE >> index.md
    sed -i 's/\.fig/fig/g' index.md

else
    echo "readme file $README_FILE does not exists"
fi
########################
#   UAV CORE PART end  #
########################

#############################
#   UAV CORE modules begin  #
#############################

for module in ~/uav_core/ros_packages/* ; do
    if [ -d "$module" ]; then
        README_FILE="$module/README.md"

        echo "$README_FILE"
        if [[ -f "$README_FILE" ]]; then
            echo "processing module $module"
            module_name=`basename $module`

            ##### fig part begin
            FIG_FOLDER="$module/.fig"
            if [ -d "$FIG_FOLDER" ]; then
                echo "copy figs from $FIG_FOLDER to $module_name/fig/"
                mkdir mkdir -p $module_name/fig/
                cp -r $FIG_FOLDER/* $module_name/fig/
            else
                echo "FIG_FOLDER $FIG_FOLDER does not exists"
            fi
            ##### fig part end

            mkdir mkdir -p $module_name/
            echo -e "---\nlayout: default\ntitle: $module_name\nparent: uav_core\ngrand_parent: Software\n---" > "$module_name/index.md"
            cat $README_FILE >> "$module_name/index.md"
            sed -i 's/\.fig/fig/g' "$module_name/index.md"

        else
            echo "readme file $README_FILE does not exists"
        fi
    fi
done

###########################
#   UAV CORE modules end  #
###########################

ls
git status

echo "Ended"
