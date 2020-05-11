#!/bin/bash
# author: Robert Penicka
set -e

echo "Start getting readme of core things" 
cd $TRAVIS_BUILD_DIR/docs/software/
mkdir mkdir -p uav_core
cd uav_core

README_FILE=~/uav_core/README.md
module_name="uav_core"
echo "$README_FILE"
if [[ -f "$README_FILE" ]]; then
    echo "processing uav_core"
    echo -e "---\nlayout: default\ntitle: $module_name\nparent: Software\nhas_children: true\n---" > index.md
    cat $README_FILE >> index.md
else
    echo "readme file $README_FILE does not exists"
fi

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
                echo "copy figs from $FIG_FOLDER to $module_name/.fig/"
                mkdir mkdir -p $module_name/.fig/
                cp -r $FIG_FOLDER/* $module_name/.fig/
            else
                echo "FIG_FOLDER $FIG_FOLDER does not exists"
            fi
            ##### fig part end

            echo -e "---\nlayout: default\ntitle: $module_name\nparent: uav_core\ngrand_parent: Software\n---" > "$module_name/index.md"
            cat $README_FILE >> "$module_name/index.md"
            
        else
            echo "readme file $README_FILE does not exists"
        fi
    fi
done

ls
git status


echo "Ended"
