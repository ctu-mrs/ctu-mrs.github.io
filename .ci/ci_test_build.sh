#!/bin/bash
# author: Robert Penicka
set -e

echo "Start getting readme of core things" 
cd $TRAVIS_BUILD_DIR/docs/software/
mkdir mkdir -p uav_core
cd uav_core

for module in ~/uav_core/ros_packages/* ; do
    if [ -d "$module" ]; then
        README_FILE="$module/README.md"
        echo "$README_FILE"
        if [[ -f "$README_FILE" ]]; then
            echo "processing module $module"
            module_name=`basename $module`
            echo -e "---\nlayout: default\ntitle: $module_name\nparent: Software\n---" > "$module_name.md"
            cat $README_FILE >> "$module_name.md"
            #ls
        else
            echo "readme file $README_FILE does not exists"
        fi
    fi
done

ls
git status



echo "Ended"
