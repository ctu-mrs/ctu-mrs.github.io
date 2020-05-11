#!/bin/bash
# author: Robert Penicka
set -e

echo "Start getting readme of core things" 
cd $TRAVIS_BUILD_DIR/docs/software/
mkdir mkdir -p uav_core
cd uav_core

README_FILE="$TRAVIS_BUILD_DIR/uav_core/README.md"
echo "$README_FILE"
if [[ -f "$README_FILE" ]]; then
    echo "processing uav_core"
    echo -e "---\nlayout: default\ntitle: $module_name\nparent: Software\n---" > index.md
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
            echo -e "---\nlayout: default\ntitle: $module_name\nparent: uav_core\ngrand_parent: Software\n---" > "$module_name.md"
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
