#!/bin/sh
# Credit: https://gist.github.com/willprice/e07efd73fb7f13f917ea

setup_git() {
  git config --global user.email "github@github.com"
  git config --global user.name "Github"
}

commit_git() {
  git checkout master
  # Current month and year, e.g: Apr 2018
  dateAndMonth=`date "+%b %Y"`
  # Stage the modified files in docs/software/mrs_uav_core
  git add -f docs/software/mrs_uav_core/*
  # Create a new commit with a custom build message
  # with "[skip ci]" to avoid a build loop
  # and Travis build number for reference
  git commit -m "Github update: $dateAndMonth" -m "[skip ci]"
}

upload_files() {
  # Remove existing "origin"
  git remote rm origin
  # Add new "origin" with access token in the git URL for authentication
  git remote add origin https://penickar:${GH_TOKEN}@github.com/ctu-mrs/ctu-mrs.github.io.git > /dev/null 2>&1
  git push origin master --quiet
}

setup_git
commit_git

# Attempt to commit to git only if "git commit" succeeded
if [ $? -eq 0 ]; then
  echo "A new commit with changed md files exists. Uploading to GitHub"
  upload_files
else
  echo "No changes in md files. Nothing to do"
fi
