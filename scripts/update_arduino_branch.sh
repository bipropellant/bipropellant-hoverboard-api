#!/bin/sh

setup_git() {
  git config --global user.email "travis@travis-ci.org"
  git config --global user.name "Travis CI"
}

flatten_submodule() {
  git checkout -b master_arduino

  git rm --cached src/hbprotocol # delete reference to submodule HEAD (no trailing slash)
  git rm .gitmodules             # if you have more than one submodules,
                                 # you need to edit this file instead of deleting!
  rm -rf src/hbprotocol/.git     # make sure you have backup!!
  rm -rf src/hbprotocol/examples
  rm -rf src/hbprotocol/README.md
  rm -rf scripts
  rm -rf .travis.yml
#  mv src/hbprotocol/* src/
#  rm -rf src/hbprotocol
  git add .
#  git mv src/* ./

  git commit --message "Auto convert to Arduino Library ($TRAVIS_BUILD_NUMBER)"
}

upload_files() {
  git remote add upload https://${GITHUB_TOKEN}@github.com/bipropellant/bipropellant-hoverboard-api.git > /dev/null 2>&1
  git push --quiet --set-upstream upload master_arduino -f
}

setup_git
flatten_submodule
upload_files
