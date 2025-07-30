---
title: Overleaf-to-git sync
pagination_label: Syncing overleaf with a git repository
description: Syncing overleaf with a git repository
---

# Sync git repository with an Overleaf project

1. Create a project in overleaf:
  * Create a blank project, no complex stuff is required
  * **Note:** you must login with an email and password and not through a Google account. It is needed to push to the overleaf afterward since it doesn't support SSH keys by default.
2. Get git link:
  * Once you've created a project, go to the menu, in the section called ```Sync``` you can find ```Git```. Go there.
    * If you have created a new account, this feature will be paid only. For this purpose you have to use our shared credentials, see on this page "Overleaf credentials" [here](http://mrs.felk.cvut.cz/internal), use them to log in and get GitLab link.
  * This will open a pop-up window with ``git clone https://git.overleaf.com/<your overleaf project link>``. Copy the ``https...`` link.

3. Sync your repo:
  * **Recommendation**: cache the credentials so you don't have to type them all the time: ``git config credential.helper "cache --timeout 3600"``
  * Add remote: open your repo, and run command: ``git remote add overleaf <https link from step 2 >``
  * Pull the overleaf repository while synchronizing the histories: ``git pull overleaf master --allow-unrelated-histories``
  * Resolve merge conflicts: ``git mergetool``
  * Push to overleaf: ``git push overleaf master``
  * (optional) Pulling from overleaf: `git pull overleaf master`

### Two-branch setup (recommended)
Overleaf automatically pushes its changes. As you don't want anyone to mess with your master branch, create a two-branch setup in which you are free to push/pull/merge as you wish.

1. Create an overleaf branch: `git checkout -b overleaf`
2. Push your changes at `overleaf` branch to Overleaf: `git push overleaf overleaf:master`
3. Pull to this branch from Overleaf: `git pull overleaf master:overleaf`
4. Or pull using `merge` as
```bash
  git fetch 
  git checkout overleaf
  git merge overleaf/master
```
