# How to connect your MRS GitLab paper repo with an overleaf project

1. Create a project in overleaf
  * Create a blank project, no complex stuff is required
  * Important note - you must be login with an email and password, not just through a google account, it is needed to push to the overleaf afterward since it doesn't support SSH keys by default
2. Getting git link
  * Once you've created a project, go to the menu, in the section called ```Sync``` you can find ```Git```, go there.
    * If you have created a new account, this feature will be paid only. For this purpose you have to use our shared credentials, see on this page "Overleaf credentials" [here](http://mrs.felk.cvut.cz/internal), use them to log in and get GitLab link
  * This will open a pop-up window with smth like ```git clone https://git.overleaf.com/<your overleaf project link>```, copy the https link

3. Syncing your repo
  * Open your repo, and run the following command

    ```git remote add over <your https link from step 2 >```
  * You will need firstly no pull the overleaf repo, to synchronize the histories

    ```git pull over master --allow-unrelated-histories```

    ***Note***: if you got tired to type in the credentials every time run this command, it will store them in cache:

    ```git config credential.helper "cache --timeout 3600" ```
  * And ofc there are gonna be merge conflicts, so use mergetool to fix them, just delete the initiated part by overleaf and use yours

    ``` git mergetool ```
  * Afterward you can push to the overleaf

    ``` git push over master ```
4. Pushing to overleaf the easy way
    ``` git push over master ```
5. Pulling from overleaf
    ```git pull over master ```
6. Pushing to overleaf the harder way with two branches
  * Since overleaf changes all the stuff automatically, you don't want somebody to mess around with your master branch,( it doesn't push, but once you want to sync it's kinda painful ),
  so let's create an overleaf branch

    ``` git checkout -b overleaf ```
  * Then if you want to pull from overleaf to this branch use the following command

    ``` git pull over master:overleaf ```
  * And all the changes from the overleaf will be on the current overleaf specific branch, and you can safely merge it with master
  * So, if you have done any changes that you want to push use this command: 

    ``` git push over overleaf:master ```

       **Note**: you can substitute overleaf with any branch name.
 7. Pulling from overleaf
   * For much more pleasant experience, I recommend pulling the changes from overleaf to a separate branch, similar to step 4, via this sequence 
  ``` 
  git fetch 
  git checkout overleaf
  git merge over/master
```