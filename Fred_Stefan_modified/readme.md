# Setup (DON'T FOLLOW THESE INSTRUCTIONS! THEY ARE OUTDATED! GO TO fred.rovder.com FOR INSTRUCTIONS! )

Follow these instructions to get up and running.



## Git

You need a Unix shell for this. (There is one on DICE ;) 

1. **Clone the repo**:

    `git clone https://bitbucket.org/sdpateam/strategy.git`


2. **Add SSH keys to bitbucket**:

    To generate an SSH key, follow this guide: (Use the email address you use for Bitbucket)

    https://confluence.atlassian.com/bitbucketserver/creating-ssh-keys-776639788.html

    Then, pair the key with your bitbucket account:

    https://confluence.atlassian.com/bitbucket/add-an-ssh-key-to-an-account-302811853.html


3. **Run the setup script** (Btw. you **have to** do step 2 for this to work ;).

    `cd strategy`
    `sh setup.sh`

    This sets up all the submodules, dependencies and everything.
    
   
Now you have all the code you need. You should use git to keep the code up to date.



## IntelliJ Setup (Optional)
IntelliJ is a convenient IDE, which TBH is superior to all (even to emacs Angus)! It will make your life easier, and takes 10 mins to set it up. I wrote 6 EASY steps for you :)
 
 1. **Get it.**
 
    Download it from here: https://www.jetbrains.com/idea/download/ and run it.
    Note: DICE complains about admin rights, you can safely ignore this.
 
 
 2. **Create a new project.**
 
    Select `New Project` --> `Java`.

    Set SDK to 1.7.0 by selecting the folder where it is installed. On DICE this is: `**INSERT FOLDER PATH HERE**.`

    Hit `Next`, `Next` --> Name the project whatever you like (e.g. SDP)

    Set location to the `/strategy` folder where your code is. 

    When prompted, don't add anything to git! IntelliJ git integration is unsuitable for this project (it doesn't handle sub modules well.)
    
    
 3. **Recreate the project structure**
    We need to create modules for vision and communication.
    
    Select `File` --> `Project Structure` --> `Modules`.
    
    To add a module click the green plus symbol at the top -> `Import Module` —> select module folder —> click next until you are done, then finish.

    Do this for Vision and Communications.
     
     
 4. **Add libraries to the project**
 
    From the same window, select strategy module-> click the dependencies tab —> there is a (green) plus at the right of the window (or at the bottom). 

    Click it, select `JARs or Dirs` and select strategy/libs folder 
 
    
 5. **Link modules together**
 
    In the same window, click the same plus button again and and select Module Dependency. Add both communication and vision module. You have now linked strategy module to communication module and to vision module.
     
    Now you have to link communication module to strategy and vision to strategy.

    
 6. **Creating a runnable**
    so that you can run the code from IntelliJ. You will have to do this for every class you'd like to run.
 
    In the project browser right click the libs folder, select Copy path.
    Click Run/Edit configurations -> Click (green) plus.
    Select Application give it a name (e.g. Strategy).
    Now just fill in the fields. 
     
     For `main class` field select Strategy.strategy (or whatever you want)
     For `module` field select strategy
      
     and for `environment variables` click three dots and add LD_LIBRARY_PATH and Paste the path you have copied.
    
    
That is it! You are ready to write and run the code!
 


## Workflow guide

This part of the guide describes the everyday routine one should use when working with git.

Before you start working run:

`git submodule foreach git pull origin master`

This makes sure you're up to date. You can now start working!
After you have made your changes, add the files to be committed: 

`git add -A`

Then group it into a commit. Don't forget to include a descriptive message, mentioning what you have done:

`git commit -m "MY COOL MESSAGE HERE" `

Lastly push the changes to the server:

`git push`

I you changed the Communication or Vision modules, you should `cd` into the respective directory and `add` and 
all the rest.