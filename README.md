# SDIR 2020 Controler
This code repository consists of two parts: the ctrl and the sim part. The ctrl folder contains all necessary code files for your controler. The sim folder contains the [VREP](https://www.coppeliarobotics.com/) .ttt file with all necessary implementation for your simulation.
For the implementation, you are free to add any additional files. Please, however, do not change the given interfaces without highlight the changes to the course supervisors. This is, because we will come back on your generated libraries for testing and evaluation purposes. In case you change the interfaces, automated testing might not work propper.

## Information for testing the code

The controller supports plotting a vector of configurations and other visualizations. To do so it makes use of a
wrapper library called matplotlibcpp (https://github.com/lava/matplotlib-cpp) which uses the python library
matplotlib. To be able to use this functionality, one has to install python3 and matplotlib. However, this
functionality can be disabled completely in the root cmake script so that none of this is required. Additionally
one can disabled the plotting functionality in code for ptp, lin, and splines. This might be useful since
showing the plots blocks the controller.

Others changes to the internal API that need to be considered:

1. SixDPos* get_pos_from_config(Configuration* config);
   
   -> Unchanged
   

2. vector<Configuration*>* get_config_from_pos(SixDPos* pos);
   
   -> Unchanged
   

3. Trajectory* move_robot_ptp(SixDPos* start, SixDPos* end);
   
    -> Not implemented since it does not get used
   

4. Trajectory* move_robot_ptp(Configuration* start, Configuration* end, bool sync = false);
   
    -> One can choose the operation mode. If not specified, it will perform an asynchronous movement.
   

5. Trajectory* move_robot_lin(Configuration* start, Configuration* end, double velocity, double acceleration, std::vector<std::vector<SixDPos*>>* loopPoints);

   -> Requires a velocity and an acceleration. Optionally, one can pass a pointer to receive certain points, just use 
   a nullptr in case this functionality is not needed.
   

6. Trajectory* move_robot_lin(Configuration* start, Configuration* end, double velocity, double acceleration);

   -> Not implemented since it does not get used
   

7. Trajectory* move_robot_spline(vector<SixDPos*> &points, Configuration * start, double velocity, double acceleration, std::vector<std::vector<SixDPos*>>* loopPoints, double elong = 0.5, int _spline_type = 0);

    -> Requires a velocity and an acceleration. Optionally, one can pass a pointer to receive certain points, just use
   a nullptr, if this functionality is not needed. The other two parameters specify the type and scalar elongation
   factor, you can leave the default values.

The functions in the lower levels for spline, lin, and ptp follow the same principle. In general, you can find the
necessary information for every function in the corresponding header file.

## Build
For building your code, please use the contained [cmake](https://cmake.org/) files.


## Some git and bibucket hints for you

**Edit a file, create a new file, and clone from Bitbucket in under 2 minutes**

When you're done, you can delete the content in this README and update the file with details for others getting started with your repository.

*We recommend that you open this README in another tab as you perform the tasks below. You can [watch our video](https://youtu.be/0ocf7u76WSo) for a full demo of all the steps in this tutorial. Open the video in a new tab to avoid leaving Bitbucket.*

---

## Edit a file

You’ll start by editing this README file to learn how to edit a file in Bitbucket.

1. Click **Source** on the left side.
2. Click the README.md link from the list of files.
3. Click the **Edit** button.
4. Delete the following text: *Delete this line to make a change to the README from Bitbucket.*
5. After making your change, click **Commit** and then **Commit** again in the dialog. The commit page will open and you’ll see the change you just made.
6. Go back to the **Source** page.

---

## Create a file

Next, you’ll add a new file to this repository.

1. Click the **New file** button at the top of the **Source** page.
2. Give the file a filename of **contributors.txt**.
3. Enter your name in the empty file space.
4. Click **Commit** and then **Commit** again in the dialog.
5. Go back to the **Source** page.

Before you move on, go ahead and explore the repository. You've already seen the **Source** page, but check out the **Commits**, **Branches**, and **Settings** pages.

---

## Clone a repository

Use these steps to clone from SourceTree, our client for using the repository command-line free. Cloning allows you to work on your files locally. If you don't yet have SourceTree, [download and install first](https://www.sourcetreeapp.com/). If you prefer to clone from the command line, see [Clone a repository](https://confluence.atlassian.com/x/4whODQ).

1. You’ll see the clone button under the **Source** heading. Click that button.
2. Now click **Check out in SourceTree**. You may need to create a SourceTree account or log in.
3. When you see the **Clone New** dialog in SourceTree, update the destination path and name if you’d like to and then click **Clone**.
4. Open the directory you just created to see your repository’s files.

Now that you're more familiar with your Bitbucket repository, go ahead and add a new file locally. You can [push your change back to Bitbucket with SourceTree](https://confluence.atlassian.com/x/iqyBMg), or you can [add, commit,](https://confluence.atlassian.com/x/8QhODQ) and [push from the command line](https://confluence.atlassian.com/x/NQ0zDQ).