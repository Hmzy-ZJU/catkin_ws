windows

```
cd D:\home\catkin_ws_win_sync
git status
git pull origin main

robocopy D:\home\catkin_ws-main D:\home\catkin_ws_win_sync /E /XD .git build devel install log logs .catkin_tools dataset_EuRoc dataset_harbor dataset_tank dataset_archaeo /XF *.bag *.bag.active *.db3 *.mcap *.log

git status -s
git diff --stat

```

```
git add .
git commit -m "Update catkin workspace from Windows"
git push origin main

```

Ubuntu

```
cd ~/catkin_ws
git status

git fetch origin
git diff --name-status HEAD..origin/main

```

```
git pull origin main

source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin build

```

