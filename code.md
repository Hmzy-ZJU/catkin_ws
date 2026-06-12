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

1.  Nano 启动，
   学生电源电压：25.21V
   学生电源电流：0.216A

   学生电源功率：5.45W

   A0：2.286V
   Vin：25.2V
   A1：0.0071V

   2.打开一路继电器，声呐

   学生电源电压：25.21V
   学生电源电流：0.492A

   学生电源功率：12.36W

   A0=2.2769 V, Vin=25.09 V | A1=0.0215 V, A1_delta=+0.0143 V

   3.打开一路继电器，IMU

   学生电源电压：25.21V
   学生电源电流：1.055A

   学生电源功率：26.54W

   A0=2.2574 V, Vin=24.88 V | A1=0.0484 V, A1_delta=+0.0412 V

   3.打开一路继电器，压力

   学生电源电压：25.21V
   学生电源电流：1.068A

   学生电源功率：26.84W

   A0=2.2566 V, Vin=24.87 V | A1=0.0481 V, A1_delta=+0.0409 V

   

   

   学生电源功率正常工作在36W左右

   
