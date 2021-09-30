# wireless
Making info about wireless networks available to ROS.

### Published Topics

Topic | Message Type 
--- | ---
`/wireless/connected` | `std_msgs/Bool`
`/wireless/connection` | `wireless_msgs/Connection`

## Python3 Setup
This node runs on python3. In order to run it on ROS Melodic, you'll need to run the following commands:

```bash
sudo apt install python3-pip python3-all-dev python3-rospkg
sudo apt install ros-melodic-ros-base --fix-missing
```
