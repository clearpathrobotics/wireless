# wireless
Making info about wireless networks available to ROS.

### Configuration
The interface to be monitored can be configured in the `config/watcher.yaml` file. If you don't set a value, the first `wl` or `wifi` interface seen by `iwconfig` will be selected.

### Published Topics

Topic | Message Type 
--- | ---
`/wireless/connected` | `std_msgs/Bool`
`/wireless/connection` | `wireless_msgs/Connection`

## Dependencies
This node runs on python3. In order to run it on ROS Melodic, you'll need to run the following commands:

```bash
sudo apt install python3-pip python3-all-dev python3-rospkg
sudo apt install ros-melodic-ros-base --fix-missing
```
This node relies on the command line utility `iwconfig`. To install it:
```bash
sudo apt install wireless-tools
```

