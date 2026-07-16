# wireless

ROS 2 packages for monitoring wireless network interfaces on Linux systems.

## Packages

### wireless_watcher

A ROS 2 node that publishes connection information about a Linux wireless interface, including link quality, signal level, bitrate, and ESSID. It also provides diagnostics via `diagnostic_updater`.

**Published Topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `connected` | `std_msgs/Bool` | Whether the wireless interface is currently connected |
| `connection` | `wireless_msgs/Connection` | Detailed connection information |

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `hz` | double | `1.0` | Publishing rate in Hz |
| `dev` | string | `""` | Wireless device to monitor (auto-detected if empty) |
| `connected_topic` | string | `"connected"` | Topic name for connection status |
| `connection_topic` | string | `"connection"` | Topic name for connection details |

**Launch:**

```bash
ros2 launch wireless_watcher watcher.launch.py
```

### wireless_msgs

Message definitions for describing a wireless network.

| Message | Description |
|---------|-------------|
| `Connection.msg` | Full connection details |
| `Network.msg` | Network information |
| `Quality.msg` | Link quality metrics |
| `Scan.msg` | Scan results |

## Build

```bash
colcon build --packages-select wireless_watcher wireless_msgs
```

## License

BSD
