^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wireless_watcher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2023-05-01)
------------------
* [wireless_watcher] Switched to underscores to get rid of usage of dash-separated warning.
* Contributors: Tony Baltovski

1.0.0 (2022-10-16)
------------------
* Minor fixes
* Added previous changelogs
* Updated wireless_msgs for ROS2.
  Updated wireless_watcher for ROS2.
* Contributors: Roni Kreinin

0.1.2 (2021-11-30)
------------------
* Add wireless-tools as a dependency (`#14 <https://github.com/clearpathrobotics/wireless/issues/14>`_)
* Contributors: Chris I-B

0.1.1 (2020-05-01)
------------------
* Improve connected detection logic so topic doesn't stop when interface is down
* Contributors: Nikesh Bernardshaw

0.1.0 (2019-10-10)
------------------
* Python 3 fixes for watcher_node. (`#10 <https://github.com/clearpathrobotics/wireless/issues/10>`_)
* Contributors: Mike Purvis

0.0.9 (2018-11-27)
------------------
* Accept 'wifi' as device prefix for auto-detection.
* Contributors: Mike Purvis

0.0.8 (2018-11-07)
------------------
* Package format 2.
* Further pep8 fixups.
* Auto-detect wl* device if not passed explicitly.
* Contributors: Mike Purvis

0.0.7 (2015-09-09)
------------------
* Added frequency to watcher node
* Contributors: Alex Bencz

0.0.6 (2015-09-02)
------------------
* Added queue_size parameter to publishers
* Contributors: Mustafa Safri

0.0.5 (2015-08-25)
------------------
* Added checks for missing fields
* Contributors: Mustafa Safri

0.0.4 (2015-06-25)
------------------
* Add install rule for launch dir
* Contributors: Paul Bovbel

0.0.3 (2015-06-17)
------------------
* Added ESSID and BSSID fields, use floats for bitrate
* Contributors: Alex Bencz

0.0.2 (2013-10-24)
------------------
* Workaround to suppress DummyThread error spew.
* Add simple boolean to publish ethernet-has-ip status.

0.0.1 (2013-10-17)
------------------
* Catkinize wireless_watcher