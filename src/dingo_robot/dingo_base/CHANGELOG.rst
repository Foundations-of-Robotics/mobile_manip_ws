^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dingo_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.3 (2021-03-08)
------------------

0.1.2 (2020-11-26)
------------------
* [dingo_base] Fixed battery topic.
* Contributors: Tony Baltovski

0.1.1 (2020-11-23)
------------------
* Added missing test dependencies.
* Roslint fixes
* Diagnostics update (`#2 <https://github.com/dingo-cpr/dingo_robot/issues/2>`_)
  * Separate the diagnostics config into 3 separate files (common, omni, diff)
  * Tidy up the wheel expectations, add the wi-fi connected topic to general
  * Fix the CAN IDs of the four wheels
  * Fix c&p error with left/right in the previous commit
  * Correctly load the diff/omni files in the right places
  * Remove the wi-fi connected entry
  * Refactor the diagnostic updater to remove magic numbers & move them to defined constants. Add a message to the power diagnostics to get rid of an error saying no message was set
  * Use constexpr instead of define
  * Fill in the correct voltage & amperage warning levels from Dana.  Move the definitions into a new header so we can re-use it in lighting.cpp.  Add a new over-volt lighting state with all-blue lights
  * Start implementing Li battery checks using the BatteryState message. Topic subscribed-to needs checking, as do the percentage warning levels
  * Update the battery state topic
  * Remove the leading / from the battery topics to stay consistent with mcu/status
  * Add an extra TODO indicator for the Lithium battery testing as a blanket warning that it's untested so far
  * Roslint fixes
  * Increase the low-battery warning to 20% for Lithium batteries
  * Increase the critical battery level to 10% for lithium batteries
* Contributors: Chris I-B, Chris Iverach-Brereton, Tony Baltovski

0.1.0 (2020-11-13)
------------------
* Bump CMake version to avoid CMP0048 warning.
* [dingo_base] Reduced log spamming.
* [dingo_base] Fixed not using function for ros::ok().
* [dingo_base] Updated encoder count for mmp-s14 motor config.
* [dingo_base] Added motor parameters as rosparams to be loaded as a YAML which is set by an enviroment variable.
* [dingo_base] Updated I gain.
* Merge branch 'melodic-devel' of gitlab.clearpathrobotics.com:dingo/dingo_robot into melodic-devel
* Merge branch 'tb-rework' into 'melodic-devel'
  Initial rework
  See merge request dingo/dingo_robot!2
* [dingo_base] roslint fixes.
* [dingo_base] Fixed mag_config_default.yaml location.
* [dingo_base] Updated mag config params.
* [dingo_base] Moved network logger before rosserial server.
* [dingo_base] Updated logger to use ROS based logging for its own logging.
* [dingo_base] Updated CAN IDs and directions for joints.
* [dingo_base] Added missing join for logger thread.
* [dingo_base] Flipped motor signs.
* [dinog_hardware] Fixed typo.
* [dingo_base] Added network based logger.
* [dingo_lighting] Updated drive lighting patterns.
* [dingo_hardware] Reduced logging by throttling.
* [dingo_hardware] Updated motor parameters.
* [dingo_base] Reworkd Dingo model selection.
* Swapped dependency location since it is used by dingo_bringup and not dingo_base.
* Unified dingo launch files.  Also, removed connman file in install.
* Merge branch 'jh-melodic-devel' into 'melodic-devel'
  Initial Dingo robot implementation; not yet tested on real board
  See merge request dingo/dingo_robot!1
* Initial Dingo robot implementation; not yet tested on real board
* Contributors: Chris Iverach-Brereton, Jason Higgins, Tony Baltovski
