^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dingo_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.9 (2021-07-16)
------------------

0.1.8 (2021-05-11)
------------------

0.1.7 (2021-03-08)
------------------

0.1.6 (2020-11-26)
------------------

0.1.5 (2020-10-30)
------------------

0.1.4 (2020-10-26)
------------------

0.1.3 (2020-09-29)
------------------
* Make the default scan topic for gmapping + amcl use the env vars (`#7 <https://github.com/dingo-cpr/dingo/issues/7>`_)
  Load the DINGO_LASER_TOPIC env var as the default scan topic for nav demos
* Contributors: Chris I-B

0.1.2 (2020-09-25)
------------------

0.1.1 (2020-09-14)
------------------
* Nav improvements (`#5 <https://github.com/dingo-cpr/dingo/issues/5>`_)
  * Expose the scan_topic argument in the gmapping_demo and amcl_demo launch files
  * Add placeholder support for the RS L515 and D455 so that the gazebo plugins work; the meshes for these sensors don't exist yet, but we can at least get the plugin configured & add the appropriate links for now
  * Refactor the RealSense macro to put the mesh + gazebo plugin in one place. Create a more-accurate L515 mesh out of cylinders until Intel releases an official mesh for the sensor
* Contributors: Chris I-B

0.1.0 (2020-08-10)
------------------
* [dingo_navigation] Removed maps from install.
* [dingo_navigation] Unified launch files, made roslaunch a test depend and bumped CMake version.
* Added dingo_control; split dingo_description into separate Dingo-D and Dingo-O flavours; still much work to be done here, but preparing hand-off for Tony
* Initial commit of dingo_navigation
* Contributors: Dave Niewinski, Jason Higgins, Tony Baltovski
