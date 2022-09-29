^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dingo_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.9 (2021-07-16)
------------------
* Change where we load the joy_dev argument so it loads correctly
* Contributors: Chris Iverach-Brereton

0.1.8 (2021-05-11)
------------------
* Update the axes to match the latest DS4DRV update; the left stick is now axes 3/4.
* Contributors: Chris Iverach-Brereton

0.1.7 (2021-03-08)
------------------

0.1.6 (2020-11-26)
------------------
* [dingo_control] Increased acceleration limit for omnidirectional platform.
* Contributors: Tony Baltovski

0.1.5 (2020-10-30)
------------------
* [dingo_control] Reduced acceleration limit for omnidirectional platform.
* Contributors: Tony Baltovski

0.1.4 (2020-10-26)
------------------
* [dingo_control] Fixed angular axis.
* Contributors: Tony Baltovski

0.1.3 (2020-09-29)
------------------
* Remove the 0/1 DINGO_CONTROL_EXTRAS + DINGO_CONTROL_EXTRAS_PATH, simplify to just DINGO_CONFIG_EXTRAS, like Husky & Warthog. (`#6 <https://github.com/dingo-cpr/dingo/issues/6>`_)
* [dingo_control] Reduced linear acceleration limits.
* Contributors: Chris I-B, Tony Baltovski

0.1.2 (2020-09-25)
------------------
* [dingo_control] Reduced Dingo-O accelerations.
* [dingo_control] Reduced line length.
* [dingo_control] Removed linear accelerations from EKF.
* Contributors: Tony Baltovski

0.1.1 (2020-09-14)
------------------
* Improve joy device configurability (`#4 <https://github.com/dingo-cpr/dingo/issues/4>`_)
  * Remove the joy device from the yaml file, but keep it as the default in the launch file. Add support for a DINGO_JOY_DEV environment variable to override the default joy device.
  * Add the ASCII art diagram of the controller from Husky to make it easier to interpret the button mappings, should anyone want to remap them.
  * Expose the yaml file as an additional argument to the teleop launch file, allow overriding it with DINGO_JOY_CONFIG
* [dingo_control] Added limits for y motion omni platform.
* [dingo_control] Updated max velocity and acceleration limits.
* [dingo_control] Updated robot_localization config.
* [dingo_control] Moved control extras to botton of launch file.
* Contributors: Chris I-B, Tony Baltovski

0.1.0 (2020-08-10)
------------------
* Fix the controller parameters for omni control; behaviour is now the same as ridgeback
* Unified Dingo folders/files as much as possible, updated meshes, bumped CMake version and made roslaunch a test depend.
* Added dingo_control; split dingo_description into separate Dingo-D and Dingo-O flavours; still much work to be done here, but preparing hand-off for Tony
* Contributors: Chris Iverach-Brereton, Jason Higgins, Tony Baltovski
