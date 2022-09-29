^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dingo_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.3 (2021-03-08)
------------------
* Move the VLP16 to a new DINGO_LASER_3D family of vars, add DINGO_LASER_SECONDARY
* Add vlp16 to the commend block explaining allowed values
* Add support for the VLP16 as a standard laser sensor for Dingo
* Contributors: Chris Iverach-Brereton

0.1.2 (2020-11-26)
------------------
* [dingo_bringup] Made ros service start after can-udp-bridge service.
* Contributors: Tony Baltovski

0.1.1 (2020-11-23)
------------------
* Added missing test dependencies.
* Contributors: Tony Baltovski

0.1.0 (2020-11-13)
------------------
* Bump CMake version to avoid CMP0048 warning.
* [dingo_bringup] Increased CAN TX queue size.
* Merge pull request `#1 <https://github.com/dingo-cpr/dingo_robot/issues/1>`_ from dingo-cpr/rs-l515-update
  Remove the work-around for the RealSense L515 pointcloud data
* Remove the work-around for the RealSense L515 pointcloud data; the driver's been updated and this is no longer needed.
* [dingo_bringup] Added missing export.
* Merge branch 'no-realsense-to-laser' into 'melodic-devel'
  Remove the realsense's depthimage-to-laserscan node
  See merge request dingo/dingo_robot!4
* Remove the realsense's depthimage-to-laserscan node; the realsense is not well-suited to use as a lidar, so just remove this
* Move the depthimage to laserscan node inside the right group
* Merge branch 'melodic-devel' of gitlab.clearpathrobotics.com:dingo/dingo_robot into melodic-devel
* Add the missing enable_infra flag that's necessary to get the L515 to publish data correctly
* Merge branch 'tb-rework' into 'melodic-devel'
  Initial rework
  See merge request dingo/dingo_robot!2
* [dingo_bringup] Symlinked launch files.
* [dingo_bringup] Update instrall script and made it executable.
* [dingo_bringup] Set dingo_bringup as executable.
* [dingo_bringup] Maded script executable.
* Swapped dependency location since it is used by dingo_bringup and not dingo_base.
* [dingo_bringup] Added script to set environment variable for dingo-o
* Unified dingo launch files.  Also, removed connman file in install.
* Merge branch 'cib-accessories' into 'melodic-devel'
  Enable Hokuyo Lidar & Realsense in dingo_bringup
  See merge request dingo/dingo_robot!3
* Enable Hokuyo Lidar & Realsense in dingo_bringup
* Merge branch 'jh-melodic-devel' into 'melodic-devel'
  Initial Dingo robot implementation; not yet tested on real board
  See merge request dingo/dingo_robot!1
* Initial Dingo robot implementation; not yet tested on real board
* Contributors: Chris Iverach-Brereton, Jason Higgins, Tony Baltovski
