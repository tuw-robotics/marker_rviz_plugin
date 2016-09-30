^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package marker_rviz_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2016-09-30)
------------------
* Fixed covariance orientation bugs. It seems to work correctly now. (At least for the few axis used in the stage and gazebo slam demo.)
* Added rviz config for send_demo_msg.py demo script.
* Code cleanup.
* Added scaled cylinders as visuals for orientational variance.
* Array out of bounds fix.
* Copy paste bug fix
* Implemented marker_with_covariance_array display.
* Implemented marker with covariance. Currently only position covariance is displayed. No orientation yet.
* Implemented marker label to show marker id.
* Added demo data script for plugin testing.
* The visual marker (and most of the ogre specific parts) are now defined in marker.cpp for easier reuse in the upcoming covariance code. Segfault is also fixed now.
* Merge branch 'devel' of github.com:tuw-robotics/marker_rviz_plugin into devel
* Added details regarding the segfault.
* Working marker visualization. There are definitly memory leaks somewhere in there and rviz does quit with segfault if exit is used.
* Added two properties to hide or show visuals.
* Added axes as orientation indicator and fixed plane orientation.
* Cleanup. Removed backface culling.
* Trying to render a few basic ogre objects (including meshes and textures).
* Added description in package.xml
* CMakeList.txt changed to work with Qt5 and Qt4
* Introduced rviz plugins for marker_msgs::MarkerWithCovarianceStamped.msg and marker_msgs::MarkerWithCovarianceArray.msg
* plugin working
* first commit
* Initial commit
* Contributors: Markus Bader, doctorseus, mmacsek
