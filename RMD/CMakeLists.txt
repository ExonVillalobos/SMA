cmake_minimum_required(VERSION 2.8.3)

project(RMD)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
  urdf
  xacro)
  
find_package(roslaunch)

catkin_package()

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


catkin_install_python(PROGRAMS
  scripts/controlh1.py
  scripts/controlh2.py
  scripts/controlh3.py
  scripts/controlh4.py
  scripts/controlh5.py
  scripts/controlh6.py
  scripts/controlh7.py
  scripts/controlh8.py
  scripts/controlh9.py
  scripts/controlh10.py
  scripts/controlc1.py
  scripts/controlc2.py
  scripts/controlc3.py
  scripts/controlc4.py
  scripts/controlc5.py
  scripts/controlc6.py
  scripts/controlc7.py
  scripts/controlc8.py
  scripts/controlc9.py
  scripts/controlc10.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
