# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/shivamss/colcon_ws/src/orbslam3_ros2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/shivamss/colcon_ws/build/orbslam3

# Include any dependencies generated for this target.
include CMakeFiles/mono.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/mono.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/mono.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mono.dir/flags.make

CMakeFiles/mono.dir/src/monocular/mono.cpp.o: CMakeFiles/mono.dir/flags.make
CMakeFiles/mono.dir/src/monocular/mono.cpp.o: /home/shivamss/colcon_ws/src/orbslam3_ros2/src/monocular/mono.cpp
CMakeFiles/mono.dir/src/monocular/mono.cpp.o: CMakeFiles/mono.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shivamss/colcon_ws/build/orbslam3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/mono.dir/src/monocular/mono.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/mono.dir/src/monocular/mono.cpp.o -MF CMakeFiles/mono.dir/src/monocular/mono.cpp.o.d -o CMakeFiles/mono.dir/src/monocular/mono.cpp.o -c /home/shivamss/colcon_ws/src/orbslam3_ros2/src/monocular/mono.cpp

CMakeFiles/mono.dir/src/monocular/mono.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mono.dir/src/monocular/mono.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shivamss/colcon_ws/src/orbslam3_ros2/src/monocular/mono.cpp > CMakeFiles/mono.dir/src/monocular/mono.cpp.i

CMakeFiles/mono.dir/src/monocular/mono.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mono.dir/src/monocular/mono.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shivamss/colcon_ws/src/orbslam3_ros2/src/monocular/mono.cpp -o CMakeFiles/mono.dir/src/monocular/mono.cpp.s

CMakeFiles/mono.dir/src/monocular/monocular-slam-node.cpp.o: CMakeFiles/mono.dir/flags.make
CMakeFiles/mono.dir/src/monocular/monocular-slam-node.cpp.o: /home/shivamss/colcon_ws/src/orbslam3_ros2/src/monocular/monocular-slam-node.cpp
CMakeFiles/mono.dir/src/monocular/monocular-slam-node.cpp.o: CMakeFiles/mono.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shivamss/colcon_ws/build/orbslam3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/mono.dir/src/monocular/monocular-slam-node.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/mono.dir/src/monocular/monocular-slam-node.cpp.o -MF CMakeFiles/mono.dir/src/monocular/monocular-slam-node.cpp.o.d -o CMakeFiles/mono.dir/src/monocular/monocular-slam-node.cpp.o -c /home/shivamss/colcon_ws/src/orbslam3_ros2/src/monocular/monocular-slam-node.cpp

CMakeFiles/mono.dir/src/monocular/monocular-slam-node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mono.dir/src/monocular/monocular-slam-node.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shivamss/colcon_ws/src/orbslam3_ros2/src/monocular/monocular-slam-node.cpp > CMakeFiles/mono.dir/src/monocular/monocular-slam-node.cpp.i

CMakeFiles/mono.dir/src/monocular/monocular-slam-node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mono.dir/src/monocular/monocular-slam-node.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shivamss/colcon_ws/src/orbslam3_ros2/src/monocular/monocular-slam-node.cpp -o CMakeFiles/mono.dir/src/monocular/monocular-slam-node.cpp.s

# Object files for target mono
mono_OBJECTS = \
"CMakeFiles/mono.dir/src/monocular/mono.cpp.o" \
"CMakeFiles/mono.dir/src/monocular/monocular-slam-node.cpp.o"

# External object files for target mono
mono_EXTERNAL_OBJECTS =

mono: CMakeFiles/mono.dir/src/monocular/mono.cpp.o
mono: CMakeFiles/mono.dir/src/monocular/monocular-slam-node.cpp.o
mono: CMakeFiles/mono.dir/build.make
mono: /opt/ros/humble/lib/libcv_bridge.so
mono: /opt/ros/humble/lib/libmessage_filters.so
mono: /home/shivamss/Dev/ORB_SLAM3/lib/libORB_SLAM3.so
mono: /home/shivamss/Dev/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so
mono: /home/shivamss/Dev/ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so
mono: /usr/local/lib/libpango_glgeometry.so
mono: /usr/local/lib/libpango_plot.so
mono: /usr/local/lib/libpango_python.so
mono: /usr/local/lib/libpango_scene.so
mono: /usr/local/lib/libpango_tools.so
mono: /usr/local/lib/libpango_video.so
mono: /usr/local/lib/libopencv_gapi.so.4.2.0
mono: /usr/local/lib/libopencv_stitching.so.4.2.0
mono: /usr/local/lib/libopencv_aruco.so.4.2.0
mono: /usr/local/lib/libopencv_bgsegm.so.4.2.0
mono: /usr/local/lib/libopencv_bioinspired.so.4.2.0
mono: /usr/local/lib/libopencv_ccalib.so.4.2.0
mono: /usr/local/lib/libopencv_dnn_objdetect.so.4.2.0
mono: /usr/local/lib/libopencv_dnn_superres.so.4.2.0
mono: /usr/local/lib/libopencv_dpm.so.4.2.0
mono: /usr/local/lib/libopencv_face.so.4.2.0
mono: /usr/local/lib/libopencv_freetype.so.4.2.0
mono: /usr/local/lib/libopencv_fuzzy.so.4.2.0
mono: /usr/local/lib/libopencv_hdf.so.4.2.0
mono: /usr/local/lib/libopencv_hfs.so.4.2.0
mono: /usr/local/lib/libopencv_img_hash.so.4.2.0
mono: /usr/local/lib/libopencv_line_descriptor.so.4.2.0
mono: /usr/local/lib/libopencv_quality.so.4.2.0
mono: /usr/local/lib/libopencv_reg.so.4.2.0
mono: /usr/local/lib/libopencv_rgbd.so.4.2.0
mono: /usr/local/lib/libopencv_saliency.so.4.2.0
mono: /usr/local/lib/libopencv_sfm.so.4.2.0
mono: /usr/local/lib/libopencv_stereo.so.4.2.0
mono: /usr/local/lib/libopencv_structured_light.so.4.2.0
mono: /usr/local/lib/libopencv_superres.so.4.2.0
mono: /usr/local/lib/libopencv_surface_matching.so.4.2.0
mono: /usr/local/lib/libopencv_tracking.so.4.2.0
mono: /usr/local/lib/libopencv_videostab.so.4.2.0
mono: /usr/local/lib/libopencv_xfeatures2d.so.4.2.0
mono: /usr/local/lib/libopencv_xobjdetect.so.4.2.0
mono: /usr/local/lib/libopencv_xphoto.so.4.2.0
mono: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_fastrtps_c.so
mono: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_c.so
mono: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_fastrtps_c.so
mono: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_fastrtps_cpp.so
mono: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_cpp.so
mono: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_fastrtps_cpp.so
mono: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/humble/lib/libsensor_msgs__rosidl_generator_py.so
mono: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_c.so
mono: /opt/ros/humble/lib/libsensor_msgs__rosidl_generator_c.so
mono: /opt/ros/humble/lib/libgeometry_msgs__rosidl_generator_py.so
mono: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_c.so
mono: /opt/ros/humble/lib/libgeometry_msgs__rosidl_generator_c.so
mono: /opt/ros/humble/lib/libstd_msgs__rosidl_generator_py.so
mono: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_c.so
mono: /opt/ros/humble/lib/libstd_msgs__rosidl_generator_c.so
mono: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/humble/lib/librclcpp.so
mono: /opt/ros/humble/lib/liblibstatistics_collector.so
mono: /opt/ros/humble/lib/librcl.so
mono: /opt/ros/humble/lib/librmw_implementation.so
mono: /opt/ros/humble/lib/libament_index_cpp.so
mono: /opt/ros/humble/lib/librcl_logging_spdlog.so
mono: /opt/ros/humble/lib/librcl_logging_interface.so
mono: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so
mono: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
mono: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_fastrtps_cpp.so
mono: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_cpp.so
mono: /opt/ros/humble/lib/librcl_interfaces__rosidl_generator_py.so
mono: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_c.so
mono: /opt/ros/humble/lib/librcl_interfaces__rosidl_generator_c.so
mono: /opt/ros/humble/lib/librcl_yaml_param_parser.so
mono: /opt/ros/humble/lib/libyaml.so
mono: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_c.so
mono: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_cpp.so
mono: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/humble/lib/librosgraph_msgs__rosidl_generator_py.so
mono: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_c.so
mono: /opt/ros/humble/lib/librosgraph_msgs__rosidl_generator_c.so
mono: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_c.so
mono: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_c.so
mono: /opt/ros/humble/lib/librosidl_typesupport_fastrtps_c.so
mono: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_cpp.so
mono: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_cpp.so
mono: /opt/ros/humble/lib/librosidl_typesupport_fastrtps_cpp.so
mono: /opt/ros/humble/lib/libfastcdr.so.1.0.24
mono: /opt/ros/humble/lib/librmw.so
mono: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
mono: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/humble/lib/librosidl_typesupport_introspection_cpp.so
mono: /opt/ros/humble/lib/librosidl_typesupport_introspection_c.so
mono: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
mono: /opt/ros/humble/lib/librosidl_typesupport_cpp.so
mono: /opt/ros/humble/lib/libstatistics_msgs__rosidl_generator_py.so
mono: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_generator_py.so
mono: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_c.so
mono: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
mono: /opt/ros/humble/lib/libstatistics_msgs__rosidl_generator_c.so
mono: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_generator_c.so
mono: /opt/ros/humble/lib/librosidl_typesupport_c.so
mono: /opt/ros/humble/lib/librcpputils.so
mono: /opt/ros/humble/lib/librosidl_runtime_c.so
mono: /usr/lib/x86_64-linux-gnu/libpython3.10.so
mono: /opt/ros/humble/lib/libtracetools.so
mono: /opt/ros/humble/lib/librcutils.so
mono: /usr/local/lib/libpango_geometry.so
mono: /usr/local/lib/libtinyobj.so
mono: /usr/local/lib/libpango_display.so
mono: /usr/local/lib/libpango_vars.so
mono: /usr/local/lib/libpango_windowing.so
mono: /usr/local/lib/libpango_opengl.so
mono: /usr/lib/x86_64-linux-gnu/libGLEW.so
mono: /usr/lib/x86_64-linux-gnu/libOpenGL.so
mono: /usr/lib/x86_64-linux-gnu/libGLX.so
mono: /usr/lib/x86_64-linux-gnu/libGLU.so
mono: /usr/local/lib/libpango_image.so
mono: /usr/local/lib/libpango_packetstream.so
mono: /usr/local/lib/libpango_core.so
mono: /usr/local/lib/libopencv_highgui.so.4.2.0
mono: /usr/local/lib/libopencv_shape.so.4.2.0
mono: /usr/local/lib/libopencv_datasets.so.4.2.0
mono: /usr/local/lib/libopencv_plot.so.4.2.0
mono: /usr/local/lib/libopencv_text.so.4.2.0
mono: /usr/local/lib/libopencv_dnn.so.4.2.0
mono: /usr/local/lib/libopencv_ml.so.4.2.0
mono: /usr/local/lib/libopencv_phase_unwrapping.so.4.2.0
mono: /usr/local/lib/libopencv_optflow.so.4.2.0
mono: /usr/local/lib/libopencv_ximgproc.so.4.2.0
mono: /usr/local/lib/libopencv_video.so.4.2.0
mono: /usr/local/lib/libopencv_videoio.so.4.2.0
mono: /usr/local/lib/libopencv_imgcodecs.so.4.2.0
mono: /usr/local/lib/libopencv_objdetect.so.4.2.0
mono: /usr/local/lib/libopencv_calib3d.so.4.2.0
mono: /usr/local/lib/libopencv_features2d.so.4.2.0
mono: /usr/local/lib/libopencv_flann.so.4.2.0
mono: /usr/local/lib/libopencv_photo.so.4.2.0
mono: /usr/local/lib/libopencv_imgproc.so.4.2.0
mono: /usr/local/lib/libopencv_core.so.4.2.0
mono: CMakeFiles/mono.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/shivamss/colcon_ws/build/orbslam3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable mono"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mono.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mono.dir/build: mono
.PHONY : CMakeFiles/mono.dir/build

CMakeFiles/mono.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mono.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mono.dir/clean

CMakeFiles/mono.dir/depend:
	cd /home/shivamss/colcon_ws/build/orbslam3 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shivamss/colcon_ws/src/orbslam3_ros2 /home/shivamss/colcon_ws/src/orbslam3_ros2 /home/shivamss/colcon_ws/build/orbslam3 /home/shivamss/colcon_ws/build/orbslam3 /home/shivamss/colcon_ws/build/orbslam3/CMakeFiles/mono.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mono.dir/depend

