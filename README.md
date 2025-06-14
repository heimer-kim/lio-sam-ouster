# 📁 Project Summary - src

## 📂 Directory Structure
```
.
├── .vscode
│   └── settings.json
├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
├── LIO-SAM
│   ├── .git
│   │   ├── HEAD
│   │   ├── branches
│   │   ├── config
│   │   ├── description
│   │   ├── hooks
│   │   │   ├── applypatch-msg.sample
│   │   │   ├── commit-msg.sample
│   │   │   ├── fsmonitor-watchman.sample
│   │   │   ├── post-update.sample
│   │   │   ├── pre-applypatch.sample
│   │   │   ├── pre-commit.sample
│   │   │   ├── pre-merge-commit.sample
│   │   │   ├── pre-push.sample
│   │   │   ├── pre-rebase.sample
│   │   │   ├── pre-receive.sample
│   │   │   ├── prepare-commit-msg.sample
│   │   │   └── update.sample
│   │   ├── index
│   │   ├── info
│   │   │   └── exclude
│   │   ├── logs
│   │   │   ├── HEAD
│   │   │   └── refs
│   │   │       ├── heads
│   │   │       │   └── master
│   │   │       └── remotes
│   │   │           └── origin
│   │   │               └── HEAD
│   │   ├── objects
│   │   │   ├── info
│   │   │   └── pack
│   │   │       ├── pack-865635e87748a9f6b5f14367cfd4686ebc7a7500.idx
│   │   │       └── pack-865635e87748a9f6b5f14367cfd4686ebc7a7500.pack
│   │   ├── packed-refs
│   │   └── refs
│   │       ├── heads
│   │       │   └── master
│   │       ├── remotes
│   │       │   └── origin
│   │       │       └── HEAD
│   │       └── tags
│   ├── .github
│   │   └── stale.yml
│   ├── CMakeLists.txt
│   ├── Dockerfile
│   ├── LICENSE
│   ├── README.md
│   ├── campus_small_dataset.bag
│   ├── config
│   │   ├── doc
│   │   │   ├── demo.gif
│   │   │   ├── device-boat.png
│   │   │   ├── device-hand-2.png
│   │   │   ├── device-hand.png
│   │   │   ├── device-jackal.png
│   │   │   ├── device-livox-horizon.png
│   │   │   ├── gps-demo.gif
│   │   │   ├── imu-debug.gif
│   │   │   ├── imu-transform.png
│   │   │   ├── kitti-demo.gif
│   │   │   ├── kitti-map.png
│   │   │   ├── kitti2bag
│   │   │   │   ├── README.md
│   │   │   │   └── kitti2bag.py
│   │   │   ├── livox-demo.gif
│   │   │   ├── loop-closure-2.gif
│   │   │   ├── loop-closure.gif
│   │   │   ├── ouster-demo.gif
│   │   │   ├── ouster-device.jpg
│   │   │   ├── paper.pdf
│   │   │   └── system.png
│   │   └── params.yaml
│   ├── include
│   │   └── utility.h
│   ├── launch
│   │   ├── include
│   │   │   ├── config
│   │   │   │   ├── robot.urdf.xacro
│   │   │   │   └── rviz.rviz
│   │   │   ├── module_loam.launch
│   │   │   ├── module_navsat.launch
│   │   │   ├── module_robot_state_publisher.launch
│   │   │   ├── module_rviz.launch
│   │   │   └── rosconsole
│   │   │       ├── rosconsole_error.conf
│   │   │       ├── rosconsole_info.conf
│   │   │       └── rosconsole_warn.conf
│   │   └── run.launch
│   ├── msg
│   │   └── cloud_info.msg
│   ├── package.xml
│   ├── remove_nan_filter.py
│   ├── src
│   │   ├── featureExtraction.cpp
│   │   ├── imageProjection.cpp
│   │   ├── imuPreintegration.cpp
│   │   ├── lidar_imu_sync.cpp
│   │   └── mapOptmization.cpp
│   ├── srv
│   │   └── save_map.srv
│   └── timestamp_check.py
├── generate_project_summary.sh
├── ouster-ros
│   ├── .git
│   │   ├── HEAD
│   │   ├── branches
│   │   ├── config
│   │   ├── description
│   │   ├── hooks
│   │   │   ├── applypatch-msg.sample
│   │   │   ├── commit-msg.sample
│   │   │   ├── fsmonitor-watchman.sample
│   │   │   ├── post-update.sample
│   │   │   ├── pre-applypatch.sample
│   │   │   ├── pre-commit.sample
│   │   │   ├── pre-merge-commit.sample
│   │   │   ├── pre-push.sample
│   │   │   ├── pre-rebase.sample
│   │   │   ├── pre-receive.sample
│   │   │   ├── prepare-commit-msg.sample
│   │   │   └── update.sample
│   │   ├── index
│   │   ├── info
│   │   │   └── exclude
│   │   ├── logs
│   │   │   ├── HEAD
│   │   │   └── refs
│   │   │       ├── heads
│   │   │       │   └── master
│   │   │       └── remotes
│   │   │           └── origin
│   │   │               └── HEAD
│   │   ├── modules
│   │   │   └── ouster-sdk
│   │   │       ├── FETCH_HEAD
│   │   │       ├── HEAD
│   │   │       ├── branches
│   │   │       ├── config
│   │   │       ├── description
│   │   │       ├── hooks
│   │   │       │   ├── applypatch-msg.sample
│   │   │       │   ├── commit-msg.sample
│   │   │       │   ├── fsmonitor-watchman.sample
│   │   │       │   ├── post-update.sample
│   │   │       │   ├── pre-applypatch.sample
│   │   │       │   ├── pre-commit.sample
│   │   │       │   ├── pre-merge-commit.sample
│   │   │       │   ├── pre-push.sample
│   │   │       │   ├── pre-rebase.sample
│   │   │       │   ├── pre-receive.sample
│   │   │       │   ├── prepare-commit-msg.sample
│   │   │       │   └── update.sample
│   │   │       ├── index
│   │   │       ├── info
│   │   │       │   └── exclude
│   │   │       ├── logs
│   │   │       │   ├── HEAD
│   │   │       │   └── refs
│   │   │       │       ├── heads
│   │   │       │       │   └── master
│   │   │       │       └── remotes
│   │   │       │           └── origin
│   │   │       │               └── HEAD
│   │   │       ├── objects
│   │   │       │   ├── info
│   │   │       │   └── pack
│   │   │       │       ├── pack-555bd747ebe908db71c5bf5095823069e4a3cd58.idx
│   │   │       │       ├── pack-555bd747ebe908db71c5bf5095823069e4a3cd58.pack
│   │   │       │       ├── pack-b6974a06cef6f158b2bb5ed247e4619af0d26fa8.idx
│   │   │       │       └── pack-b6974a06cef6f158b2bb5ed247e4619af0d26fa8.pack
│   │   │       ├── packed-refs
│   │   │       └── refs
│   │   │           ├── heads
│   │   │           │   └── master
│   │   │           ├── remotes
│   │   │           │   └── origin
│   │   │           │       └── HEAD
│   │   │           └── tags
│   │   ├── objects
│   │   │   ├── info
│   │   │   └── pack
│   │   │       ├── pack-9ec8f360098866d9ffe0b9ca3ee4e06771e51e79.idx
│   │   │       └── pack-9ec8f360098866d9ffe0b9ca3ee4e06771e51e79.pack
│   │   ├── packed-refs
│   │   └── refs
│   │       ├── heads
│   │       │   └── master
│   │       ├── remotes
│   │       │   └── origin
│   │       │       └── HEAD
│   │       └── tags
│   ├── .github
│   │   ├── ISSUE_TEMPLATE
│   │   │   ├── bug_report.md
│   │   │   └── feature_request.md
│   │   ├── pull_request_template.md
│   │   └── workflows
│   │       └── docker-image.yml
│   ├── .gitmodules
│   ├── CHANGELOG.rst
│   ├── CMakeLists.txt
│   ├── Dockerfile
│   ├── LICENSE
│   ├── README.md
│   ├── config
│   │   └── viz.rviz
│   ├── docs
│   │   ├── images
│   │   │   └── logo.png
│   │   ├── index.rst
│   │   └── migration-guide.rst
│   ├── include
│   │   └── ouster_ros
│   │       ├── common_point_types.h
│   │       ├── os_point.h
│   │       ├── os_ros.h
│   │       ├── os_sensor_nodelet_base.h
│   │       └── sensor_point_types.h
│   ├── launch
│   │   ├── common.launch
│   │   ├── driver.launch
│   │   ├── record.launch
│   │   ├── replay.launch
│   │   ├── replay_pcap.launch
│   │   ├── sensor.launch
│   │   └── sensor_mtp.launch
│   ├── msg
│   │   ├── PacketMsg.msg
│   │   └── Telemetry.msg
│   ├── ouster-sdk
│   │   ├── .clang-format
│   │   ├── .clangd
│   │   ├── .dockerignore
│   │   ├── .git
│   │   ├── .github
│   │   │   ├── ISSUE_TEMPLATE
│   │   │   │   ├── bug_report.md
│   │   │   │   ├── feature_request.md
│   │   │   │   └── question.md
│   │   │   ├── check_title_and_description.py
│   │   │   ├── pull_request_template.md
│   │   │   └── workflows
│   │   │       └── check-commit-message.yml
│   │   ├── .gitignore
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── CMakeSettings.json
│   │   ├── LICENSE
│   │   ├── LICENSE-bin
│   │   ├── README.rst
│   │   ├── clang-linting.sh
│   │   ├── cmake
│   │   │   ├── Coverage.cmake
│   │   │   ├── DefaultBuildType.cmake
│   │   │   ├── FindCURL.cmake
│   │   │   ├── FindEigen3.cmake
│   │   │   ├── FindGTest.cmake
│   │   │   ├── FindOusterSDK.cmake
│   │   │   ├── FindPcap.cmake
│   │   │   ├── FindPybind11Internal.cmake
│   │   │   ├── Findglfw3.cmake
│   │   │   ├── Findjsoncpp.cmake
│   │   │   ├── Findlibtins.cmake
│   │   │   ├── OusterSDKConfig.cmake.in
│   │   │   ├── VcpkgEnv.cmake
│   │   │   ├── VersionGen.cmake
│   │   │   └── build.h.in
│   │   ├── conan
│   │   │   └── test_package
│   │   │       ├── CMakeLists.txt
│   │   │       └── conanfile.py
│   │   ├── conanfile.py
│   │   ├── docs
│   │   │   ├── Doxyfile
│   │   │   ├── _static
│   │   │   │   └── css
│   │   │   │       └── ouster_rtd_tweaks.css
│   │   │   ├── _templates
│   │   │   │   └── versions.html
│   │   │   ├── cli
│   │   │   │   ├── common-use-cases.rst
│   │   │   │   ├── getting-started.rst
│   │   │   │   ├── mapping-sessions.rst
│   │   │   │   ├── overview.rst
│   │   │   │   └── sample-sessions.rst
│   │   │   ├── conf.py
│   │   │   ├── cpp
│   │   │   │   ├── api.rst
│   │   │   │   ├── building.rst
│   │   │   │   ├── examples
│   │   │   │   │   ├── index.rst
│   │   │   │   │   └── simple_examples.rst
│   │   │   │   ├── ouster_client
│   │   │   │   │   ├── client.rst
│   │   │   │   │   ├── image_processing.rst
│   │   │   │   │   ├── index.rst
│   │   │   │   │   ├── lidar_scan.rst
│   │   │   │   │   ├── types.rst
│   │   │   │   │   └── version.rst
│   │   │   │   ├── ouster_osf
│   │   │   │   │   ├── basics.rst
│   │   │   │   │   ├── crc32.rst
│   │   │   │   │   ├── file.rst
│   │   │   │   │   ├── index.rst
│   │   │   │   │   ├── layout_streaming.rst
│   │   │   │   │   ├── meta_extrinsics.rst
│   │   │   │   │   ├── meta_lidar_sensor.rst
│   │   │   │   │   ├── meta_streaming_info.rst
│   │   │   │   │   ├── metadata.rst
│   │   │   │   │   ├── operations.rst
│   │   │   │   │   ├── pcap_source.rst
│   │   │   │   │   ├── reader.rst
│   │   │   │   │   ├── stream_lidar_scan.rst
│   │   │   │   │   └── writer.rst
│   │   │   │   ├── ouster_pcap
│   │   │   │   │   ├── index.rst
│   │   │   │   │   └── os_pcap.rst
│   │   │   │   └── ouster_viz
│   │   │   │       ├── index.rst
│   │   │   │       └── point_viz.rst
│   │   │   ├── images
│   │   │   │   ├── Ouster_Logo_TM_Horiz_Black_RGB_600px.png
│   │   │   │   ├── Ouster_Logo_TM_Horiz_White_RGB.svg
│   │   │   │   ├── Ouster_Logo_TM_Stacked_White_RGB.svg
│   │   │   │   ├── brooklyn_bridge_ls_50_range_image.png
│   │   │   │   ├── brooklyn_bridge_ls_50_xyz_cut.png
│   │   │   │   ├── lidar_scan_destaggered.png
│   │   │   │   ├── lidar_scan_staggered.png
│   │   │   │   ├── lidar_scan_xyz_84.png
│   │   │   │   ├── lidar_scan_xyz_84_3d.png
│   │   │   │   ├── ouster-viz.png
│   │   │   │   ├── scans_accum_accum_scan.png
│   │   │   │   ├── scans_accum_dense_every.png
│   │   │   │   ├── scans_accum_map_all_scan.png
│   │   │   │   ├── scans_accum_track_all.png
│   │   │   │   └── viz-tutorial
│   │   │   │       ├── axes_helper_unstructured.png
│   │   │   │       ├── empty_point_viz.png
│   │   │   │       ├── image_set_pos_bot_right.png
│   │   │   │       ├── image_set_pos_center.png
│   │   │   │       ├── image_set_pos_left.png
│   │   │   │       ├── image_set_pos_right.png
│   │   │   │       ├── lidar_scan_fields_images.png
│   │   │   │       ├── lidar_scan_fields_images_labels.png
│   │   │   │       ├── lidar_scan_structured.png
│   │   │   │       ├── lidar_scan_viz.png
│   │   │   │       ├── lidar_scan_viz_checkers.png
│   │   │   │       └── lidar_scan_viz_labels.png
│   │   │   ├── index.rst
│   │   │   ├── installation.rst
│   │   │   ├── migration
│   │   │   │   ├── migration-20220927-20230114.rst
│   │   │   │   ├── migration-20230114-20230403.rst
│   │   │   │   └── migration-20231031-20240423.rst
│   │   │   ├── overview.rst
│   │   │   ├── python
│   │   │   │   ├── api
│   │   │   │   │   ├── client.rst
│   │   │   │   │   ├── examples.rst
│   │   │   │   │   ├── index.rst
│   │   │   │   │   ├── osf.rst
│   │   │   │   │   ├── pcap.rst
│   │   │   │   │   └── viz.rst
│   │   │   │   ├── devel.rst
│   │   │   │   ├── examples
│   │   │   │   │   ├── basics-sensor.rst
│   │   │   │   │   ├── conversion.rst
│   │   │   │   │   ├── index.rst
│   │   │   │   │   ├── lidar-scan.rst
│   │   │   │   │   ├── osf-examples.rst
│   │   │   │   │   ├── record-stream.rst
│   │   │   │   │   ├── udp-packets.rst
│   │   │   │   │   └── visualizations.rst
│   │   │   │   ├── quickstart.rst
│   │   │   │   ├── slam-api-example.rst
│   │   │   │   ├── using-scan-source.rst
│   │   │   │   └── viz
│   │   │   │       ├── index.rst
│   │   │   │       ├── viz-api-tutorial.rst
│   │   │   │       ├── viz-run.rst
│   │   │   │       └── viz-scans-accum.rst
│   │   │   ├── reference
│   │   │   │   ├── changelog.rst
│   │   │   │   ├── lidar-scan.rst
│   │   │   │   └── osf.rst
│   │   │   ├── sample-data.rst
│   │   │   └── versions.json
│   │   ├── examples
│   │   │   ├── CMakeLists.txt
│   │   │   ├── async_client_example.cpp
│   │   │   ├── client_example.cpp
│   │   │   ├── config_example.cpp
│   │   │   ├── helpers.cpp
│   │   │   ├── helpers.h
│   │   │   ├── lidar_scan_example.cpp
│   │   │   ├── linking_example
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── Dockerfile
│   │   │   │   ├── example.bash
│   │   │   │   ├── main.cpp
│   │   │   │   └── readme.rst
│   │   │   ├── mtp_client_example.cpp
│   │   │   ├── osf_reader_example.cpp
│   │   │   ├── osf_writer_example.cpp
│   │   │   ├── representations_example.cpp
│   │   │   └── viz_example.cpp
│   │   ├── ouster_client
│   │   │   ├── CMakeLists.txt
│   │   │   ├── include
│   │   │   │   ├── optional-lite
│   │   │   │   │   └── nonstd
│   │   │   │   │       └── optional.hpp
│   │   │   │   └── ouster
│   │   │   │       ├── client.h
│   │   │   │       ├── defaults.h
│   │   │   │       ├── image_processing.h
│   │   │   │       ├── impl
│   │   │   │       │   ├── cartesian.h
│   │   │   │       │   ├── client_poller.h
│   │   │   │       │   ├── lidar_scan_impl.h
│   │   │   │       │   ├── netcompat.h
│   │   │   │       │   ├── packet_writer.h
│   │   │   │       │   ├── profile_extension.h
│   │   │   │       │   └── ring_buffer.h
│   │   │   │       ├── lidar_scan.h
│   │   │   │       ├── sensor_http.h
│   │   │   │       ├── types.h
│   │   │   │       ├── udp_packet_source.h
│   │   │   │       ├── util.h
│   │   │   │       └── version.h
│   │   │   └── src
│   │   │       ├── client.cpp
│   │   │       ├── curl_client.h
│   │   │       ├── http_client.h
│   │   │       ├── image_processing.cpp
│   │   │       ├── lidar_scan.cpp
│   │   │       ├── logging.cpp
│   │   │       ├── logging.h
│   │   │       ├── netcompat.cpp
│   │   │       ├── parsing.cpp
│   │   │       ├── profile_extension.cpp
│   │   │       ├── sensor_http.cpp
│   │   │       ├── sensor_http_imp.cpp
│   │   │       ├── sensor_http_imp.h
│   │   │       ├── sensor_info.cpp
│   │   │       ├── sensor_tcp_imp.cpp
│   │   │       ├── sensor_tcp_imp.h
│   │   │       ├── types.cpp
│   │   │       ├── udp_packet_source.cpp
│   │   │       └── util.cpp
│   │   ├── ouster_osf
│   │   │   ├── CHANGELOG.rst
│   │   │   ├── CMakeLists.txt
│   │   │   ├── README.rst
│   │   │   ├── cmake
│   │   │   │   └── osf_fb_utils.cmake
│   │   │   ├── fb
│   │   │   │   ├── chunk.fbs
│   │   │   │   ├── header.fbs
│   │   │   │   ├── metadata.fbs
│   │   │   │   ├── os_sensor
│   │   │   │   │   ├── extrinsics.fbs
│   │   │   │   │   ├── lidar_scan_stream.fbs
│   │   │   │   │   └── lidar_sensor.fbs
│   │   │   │   └── streaming
│   │   │   │       └── streaming_info.fbs
│   │   │   ├── include
│   │   │   │   └── ouster
│   │   │   │       └── osf
│   │   │   │           ├── basics.h
│   │   │   │           ├── crc32.h
│   │   │   │           ├── file.h
│   │   │   │           ├── layout_streaming.h
│   │   │   │           ├── meta_extrinsics.h
│   │   │   │           ├── meta_lidar_sensor.h
│   │   │   │           ├── meta_streaming_info.h
│   │   │   │           ├── metadata.h
│   │   │   │           ├── operations.h
│   │   │   │           ├── pcap_source.h
│   │   │   │           ├── reader.h
│   │   │   │           ├── stream_lidar_scan.h
│   │   │   │           └── writer.h
│   │   │   ├── src
│   │   │   │   ├── basics.cpp
│   │   │   │   ├── compat_ops.cpp
│   │   │   │   ├── compat_ops.h
│   │   │   │   ├── crc32.cpp
│   │   │   │   ├── fb_utils.cpp
│   │   │   │   ├── fb_utils.h
│   │   │   │   ├── file.cpp
│   │   │   │   ├── json_utils.cpp
│   │   │   │   ├── json_utils.h
│   │   │   │   ├── layout_streaming.cpp
│   │   │   │   ├── meta_extrinsics.cpp
│   │   │   │   ├── meta_lidar_sensor.cpp
│   │   │   │   ├── meta_streaming_info.cpp
│   │   │   │   ├── metadata.cpp
│   │   │   │   ├── operations.cpp
│   │   │   │   ├── pcap_source.cpp
│   │   │   │   ├── png_tools.cpp
│   │   │   │   ├── png_tools.h
│   │   │   │   ├── reader.cpp
│   │   │   │   ├── stream_lidar_scan.cpp
│   │   │   │   └── writer.cpp
│   │   │   └── tests
│   │   │       ├── CMakeLists.txt
│   │   │       ├── basics_test.cpp
│   │   │       ├── common.h
│   │   │       ├── crc_test.cpp
│   │   │       ├── file_ops_test.cpp
│   │   │       ├── file_test.cpp
│   │   │       ├── meta_streaming_info_test.cpp
│   │   │       ├── metadata_tests.cpp
│   │   │       ├── operations_test.cpp
│   │   │       ├── osf_test.h
│   │   │       ├── pcap_source_test.cpp
│   │   │       ├── png_tools_test.cpp
│   │   │       ├── reader_test.cpp
│   │   │       ├── writer_custom_test.cpp
│   │   │       ├── writer_test.cpp
│   │   │       └── writerv2_test.cpp
│   │   ├── ouster_pcap
│   │   │   ├── CMakeLists.txt
│   │   │   ├── include
│   │   │   │   └── ouster
│   │   │   │       ├── indexed_pcap_reader.h
│   │   │   │       ├── os_pcap.h
│   │   │   │       └── pcap.h
│   │   │   └── src
│   │   │       ├── indexed_pcap_reader.cpp
│   │   │       ├── os_pcap.cpp
│   │   │       └── pcap.cpp
│   │   ├── ouster_viz
│   │   │   ├── CMakeLists.txt
│   │   │   ├── include
│   │   │   │   └── ouster
│   │   │   │       └── point_viz.h
│   │   │   └── src
│   │   │       ├── camera.cpp
│   │   │       ├── camera.h
│   │   │       ├── cloud.cpp
│   │   │       ├── cloud.h
│   │   │       ├── colormaps.h
│   │   │       ├── common.h
│   │   │       ├── glfw.cpp
│   │   │       ├── glfw.h
│   │   │       ├── gltext.cpp
│   │   │       ├── gltext.h
│   │   │       ├── image.cpp
│   │   │       ├── image.h
│   │   │       ├── misc.cpp
│   │   │       ├── misc.h
│   │   │       └── point_viz.cpp
│   │   ├── python
│   │   │   ├── CMakeLists.txt
│   │   │   ├── Dockerfile
│   │   │   ├── MANIFEST.in
│   │   │   ├── README.rst
│   │   │   ├── mypy.ini
│   │   │   ├── pyproject.toml
│   │   │   ├── setup.cfg
│   │   │   ├── setup.py
│   │   │   ├── src
│   │   │   │   ├── cpp
│   │   │   │   │   ├── _client.cpp
│   │   │   │   │   ├── _osf.cpp
│   │   │   │   │   ├── _pcap.cpp
│   │   │   │   │   └── _viz.cpp
│   │   │   │   └── ouster
│   │   │   │       ├── cli
│   │   │   │       │   ├── __init__.py
│   │   │   │       │   ├── core
│   │   │   │       │   │   ├── __init__.py
│   │   │   │       │   │   ├── borg.py
│   │   │   │       │   │   ├── cli_args.py
│   │   │   │       │   │   └── util.py
│   │   │   │       │   ├── plugins
│   │   │   │       │   │   ├── discover.py
│   │   │   │       │   │   ├── source.py
│   │   │   │       │   │   ├── source_osf.py
│   │   │   │       │   │   ├── source_pcap.py
│   │   │   │       │   │   ├── source_save.py
│   │   │   │       │   │   ├── source_sensor.py
│   │   │   │       │   │   ├── source_util.py
│   │   │   │       │   │   └── testing.py
│   │   │   │       │   └── py.typed
│   │   │   │       ├── client
│   │   │   │       │   └── __init__.py
│   │   │   │       ├── osf
│   │   │   │       │   └── __init__.py
│   │   │   │       ├── pcap
│   │   │   │       │   └── __init__.py
│   │   │   │       ├── sdk
│   │   │   │       │   ├── __init__.py
│   │   │   │       │   ├── bag
│   │   │   │       │   │   ├── __init__.py
│   │   │   │       │   │   ├── bag.py
│   │   │   │       │   │   └── py.typed
│   │   │   │       │   ├── client
│   │   │   │       │   │   ├── __init__.py
│   │   │   │       │   │   ├── _client.pyi
│   │   │   │       │   │   ├── _digest.py
│   │   │   │       │   │   ├── _utils
│   │   │   │       │   │   │   └── __init__.py
│   │   │   │       │   │   ├── core.py
│   │   │   │       │   │   ├── data.py
│   │   │   │       │   │   ├── multi.py
│   │   │   │       │   │   ├── multi_scan_source.py
│   │   │   │       │   │   ├── py.typed
│   │   │   │       │   │   ├── scan_source.py
│   │   │   │       │   │   └── scan_source_adapter.py
│   │   │   │       │   ├── convert_to_legacy.py
│   │   │   │       │   ├── examples
│   │   │   │       │   │   ├── __init__.py
│   │   │   │       │   │   ├── client.py
│   │   │   │       │   │   ├── colormaps.py
│   │   │   │       │   │   ├── open3d.py
│   │   │   │       │   │   ├── osf.py
│   │   │   │       │   │   ├── pcap.py
│   │   │   │       │   │   ├── reference.py
│   │   │   │       │   │   └── viz.py
│   │   │   │       │   ├── io_type.py
│   │   │   │       │   ├── open_source.py
│   │   │   │       │   ├── osf
│   │   │   │       │   │   ├── __init__.py
│   │   │   │       │   │   ├── _osf.pyi
│   │   │   │       │   │   ├── data.py
│   │   │   │       │   │   ├── multi.py
│   │   │   │       │   │   ├── osf_scan_source.py
│   │   │   │       │   │   └── py.typed
│   │   │   │       │   ├── pcap
│   │   │   │       │   │   ├── __init__.py
│   │   │   │       │   │   ├── _pcap.pyi
│   │   │   │       │   │   ├── packet_iter.py
│   │   │   │       │   │   ├── pcap.py
│   │   │   │       │   │   ├── pcap_multi_packet_reader.py
│   │   │   │       │   │   ├── pcap_scan_source.py
│   │   │   │       │   │   └── py.typed
│   │   │   │       │   ├── py.typed
│   │   │   │       │   ├── sensor
│   │   │   │       │   │   ├── __init__.py
│   │   │   │       │   │   ├── py.typed
│   │   │   │       │   │   ├── sensor_multi_packet_reader.py
│   │   │   │       │   │   ├── sensor_scan_source.py
│   │   │   │       │   │   └── util.py
│   │   │   │       │   ├── sensor_util.py
│   │   │   │       │   ├── simple_viz.py
│   │   │   │       │   ├── util
│   │   │   │       │   │   ├── __init__.py
│   │   │   │       │   │   ├── extrinsics.py
│   │   │   │       │   │   ├── forward_slicer.py
│   │   │   │       │   │   ├── metadata.py
│   │   │   │       │   │   ├── parsing.py
│   │   │   │       │   │   ├── pose_util.py
│   │   │   │       │   │   ├── progress_bar.py
│   │   │   │       │   │   └── py.typed
│   │   │   │       │   └── viz
│   │   │   │       │       ├── __init__.py
│   │   │   │       │       ├── _viz.pyi
│   │   │   │       │       ├── core.py
│   │   │   │       │       ├── multi_viz.py
│   │   │   │       │       ├── py.typed
│   │   │   │       │       ├── scans_accum.py
│   │   │   │       │       ├── util.py
│   │   │   │       │       └── view_mode.py
│   │   │   │       └── viz
│   │   │   │           └── __init__.py
│   │   │   ├── tests
│   │   │   │   ├── __init__.py
│   │   │   │   ├── conftest.py
│   │   │   │   ├── osf
│   │   │   │   │   ├── test_osf_basics.py
│   │   │   │   │   └── test_osf_extrinsics.py
│   │   │   │   ├── ouster
│   │   │   │   │   └── cli
│   │   │   │   │       └── plugins
│   │   │   │   │           └── bad_plugin.py
│   │   │   │   ├── test_batching.py
│   │   │   │   ├── test_cli.py
│   │   │   │   ├── test_cli_util.py
│   │   │   │   ├── test_config.py
│   │   │   │   ├── test_core.py
│   │   │   │   ├── test_data.py
│   │   │   │   ├── test_destagger.py
│   │   │   │   ├── test_discover.py
│   │   │   │   ├── test_extended_profiles.py
│   │   │   │   ├── test_forward_slicer.py
│   │   │   │   ├── test_http_client.py
│   │   │   │   ├── test_metadata.py
│   │   │   │   ├── test_open_source.py
│   │   │   │   ├── test_packet_iter.py
│   │   │   │   ├── test_parsing.py
│   │   │   │   ├── test_pcap.py
│   │   │   │   ├── test_plugins.py
│   │   │   │   ├── test_pose_util.py
│   │   │   │   ├── test_resolve_extrinsics.py
│   │   │   │   ├── test_sdk_utils.py
│   │   │   │   ├── test_single_scan_source.py
│   │   │   │   ├── test_viz.py
│   │   │   │   ├── test_viz_utils.py
│   │   │   │   └── test_xyzlut.py
│   │   │   └── tox.ini
│   │   └── tests
│   │       ├── CMakeLists.txt
│   │       ├── bcompat_meta_json_test.cpp
│   │       ├── cartesian_test.cpp
│   │       ├── fusa_profile_test.cpp
│   │       ├── lidar_scan_test.cpp
│   │       ├── metadata
│   │       │   ├── 1_12_os1-991913000010-64.json
│   │       │   ├── 1_12_os1-991937000062-16A0_legacy.json
│   │       │   ├── 1_12_os1-991937000062-64_legacy.json
│   │       │   ├── 1_13_os1-991913000010-64.json
│   │       │   ├── 1_13_os1-991937000062-16A0_legacy.json
│   │       │   ├── 1_13_os1-991937000062-32A02_legacy.json
│   │       │   ├── 1_14_6cccd_os-882002000138-128_legacy.json
│   │       │   ├── 1_14_6cccd_os-882002000138-32U0_legacy.json
│   │       │   ├── 1_14_beta_os1-991937000062-16A0_legacy.json
│   │       │   ├── 1_14_beta_os1-991937000062-64_legacy.json
│   │       │   ├── 2_0_0_os1-991913000010-64.json
│   │       │   ├── 2_0_0_os1-992008000494-128_col_win_legacy.json
│   │       │   ├── 2_0_rc2_os-992011000121-32U0_legacy.json
│   │       │   ├── 2_1_2_os1-991913000010-64.json
│   │       │   ├── 2_1_2_os1-991913000010-64_legacy.json
│   │       │   ├── 2_2_os-992119000444-128.json
│   │       │   ├── 2_2_os-992119000444-128_legacy.json
│   │       │   ├── 2_3_1_os-992146000760-128.json
│   │       │   ├── 2_3_1_os-992146000760-128_legacy.json
│   │       │   ├── 2_4_0_os-992146000760-128.json
│   │       │   ├── 2_4_0_os-992146000760-128_legacy.json
│   │       │   ├── 2_5_0_os-992146000760-128.json
│   │       │   ├── 2_5_0_os-992146000760-128_legacy.json
│   │       │   ├── 3_0_1_os-122246000293-128.json
│   │       │   ├── 3_0_1_os-122246000293-128_legacy.json
│   │       │   ├── malformed
│   │       │   │   ├── complete_but_all_zeros_legacy.json
│   │       │   │   ├── garbled_legacy_and_nonlegacy.json
│   │       │   │   ├── incomplete_data_format_legacy.json
│   │       │   │   ├── incomplete_data_format_nonlegacy.json
│   │       │   │   ├── incomplete_no_calref_nonlegacy.json
│   │       │   │   ├── incomplete_no_sensor_info_nonlegacy.json
│   │       │   │   ├── incorrect_nbeam_angles_legacy_113.json
│   │       │   │   └── legacy_with_calibration_status.json
│   │       │   └── ouster-studio-reduced-config-v1.json
│   │       ├── metadata_errors_test.cpp
│   │       ├── metadata_test.cpp
│   │       ├── osfs
│   │       │   ├── OS-1-128_v2.3.0_1024x10_lb_n3.osf
│   │       │   └── empty_osf.osf
│   │       ├── packet_writer_test.cpp
│   │       ├── parsing_benchmark_test.cpp
│   │       ├── pcap_test.cpp
│   │       ├── pcap_with_extrinsics
│   │       │   ├── OS-0-128-U1_v2.3.0_10.pcap
│   │       │   ├── OS-0-128-U1_v2.3.0_1024x10.json
│   │       │   └── extrinsic_parameters.json
│   │       ├── pcap_without_extrinsics
│   │       │   ├── OS-0-128-U1_v2.3.0_10.pcap
│   │       │   └── OS-0-128-U1_v2.3.0_1024x10.json
│   │       ├── pcaps
│   │       │   ├── OS-0-128-U1_v2.3.0_1024x10.json
│   │       │   ├── OS-0-128-U1_v2.3.0_1024x10.pcap
│   │       │   ├── OS-0-128-U1_v2.3.0_1024x10_digest.json
│   │       │   ├── OS-0-32-U1_v2.2.0_1024x10-single-packet.pcap
│   │       │   ├── OS-0-32-U1_v2.2.0_1024x10.json
│   │       │   ├── OS-0-32-U1_v2.2.0_1024x10.pcap
│   │       │   ├── OS-0-32-U1_v2.2.0_1024x10_digest.json
│   │       │   ├── OS-1-128_767798045_1024x10_20230712_120049.json
│   │       │   ├── OS-1-128_767798045_1024x10_20230712_120049.pcap
│   │       │   ├── OS-1-128_v2.3.0_1024x10.json
│   │       │   ├── OS-1-128_v2.3.0_1024x10_lb_n3.pcap
│   │       │   ├── OS-1-128_v2.3.0_1024x10_lb_n3_poses_kitti.txt
│   │       │   ├── OS-1-128_v3.0.1_1024x10_20230216_142857_poses_kitti.txt
│   │       │   ├── OS-1-32-G_v2.1.1_1024x10.json
│   │       │   ├── OS-1-32-G_v2.1.1_1024x10.pcap
│   │       │   ├── OS-1-32-G_v2.1.1_1024x10_digest.json
│   │       │   ├── OS-1-64_1024x10_fw20.pcap
│   │       │   ├── OS-1-64_sensor_config_reduced.json
│   │       │   ├── OS-2-128-U1_v2.3.0_1024x10.json
│   │       │   ├── OS-2-128-U1_v2.3.0_1024x10.pcap
│   │       │   ├── OS-2-128-U1_v2.3.0_1024x10_digest.json
│   │       │   ├── OS-2-32-U0_v2.0.0_1024x10.json
│   │       │   ├── OS-2-32-U0_v2.0.0_1024x10.pcap
│   │       │   ├── OS-2-32-U0_v2.0.0_1024x10_digest.json
│   │       │   ├── VLI-16-one-packet.pcap
│   │       │   ├── empty_pcap.json
│   │       │   └── empty_pcap.pcap
│   │       ├── profile_extension_test.cpp
│   │       ├── scan_batcher_test.cpp
│   │       ├── udp_queue_test.cpp
│   │       └── util.h
│   ├── ouster_ros_nodelets.xml
│   ├── package.xml
│   ├── src
│   │   ├── image_processor.h
│   │   ├── impl
│   │   │   └── cartesian.h
│   │   ├── imu_packet_handler.h
│   │   ├── laser_scan_processor.h
│   │   ├── lidar_packet_handler.h
│   │   ├── lock_free_ring_buffer.h
│   │   ├── os_cloud_nodelet.cpp
│   │   ├── os_driver_nodelet.cpp
│   │   ├── os_image_nodelet.cpp
│   │   ├── os_pcap_nodelet.cpp
│   │   ├── os_replay_nodelet.cpp
│   │   ├── os_ros.cpp
│   │   ├── os_sensor_nodelet.cpp
│   │   ├── os_sensor_nodelet.h
│   │   ├── os_sensor_nodelet_base.cpp
│   │   ├── os_transforms_broadcaster.h
│   │   ├── point_cloud_compose.h
│   │   ├── point_cloud_processor.h
│   │   ├── point_cloud_processor_factory.h
│   │   ├── point_meta_helpers.h
│   │   ├── point_transform.h
│   │   └── telemetry_handler.h
│   ├── srv
│   │   ├── GetConfig.srv
│   │   ├── GetMetadata.srv
│   │   └── SetConfig.srv
│   ├── tests
│   │   ├── lock_free_ring_buffer_test.cpp
│   │   ├── point_accessor_test.cpp
│   │   ├── point_cloud_compose_test.cpp
│   │   ├── point_transform_test.cpp
│   │   └── test_main.cpp
│   └── util
│       └── network-configure.bash
├── project_summary.md
└── xsens_ros_mti_driver
    ├── .gitattributes
    ├── CMakeLists.txt
    ├── LICENSE.txt
    ├── README.txt
    ├── launch
    │   ├── display.launch
    │   └── xsens_mti_node.launch
    ├── lib
    │   └── xspublic
    │       ├── Makefile
    │       ├── xscommon
    │       │   ├── Makefile
    │       │   ├── abstractadditionallogger.h
    │       │   ├── additionalloggerbase.cpp
    │       │   ├── additionalloggerbase.cpp.o
    │       │   ├── additionalloggerbase.dpp
    │       │   ├── additionalloggerbase.h
    │       │   ├── calltracer.h
    │       │   ├── common_qdebug.h
    │       │   ├── consolelogger.cpp
    │       │   ├── consolelogger.cpp.o
    │       │   ├── consolelogger.dpp
    │       │   ├── consolelogger.h
    │       │   ├── enumexpandersbase.cpp
    │       │   ├── enumexpandersbase.cpp.o
    │       │   ├── enumexpandersbase.dpp
    │       │   ├── enumexpandersbase.h
    │       │   ├── extendedenum.h
    │       │   ├── fwupdate.c
    │       │   ├── fwupdate.c.o
    │       │   ├── fwupdate.d
    │       │   ├── fwupdate.h
    │       │   ├── journalexception.cpp
    │       │   ├── journalexception.cpp.o
    │       │   ├── journalexception.dpp
    │       │   ├── journalexception.h
    │       │   ├── journalfile.cpp
    │       │   ├── journalfile.cpp.o
    │       │   ├── journalfile.dpp
    │       │   ├── journalfile.h
    │       │   ├── journaller.cpp
    │       │   ├── journaller.cpp.o
    │       │   ├── journaller.dpp
    │       │   ├── journaller.h
    │       │   ├── journalloglevel.h
    │       │   ├── journalstackwalker.h
    │       │   ├── journalthreader.cpp
    │       │   ├── journalthreader.cpp.o
    │       │   ├── journalthreader.dpp
    │       │   ├── journalthreader.h
    │       │   ├── libxscommon.a
    │       │   ├── stackdumper.cpp
    │       │   ├── stackdumper.cpp.o
    │       │   ├── stackdumper.dpp
    │       │   ├── stackdumper.h
    │       │   ├── stackwalker.cpp
    │       │   ├── stackwalker.cpp.o
    │       │   ├── stackwalker.dpp
    │       │   ├── stackwalker.h
    │       │   ├── stackwalker_linux.cpp
    │       │   ├── stackwalker_linux.cpp.o
    │       │   ├── stackwalker_linux.dpp
    │       │   ├── stackwalker_linux.h
    │       │   ├── threading.cpp
    │       │   ├── threading.cpp.o
    │       │   ├── threading.dpp
    │       │   ├── threading.h
    │       │   ├── xbus.c
    │       │   ├── xbus.c.o
    │       │   ├── xbus.d
    │       │   ├── xbus.h
    │       │   ├── xbusparser.c
    │       │   ├── xbusparser.c.o
    │       │   ├── xbusparser.d
    │       │   ├── xbusparser.h
    │       │   ├── xprintf.cpp
    │       │   ├── xprintf.cpp.o
    │       │   ├── xprintf.dpp
    │       │   ├── xprintf.h
    │       │   ├── xscommon_config.h
    │       │   ├── xsens_debugtools.h
    │       │   ├── xsens_generic_matrix.h
    │       │   ├── xsens_janitors.h
    │       │   ├── xsens_math_throw.cpp
    │       │   ├── xsens_math_throw.cpp.o
    │       │   ├── xsens_math_throw.dpp
    │       │   ├── xsens_math_throw.h
    │       │   ├── xsens_mutex.h
    │       │   ├── xsens_threadpool.cpp
    │       │   ├── xsens_threadpool.cpp.o
    │       │   ├── xsens_threadpool.dpp
    │       │   └── xsens_threadpool.h
    │       ├── xscontroller
    │       │   ├── Makefile
    │       │   ├── broadcastdevice.cpp
    │       │   ├── broadcastdevice.cpp.o
    │       │   ├── broadcastdevice.dpp
    │       │   ├── broadcastdevice.h
    │       │   ├── callbackmanagerxda.cpp
    │       │   ├── callbackmanagerxda.cpp.o
    │       │   ├── callbackmanagerxda.dpp
    │       │   ├── callbackmanagerxda.h
    │       │   ├── clocksynccommand.h
    │       │   ├── communicator.cpp
    │       │   ├── communicator.cpp.o
    │       │   ├── communicator.dpp
    │       │   ├── communicator.h
    │       │   ├── communicatorfactory.cpp
    │       │   ├── communicatorfactory.cpp.o
    │       │   ├── communicatorfactory.dpp
    │       │   ├── communicatorfactory.h
    │       │   ├── compat.h
    │       │   ├── datalogger.cpp
    │       │   ├── datalogger.cpp.o
    │       │   ├── datalogger.dpp
    │       │   ├── datalogger.h
    │       │   ├── datapacketcache.cpp
    │       │   ├── datapacketcache.cpp.o
    │       │   ├── datapacketcache.dpp
    │       │   ├── datapacketcache.h
    │       │   ├── dataparser.cpp
    │       │   ├── dataparser.cpp.o
    │       │   ├── dataparser.dpp
    │       │   ├── dataparser.h
    │       │   ├── datapoller.cpp
    │       │   ├── datapoller.cpp.o
    │       │   ├── datapoller.dpp
    │       │   ├── datapoller.h
    │       │   ├── devicecommunicator.cpp
    │       │   ├── devicecommunicator.cpp.o
    │       │   ├── devicecommunicator.dpp
    │       │   ├── devicecommunicator.h
    │       │   ├── devicefactory.cpp
    │       │   ├── devicefactory.cpp.o
    │       │   ├── devicefactory.dpp
    │       │   ├── devicefactory.h
    │       │   ├── deviceredetector.cpp
    │       │   ├── deviceredetector.cpp.o
    │       │   ├── deviceredetector.dpp
    │       │   ├── deviceredetector.h
    │       │   ├── devicetypes.h
    │       │   ├── dotdevice.cpp
    │       │   ├── dotdevice.cpp.o
    │       │   ├── dotdevice.dpp
    │       │   ├── dotdevice.h
    │       │   ├── dummy.cpp
    │       │   ├── dummy.cpp.o
    │       │   ├── dummy.dpp
    │       │   ├── enumerateusbdevices.cpp
    │       │   ├── enumerateusbdevices.cpp.o
    │       │   ├── enumerateusbdevices.dpp
    │       │   ├── enumerateusbdevices.h
    │       │   ├── enumexpanders.cpp
    │       │   ├── enumexpanders.cpp.o
    │       │   ├── enumexpanders.dpp
    │       │   ├── enumexpanders.h
    │       │   ├── fileloader.h
    │       │   ├── gpsstatus.h
    │       │   ├── idfetchhelpers.h
    │       │   ├── iointerface.cpp
    │       │   ├── iointerface.cpp.o
    │       │   ├── iointerface.dpp
    │       │   ├── iointerface.h
    │       │   ├── iointerfacefile.cpp
    │       │   ├── iointerfacefile.cpp.o
    │       │   ├── iointerfacefile.dpp
    │       │   ├── iointerfacefile.h
    │       │   ├── iprotocolhandler.h
    │       │   ├── iprotocolmanager.h
    │       │   ├── lastresultmanager.h
    │       │   ├── libxscontroller.a
    │       │   ├── messageextractor.cpp
    │       │   ├── messageextractor.cpp.o
    │       │   ├── messageextractor.dpp
    │       │   ├── messageextractor.h
    │       │   ├── messagelocation.h
    │       │   ├── messageserializer.cpp
    │       │   ├── messageserializer.cpp.o
    │       │   ├── messageserializer.dpp
    │       │   ├── messageserializer.h
    │       │   ├── mtbdatalogger.cpp
    │       │   ├── mtbdatalogger.cpp.o
    │       │   ├── mtbdatalogger.dpp
    │       │   ├── mtbdatalogger.h
    │       │   ├── mtbfilecommunicator.cpp
    │       │   ├── mtbfilecommunicator.cpp.o
    │       │   ├── mtbfilecommunicator.dpp
    │       │   ├── mtbfilecommunicator.h
    │       │   ├── mtdevice.cpp
    │       │   ├── mtdevice.cpp.o
    │       │   ├── mtdevice.dpp
    │       │   ├── mtdevice.h
    │       │   ├── mti3x0device.cpp
    │       │   ├── mti3x0device.cpp.o
    │       │   ├── mti3x0device.dpp
    │       │   ├── mti3x0device.h
    │       │   ├── mti6x0device.cpp
    │       │   ├── mti6x0device.cpp.o
    │       │   ├── mti6x0device.dpp
    │       │   ├── mti6x0device.h
    │       │   ├── mti7_mti8device.cpp
    │       │   ├── mti7_mti8device.cpp.o
    │       │   ├── mti7_mti8device.dpp
    │       │   ├── mti7_mti8device.h
    │       │   ├── mti8x0device.cpp
    │       │   ├── mti8x0device.cpp.o
    │       │   ├── mti8x0device.dpp
    │       │   ├── mti8x0device.h
    │       │   ├── mti9x0device.cpp
    │       │   ├── mti9x0device.cpp.o
    │       │   ├── mti9x0device.dpp
    │       │   ├── mti9x0device.h
    │       │   ├── mtibasedevice.cpp
    │       │   ├── mtibasedevice.cpp.o
    │       │   ├── mtibasedevice.dpp
    │       │   ├── mtibasedevice.h
    │       │   ├── mtigdevice.cpp
    │       │   ├── mtigdevice.cpp.o
    │       │   ├── mtigdevice.dpp
    │       │   ├── mtigdevice.h
    │       │   ├── mtix00device.cpp
    │       │   ├── mtix00device.cpp.o
    │       │   ├── mtix00device.dpp
    │       │   ├── mtix00device.h
    │       │   ├── mtix0device.cpp
    │       │   ├── mtix0device.cpp.o
    │       │   ├── mtix0device.dpp
    │       │   ├── mtix0device.h
    │       │   ├── mtixdevice.cpp
    │       │   ├── mtixdevice.cpp.o
    │       │   ├── mtixdevice.dpp
    │       │   ├── mtixdevice.h
    │       │   ├── mtsyncsettings.h
    │       │   ├── mtthread.cpp
    │       │   ├── mtthread.cpp.o
    │       │   ├── mtthread.dpp
    │       │   ├── mtthread.h
    │       │   ├── nmea_protocolhandler.cpp
    │       │   ├── nmea_protocolhandler.cpp.o
    │       │   ├── nmea_protocolhandler.dpp
    │       │   ├── nmea_protocolhandler.h
    │       │   ├── openportstage.h
    │       │   ├── packeterrorrateestimator.cpp
    │       │   ├── packeterrorrateestimator.cpp.o
    │       │   ├── packeterrorrateestimator.dpp
    │       │   ├── packeterrorrateestimator.h
    │       │   ├── packetstamper.cpp
    │       │   ├── packetstamper.cpp.o
    │       │   ├── packetstamper.dpp
    │       │   ├── packetstamper.h
    │       │   ├── protocolhandler.cpp
    │       │   ├── protocolhandler.cpp.o
    │       │   ├── protocolhandler.dpp
    │       │   ├── protocolhandler.h
    │       │   ├── protocolmanager.cpp
    │       │   ├── protocolmanager.cpp.o
    │       │   ├── protocolmanager.dpp
    │       │   ├── protocolmanager.h
    │       │   ├── proxycommunicator.cpp
    │       │   ├── proxycommunicator.cpp.o
    │       │   ├── proxycommunicator.dpp
    │       │   ├── proxycommunicator.h
    │       │   ├── rangequeue.h
    │       │   ├── replymonitor.cpp
    │       │   ├── replymonitor.cpp.o
    │       │   ├── replymonitor.dpp
    │       │   ├── replymonitor.h
    │       │   ├── replyobject.cpp
    │       │   ├── replyobject.cpp.o
    │       │   ├── replyobject.dpp
    │       │   ├── replyobject.h
    │       │   ├── restorecommunication.cpp
    │       │   ├── restorecommunication.cpp.o
    │       │   ├── restorecommunication.dpp
    │       │   ├── restorecommunication.h
    │       │   ├── rx_tx_log.h
    │       │   ├── scanner.cpp
    │       │   ├── scanner.cpp.o
    │       │   ├── scanner.dpp
    │       │   ├── scanner.h
    │       │   ├── scenariomatchpred.h
    │       │   ├── serialcommunicator.cpp
    │       │   ├── serialcommunicator.cpp.o
    │       │   ├── serialcommunicator.dpp
    │       │   ├── serialcommunicator.h
    │       │   ├── serialinterface.cpp
    │       │   ├── serialinterface.cpp.o
    │       │   ├── serialinterface.dpp
    │       │   ├── serialinterface.h
    │       │   ├── serialportcommunicator.cpp
    │       │   ├── serialportcommunicator.cpp.o
    │       │   ├── serialportcommunicator.dpp
    │       │   ├── serialportcommunicator.h
    │       │   ├── simpleprotocolmanager.h
    │       │   ├── streaminterface.cpp
    │       │   ├── streaminterface.cpp.o
    │       │   ├── streaminterface.dpp
    │       │   ├── streaminterface.h
    │       │   ├── supportedsyncsettings.cpp
    │       │   ├── supportedsyncsettings.cpp.o
    │       │   ├── supportedsyncsettings.dpp
    │       │   ├── supportedsyncsettings.h
    │       │   ├── synclinegmt.c
    │       │   ├── synclinegmt.c.o
    │       │   ├── synclinegmt.d
    │       │   ├── synclinegmt.h
    │       │   ├── synclinemk4.c
    │       │   ├── synclinemk4.c.o
    │       │   ├── synclinemk4.d
    │       │   ├── synclinemk4.h
    │       │   ├── udev.cpp
    │       │   ├── udev.cpp.o
    │       │   ├── udev.dpp
    │       │   ├── udev.h
    │       │   ├── usbcommunicator.cpp
    │       │   ├── usbcommunicator.cpp.o
    │       │   ├── usbcommunicator.dpp
    │       │   ├── usbcommunicator.h
    │       │   ├── usbinterface.cpp
    │       │   ├── usbinterface.cpp.o
    │       │   ├── usbinterface.dpp
    │       │   ├── usbinterface.h
    │       │   ├── xdacommunicatorfactory.cpp
    │       │   ├── xdacommunicatorfactory.cpp.o
    │       │   ├── xdacommunicatorfactory.dpp
    │       │   ├── xdacommunicatorfactory.h
    │       │   ├── xsaccesscontrolmode.h
    │       │   ├── xsalignmentframe.h
    │       │   ├── xscalibrateddatamode.h
    │       │   ├── xscallback.h
    │       │   ├── xscallbackplainc.h
    │       │   ├── xsconnectivitystate.c
    │       │   ├── xsconnectivitystate.c.o
    │       │   ├── xsconnectivitystate.d
    │       │   ├── xsconnectivitystate.h
    │       │   ├── xscontrol_def.cpp
    │       │   ├── xscontrol_def.cpp.o
    │       │   ├── xscontrol_def.dpp
    │       │   ├── xscontrol_def.h
    │       │   ├── xscontrol_public.h
    │       │   ├── xscontrollerconfig.h
    │       │   ├── xscoordinatesystem.h
    │       │   ├── xsdef.cpp
    │       │   ├── xsdef.cpp.o
    │       │   ├── xsdef.dpp
    │       │   ├── xsdef.h
    │       │   ├── xsdevice_def.cpp
    │       │   ├── xsdevice_def.cpp.o
    │       │   ├── xsdevice_def.dpp
    │       │   ├── xsdevice_def.h
    │       │   ├── xsdevice_public.h
    │       │   ├── xsdeviceconfiguration.c
    │       │   ├── xsdeviceconfiguration.c.o
    │       │   ├── xsdeviceconfiguration.d
    │       │   ├── xsdeviceconfiguration.h
    │       │   ├── xsdeviceparameter.h
    │       │   ├── xsdeviceparameteridentifier.h
    │       │   ├── xsdeviceptr.h
    │       │   ├── xsdeviceptrarray.c
    │       │   ├── xsdeviceptrarray.c.o
    │       │   ├── xsdeviceptrarray.d
    │       │   ├── xsdeviceptrarray.h
    │       │   ├── xsdevicestate.c
    │       │   ├── xsdevicestate.c.o
    │       │   ├── xsdevicestate.d
    │       │   ├── xsdevicestate.h
    │       │   ├── xsdevicestate_enum.h
    │       │   ├── xserrormode.h
    │       │   ├── xsfloatformat.h
    │       │   ├── xsgnssplatform.h
    │       │   ├── xsgnssreceivertype.h
    │       │   ├── xsgnssstatus.h
    │       │   ├── xsicccommand.h
    │       │   ├── xsiccrepmotionresult.h
    │       │   ├── xslibusb.cpp
    │       │   ├── xslibusb.cpp.o
    │       │   ├── xslibusb.dpp
    │       │   ├── xslibusb.h
    │       │   ├── xsoperationalmode.h
    │       │   ├── xsorientationmode.h
    │       │   ├── xsprocessingflag.h
    │       │   ├── xsprotocoltype.h
    │       │   ├── xsrejectreason.c
    │       │   ├── xsrejectreason.c.o
    │       │   ├── xsrejectreason.d
    │       │   ├── xsrejectreason.h
    │       │   ├── xsscanner.cpp
    │       │   ├── xsscanner.cpp.o
    │       │   ├── xsscanner.dpp
    │       │   ├── xsscanner.h
    │       │   ├── xsselftestresult.h
    │       │   ├── xsubloxgnssplatform.h
    │       │   ├── xsusbhubinfo.c
    │       │   ├── xsusbhubinfo.c.o
    │       │   ├── xsusbhubinfo.d
    │       │   ├── xsusbhubinfo.h
    │       │   ├── xswinusb.cpp
    │       │   ├── xswinusb.cpp.o
    │       │   ├── xswinusb.dpp
    │       │   └── xswinusb.h
    │       └── xstypes
    │           ├── Makefile
    │           ├── datapacket_p.cpp
    │           ├── datapacket_p.cpp.o
    │           ├── datapacket_p.dpp
    │           ├── datapacket_p.h
    │           ├── libxstypes.a
    │           ├── pstdint.h
    │           ├── resource.h
    │           ├── xsanalogindata.h
    │           ├── xsarray.c
    │           ├── xsarray.c.o
    │           ├── xsarray.d
    │           ├── xsarray.h
    │           ├── xsbaud.c
    │           ├── xsbaud.c.o
    │           ├── xsbaud.d
    │           ├── xsbaud.h
    │           ├── xsbaudcode.h
    │           ├── xsbaudrate.h
    │           ├── xsbusid.h
    │           ├── xsbytearray.c
    │           ├── xsbytearray.c.o
    │           ├── xsbytearray.d
    │           ├── xsbytearray.h
    │           ├── xscalibrateddata.c
    │           ├── xscalibrateddata.c.o
    │           ├── xscalibrateddata.d
    │           ├── xscalibrateddata.h
    │           ├── xscanbaudcode.h
    │           ├── xscanconfigidentifier.h
    │           ├── xscandataidentifier.h
    │           ├── xscanframeformat.h
    │           ├── xscanoutputconfiguration.c
    │           ├── xscanoutputconfiguration.c.o
    │           ├── xscanoutputconfiguration.d
    │           ├── xscanoutputconfiguration.h
    │           ├── xscanoutputconfigurationarray.c
    │           ├── xscanoutputconfigurationarray.c.o
    │           ├── xscanoutputconfigurationarray.d
    │           ├── xscanoutputconfigurationarray.h
    │           ├── xscontrolline.h
    │           ├── xscopy.h
    │           ├── xsdataidentifier.h
    │           ├── xsdataidentifiervalue.h
    │           ├── xsdatapacket.cpp
    │           ├── xsdatapacket.cpp.o
    │           ├── xsdatapacket.dpp
    │           ├── xsdatapacket.h
    │           ├── xsdatapacketptr.h
    │           ├── xsdatapacketptrarray.c
    │           ├── xsdatapacketptrarray.c.o
    │           ├── xsdatapacketptrarray.d
    │           ├── xsdatapacketptrarray.h
    │           ├── xsdebugcounters.cpp
    │           ├── xsdebugcounters.cpp.o
    │           ├── xsdebugcounters.dpp
    │           ├── xsdebugcounters.h
    │           ├── xsdevicecapabilities.c
    │           ├── xsdevicecapabilities.c.o
    │           ├── xsdevicecapabilities.d
    │           ├── xsdevicecapabilities.h
    │           ├── xsdeviceid.c
    │           ├── xsdeviceid.c.o
    │           ├── xsdeviceid.d
    │           ├── xsdeviceid.h
    │           ├── xsdeviceidarray.c
    │           ├── xsdeviceidarray.c.o
    │           ├── xsdeviceidarray.d
    │           ├── xsdeviceidarray.h
    │           ├── xsdeviceoptionflag.h
    │           ├── xsdid.h
    │           ├── xsens_compat.h
    │           ├── xseuler.c
    │           ├── xseuler.c.o
    │           ├── xseuler.d
    │           ├── xseuler.h
    │           ├── xsexception.h
    │           ├── xsfile.c
    │           ├── xsfile.c.o
    │           ├── xsfile.d
    │           ├── xsfile.h
    │           ├── xsfilepos.h
    │           ├── xsfilterprofile.c
    │           ├── xsfilterprofile.c.o
    │           ├── xsfilterprofile.d
    │           ├── xsfilterprofile.h
    │           ├── xsfilterprofilearray.c
    │           ├── xsfilterprofilearray.c.o
    │           ├── xsfilterprofilearray.d
    │           ├── xsfilterprofilearray.h
    │           ├── xsfilterprofilekind.h
    │           ├── xsfloatmath.h
    │           ├── xsfloatvector.h
    │           ├── xsglovedata.c
    │           ├── xsglovedata.c.o
    │           ├── xsglovedata.d
    │           ├── xsglovedata.h
    │           ├── xsglovesnapshot.h
    │           ├── xshandid.h
    │           ├── xsinforequest.h
    │           ├── xsint64array.c
    │           ├── xsint64array.c.o
    │           ├── xsint64array.d
    │           ├── xsint64array.h
    │           ├── xsintarray.c
    │           ├── xsintarray.c.o
    │           ├── xsintarray.d
    │           ├── xsintarray.h
    │           ├── xslibraryloader.c
    │           ├── xslibraryloader.c.o
    │           ├── xslibraryloader.d
    │           ├── xslibraryloader.h
    │           ├── xsmalloc.c
    │           ├── xsmalloc.c.o
    │           ├── xsmalloc.d
    │           ├── xsmalloc.h
    │           ├── xsmath.c
    │           ├── xsmath.c.o
    │           ├── xsmath.d
    │           ├── xsmath.h
    │           ├── xsmath2.h
    │           ├── xsmatrix.c
    │           ├── xsmatrix.c.o
    │           ├── xsmatrix.d
    │           ├── xsmatrix.h
    │           ├── xsmatrix3x3.c
    │           ├── xsmatrix3x3.c.o
    │           ├── xsmatrix3x3.d
    │           ├── xsmatrix3x3.h
    │           ├── xsmessage.c
    │           ├── xsmessage.c.o
    │           ├── xsmessage.d
    │           ├── xsmessage.h
    │           ├── xsmessagearray.c
    │           ├── xsmessagearray.c.o
    │           ├── xsmessagearray.d
    │           ├── xsmessagearray.h
    │           ├── xsmfmresultvalue.h
    │           ├── xsoption.h
    │           ├── xsoutputconfiguration.c
    │           ├── xsoutputconfiguration.c.o
    │           ├── xsoutputconfiguration.d
    │           ├── xsoutputconfiguration.h
    │           ├── xsoutputconfigurationarray.c
    │           ├── xsoutputconfigurationarray.c.o
    │           ├── xsoutputconfigurationarray.d
    │           ├── xsoutputconfigurationarray.h
    │           ├── xsplatform.h
    │           ├── xsportinfo.c
    │           ├── xsportinfo.c.o
    │           ├── xsportinfo.d
    │           ├── xsportinfo.h
    │           ├── xsportinfoarray.c
    │           ├── xsportinfoarray.c.o
    │           ├── xsportinfoarray.d
    │           ├── xsportinfoarray.h
    │           ├── xspressure.h
    │           ├── xsprotocol.h
    │           ├── xsquaternion.c
    │           ├── xsquaternion.c.o
    │           ├── xsquaternion.d
    │           ├── xsquaternion.h
    │           ├── xsquaternionarray.c
    │           ├── xsquaternionarray.c.o
    │           ├── xsquaternionarray.d
    │           ├── xsquaternionarray.h
    │           ├── xsrange.c
    │           ├── xsrange.c.o
    │           ├── xsrange.d
    │           ├── xsrange.h
    │           ├── xsrawgnsspvtdata.h
    │           ├── xsrawgnsssatinfo.h
    │           ├── xsresetmethod.h
    │           ├── xsresultvalue.c
    │           ├── xsresultvalue.c.o
    │           ├── xsresultvalue.d
    │           ├── xsresultvalue.h
    │           ├── xsrssi.c
    │           ├── xsrssi.c.o
    │           ├── xsrssi.d
    │           ├── xsrssi.h
    │           ├── xsscrdata.h
    │           ├── xsscrdatafloat.h
    │           ├── xssdidata.c
    │           ├── xssdidata.c.o
    │           ├── xssdidata.d
    │           ├── xssdidata.h
    │           ├── xssensorranges.cpp
    │           ├── xssensorranges.cpp.o
    │           ├── xssensorranges.dpp
    │           ├── xssensorranges.h
    │           ├── xssimpleversion.c
    │           ├── xssimpleversion.c.o
    │           ├── xssimpleversion.d
    │           ├── xssimpleversion.h
    │           ├── xssnapshot.c
    │           ├── xssnapshot.c.o
    │           ├── xssnapshot.d
    │           ├── xssnapshot.h
    │           ├── xssocket.c
    │           ├── xssocket.c.o
    │           ├── xssocket.d
    │           ├── xssocket.h
    │           ├── xsstatusflag.h
    │           ├── xsstring.c
    │           ├── xsstring.c.o
    │           ├── xsstring.d
    │           ├── xsstring.h
    │           ├── xsstringarray.c
    │           ├── xsstringarray.c.o
    │           ├── xsstringarray.d
    │           ├── xsstringarray.h
    │           ├── xsstringoutputtype.h
    │           ├── xsstringoutputtypearray.c
    │           ├── xsstringoutputtypearray.c.o
    │           ├── xsstringoutputtypearray.d
    │           ├── xsstringoutputtypearray.h
    │           ├── xsstringstreaming.h
    │           ├── xssyncfunction.h
    │           ├── xssyncline.h
    │           ├── xssyncpolarity.h
    │           ├── xssyncrole.h
    │           ├── xssyncsetting.c
    │           ├── xssyncsetting.c.o
    │           ├── xssyncsetting.d
    │           ├── xssyncsetting.h
    │           ├── xssyncsettingarray.c
    │           ├── xssyncsettingarray.c.o
    │           ├── xssyncsettingarray.d
    │           ├── xssyncsettingarray.h
    │           ├── xsthread.c
    │           ├── xsthread.c.o
    │           ├── xsthread.d
    │           ├── xsthread.h
    │           ├── xstime.c
    │           ├── xstime.c.o
    │           ├── xstime.d
    │           ├── xstime.h
    │           ├── xstimeinfo.c
    │           ├── xstimeinfo.c.o
    │           ├── xstimeinfo.d
    │           ├── xstimeinfo.h
    │           ├── xstimestamp.c
    │           ├── xstimestamp.c.o
    │           ├── xstimestamp.d
    │           ├── xstimestamp.h
    │           ├── xstriggerindicationdata.c
    │           ├── xstriggerindicationdata.c.o
    │           ├── xstriggerindicationdata.d
    │           ├── xstriggerindicationdata.h
    │           ├── xstypedefs.c
    │           ├── xstypedefs.c.o
    │           ├── xstypedefs.d
    │           ├── xstypedefs.h
    │           ├── xstypesconfig.h
    │           ├── xstypesdef.h
    │           ├── xstypesdynlib.c
    │           ├── xstypesdynlib.h
    │           ├── xstypesinfo.h
    │           ├── xsushortvector.h
    │           ├── xsutctime.c
    │           ├── xsutctime.c.o
    │           ├── xsutctime.d
    │           ├── xsutctime.h
    │           ├── xsvector.c
    │           ├── xsvector.c.o
    │           ├── xsvector.d
    │           ├── xsvector.h
    │           ├── xsvector3.c
    │           ├── xsvector3.c.o
    │           ├── xsvector3.d
    │           ├── xsvector3.h
    │           ├── xsversion.c
    │           ├── xsversion.c.o
    │           ├── xsversion.d
    │           ├── xsversion.h
    │           └── xsxbusmessageid.h
    ├── package.xml
    ├── param
    │   └── xsens_mti_node.yaml
    ├── rviz
    │   ├── display.rviz
    │   └── example.rviz
    ├── src
    │   ├── main.cpp
    │   ├── messagepublishers
    │   │   ├── accelerationpublisher.h
    │   │   ├── angularvelocitypublisher.h
    │   │   ├── freeaccelerationpublisher.h
    │   │   ├── gnsspublisher.h
    │   │   ├── imupublisher.h
    │   │   ├── magneticfieldpublisher.h
    │   │   ├── orientationincrementspublisher.h
    │   │   ├── orientationpublisher.h
    │   │   ├── packetcallback.h
    │   │   ├── positionllapublisher.h
    │   │   ├── pressurepublisher.h
    │   │   ├── temperaturepublisher.h
    │   │   ├── timereferencepublisher.h
    │   │   ├── transformpublisher.h
    │   │   ├── twistpublisher.h
    │   │   ├── velocityincrementpublisher.h
    │   │   └── velocitypublisher.h
    │   ├── xdacallback.cpp
    │   ├── xdacallback.h
    │   ├── xdainterface.cpp
    │   └── xdainterface.h
    └── urdf
        ├── MTi_1.stl
        ├── MTi_1.urdf
        ├── MTi_10.stl
        ├── MTi_10.urdf
        ├── MTi_6xx.stl
        └── MTi_6xx.urdf

180 directories, 1371 files
```

## 📌 주요 설정 파일 요약

### 📄 CMakeLists.txt (in /home/hi/ouster_ws/src/ouster-ros)
```
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(tf2_eigen REQUIRED)
find_package(CURL REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(
catkin_package(
find_package(OusterSDK REQUIRED)
target_link_libraries(ouster_ros
target_link_libraries(
  target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES}
```

### 📄 common.launch (in /home/hi/ouster_ws/src/ouster-ros/launch)
```
<arg name="ouster_ns" doc="Override the default namespace of all ouster nodes"/>
<arg name="viz" doc="whether to run a rviz"/>
<arg name="rviz_config" doc="optional rviz config file"/>
<arg name="tf_prefix" doc="namespace for tf transforms"/>
<arg name="sensor_frame" doc="
<arg name="lidar_frame" doc="
<arg name="imu_frame" doc="
<arg name="point_cloud_frame" doc="
<arg name="pub_static_tf" doc="
<arg name="timestamp_mode" doc="method used to timestamp measurements"/>
<arg name="ptp_utc_tai_offset" doc="UTC/TAI offset in seconds to apply when using TIME_FROM_PTP_1588"/>
<arg name="dynamic_transforms_broadcast" doc="static or dynamic transforms broadcast"/>
<arg name="dynamic_transforms_broadcast_rate"
<arg name="_no_bond" default="" doc="set the no-bond option when loading nodelets"/>
<arg name="proc_mask" doc="
<arg name="scan_ring" doc="
<arg name="point_type" doc="point type for the generated point cloud;
<arg name="organized" doc="generate an organzied point cloud"/>
<arg name="destagger" doc="enable or disable point cloud destaggering"/>
<arg name="min_range" doc="minimum lidar range to consider (meters)"/>
<arg name="max_range" doc="maximum lidar range to consider (meters)"/>
<arg name="min_scan_valid_columns_ratio"
<arg name="v_reduction" doc="vertical beam reduction; available options: {1, 2, 4, 8, 16}"/>
<arg name="mask_path" doc="path to an image file that will be used to mask parts of the pointcloud"/>
<node pkg="nodelet" type="nodelet" name="os_cloud_node"
<param name="~/tf_prefix" type="str" value="$(arg tf_prefix)"/>
<param name="~/sensor_frame" value="$(arg sensor_frame)"/>
<param name="~/lidar_frame" value="$(arg lidar_frame)"/>
<param name="~/imu_frame" value="$(arg imu_frame)"/>
<param name="~/point_cloud_frame" value="$(arg point_cloud_frame)"/>
<param name="~/pub_static_tf" value="$(arg pub_static_tf)"/>
<param name="~/timestamp_mode" type="str" value="$(arg timestamp_mode)"/>
<param name="~/dynamic_transforms_broadcast" type="bool"
<param name="~/dynamic_transforms_broadcast_rate" type="double"
<param name="~/proc_mask" value="$(arg proc_mask)"/>
<param name="~/scan_ring" value="$(arg scan_ring)"/>
<param name="~/ptp_utc_tai_offset" type="double" value="$(arg ptp_utc_tai_offset)"/>
<param name="~/point_type" value="$(arg point_type)"/>
<param name="~/organized" value="$(arg organized)"/>
<param name="~/destagger" value="$(arg destagger)"/>
<param name="~/min_range" value="$(arg min_range)"/>
<param name="~/max_range" value="$(arg max_range)"/>
<param name="~/v_reduction" value="$(arg v_reduction)"/>
<param name="~/mask_path" value="$(arg mask_path)"/>
<param name="~/min_scan_valid_columns_ratio"
<node pkg="nodelet" type="nodelet" name="img_node"
<param name="~/proc_mask" value="$(arg proc_mask)"/>
<param name="~/mask_path" value="$(arg mask_path)"/>
<node if="$(arg viz)"
```

### 📄 sensor_mtp.launch (in /home/hi/ouster_ws/src/ouster-ros/launch)
```
<arg name="ouster_ns" default="ouster" doc="Override the default namespace of all ouster nodes"/>
<arg name="sensor_hostname" doc="hostname or IP in dotted decimal form of the sensor"/>
<arg name="udp_dest" doc="hostname or multicast group IP where the sensor will send UDP data packets"/>
<arg name="mtp_dest" default=" " doc="hostname IP address for receiving data packets via multicast,
<arg name="mtp_main" default="false" doc="if true, then configure and reinit the sensor, otherwise
<arg name="lidar_port" default="0" doc="port to which the sensor should send lidar data"/>
<arg name="imu_port" default="0" doc="port to which the sensor should send imu data"/>
<arg name="udp_profile_lidar" default=" "
<arg name="lidar_mode" default=" "
<arg name="timestamp_mode" default=" "
<arg name="ptp_utc_tai_offset" default="-37.0"
<arg name="metadata" default=" " doc="path to write metadata file when receiving sensor data"/>
<arg name="viz" default="true" doc="whether to run a rviz"/>
<arg name="rviz_config" default="$(find ouster_ros)/config/viz.rviz" doc="optional rviz config file"/>
<arg name="tf_prefix" default=" " doc="namespace for tf transforms"/>
<arg name="sensor_frame" default="os_sensor"
<arg name="lidar_frame" default="os_lidar"
<arg name="imu_frame" default="os_imu"
<arg name="point_cloud_frame" default=" "
<arg name="pub_static_tf" default="true" doc="
<arg name="no_bond" default="false"
<arg if="$(arg no_bond)" name="_no_bond" value="--no-bond"/>
<arg unless="$(arg no_bond)" name="_no_bond" value=" "/>
<arg name="dynamic_transforms_broadcast" default="false"
<arg name="dynamic_transforms_broadcast_rate" default="1.0"
<arg name="proc_mask" default="IMG|PCL|IMU|SCAN|TLM" doc="
<arg name="scan_ring" default="0" doc="
<arg name="point_type" default="original"
<arg name="azimuth_window_start" default="0" doc="azimuth window start;
<arg name="azimuth_window_end" default="360000" doc="azimuth window end;
<arg name="persist_config" default="false"
<arg name="attempt_reconnect" default="false"
<arg name="dormant_period_between_reconnects" default="1.0"
<arg name="max_failed_reconnect_attempts" default="2147483647"
<arg name="organized" default="true"
<arg name="destagger" default="true"
<arg name="min_range" default="0.0"
<arg name="max_range" default="10000.0"
<arg name="min_scan_valid_columns_ratio" default="0.0"
<arg name="v_reduction" default="1"
<arg name="mask_path" default=""
<node pkg="nodelet" type="nodelet" name="os_nodelet_mgr"
<node pkg="nodelet" type="nodelet" name="os_node"
<param name="~/sensor_hostname" type="str" value="$(arg sensor_hostname)"/>
<param name="~/udp_dest" type="str" value="$(arg udp_dest)"/>
<param name="~/mtp_dest" type="str" value="$(arg mtp_dest)"/>
<param name="~/mtp_main" type="bool" value="$(arg mtp_main)"/>
<param name="~/lidar_port" type="int" value="$(arg lidar_port)"/>
<param name="~/imu_port" type="int" value="$(arg imu_port)"/>
<param name="~/udp_profile_lidar" type="str" value="$(arg udp_profile_lidar)"/>
<param name="~/lidar_mode" type="str" value="$(arg lidar_mode)"/>
<param name="~/timestamp_mode" type="str" value="$(arg timestamp_mode)"/>
<param name="~/ptp_utc_tai_offset" type="double"
<param name="~/metadata" type="str" value="$(arg metadata)"/>
<param name="~/proc_mask" type="str" value="$(arg proc_mask)"/>
<param name="~/azimuth_window_start" value="$(arg azimuth_window_start)"/>
<param name="~/azimuth_window_end" value="$(arg azimuth_window_end)"/>
<param name="~/persist_config" value="$(arg persist_config)"/>
<param name="~/attempt_reconnect" value="$(arg attempt_reconnect)"/>
<param name="~/dormant_period_between_reconnects"
<param name="~/max_failed_reconnect_attempts"
<arg name="ouster_ns" value="$(arg ouster_ns)"/>
<arg name="viz" value="$(arg viz)"/>
<arg name="rviz_config" value="$(arg rviz_config)"/>
<arg name="tf_prefix" value="$(arg tf_prefix)"/>
<arg name="sensor_frame" value="$(arg sensor_frame)"/>
<arg name="lidar_frame" value="$(arg lidar_frame)"/>
<arg name="imu_frame" value="$(arg imu_frame)"/>
<arg name="point_cloud_frame" value="$(arg point_cloud_frame)"/>
<arg name="pub_static_tf" value="$(arg pub_static_tf)"/>
<arg name="timestamp_mode" value="$(arg timestamp_mode)"/>
<arg name="ptp_utc_tai_offset" value="$(arg ptp_utc_tai_offset)"/>
<arg name="_no_bond" value="$(arg _no_bond)"/>
<arg name="dynamic_transforms_broadcast"
<arg name="dynamic_transforms_broadcast_rate"
<arg name="proc_mask" value="$(arg proc_mask)"/>
<arg name="scan_ring" value="$(arg scan_ring)"/>
<arg name="point_type" value="$(arg point_type)"/>
<arg name="organized" value="$(arg organized)"/>
<arg name="destagger" value="$(arg destagger)"/>
<arg name="min_range" value="$(arg min_range)"/>
<arg name="max_range" value="$(arg max_range)"/>
<arg name="v_reduction" value="$(arg v_reduction)"/>
<arg name="mask_path" value="$(arg mask_path)"/>
<arg name="min_scan_valid_columns_ratio"
```

### 📄 record.launch (in /home/hi/ouster_ws/src/ouster-ros/launch)
```
<arg name="ouster_ns" default="ouster" doc="Override the default namespace of all ouster nodes"/>
<arg name="sensor_hostname" doc="hostname or IP in dotted decimal form of the sensor"/>
<arg name="udp_dest" default=" " doc="hostname or IP where the sensor will send data packets"/>
<arg name="lidar_port" default="0" doc="port to which the sensor should send lidar data"/>
<arg name="imu_port" default="0" doc="port to which the sensor should send imu data"/>
<arg name="udp_profile_lidar" default=" " doc="lidar packet profile; possible values: {
<arg name="lidar_mode" default=" " doc="resolution and rate; possible values: {
<arg name="timestamp_mode" default=" " doc="method used to timestamp measurements; possible values: {
<arg name="ptp_utc_tai_offset" default="-37.0"
<arg name="metadata" default="" doc="path to write metadata file when receiving sensor data"/>
<arg name="bag_file" default="" doc="file name to use for the recorded bag file"/>
<arg name="viz" default="true" doc="whether to run a rviz"/>
<arg name="rviz_config" default="$(find ouster_ros)/config/viz.rviz" doc="optional rviz config file"/>
<arg name="tf_prefix" default=" " doc="namespace for tf transforms"/>
<arg name="sensor_frame" default="os_sensor"
<arg name="lidar_frame" default="os_lidar"
<arg name="imu_frame" default="os_imu"
<arg name="point_cloud_frame" default=" "
<arg name="pub_static_tf" default="true" doc="
<arg name="dynamic_transforms_broadcast" default="false"
<arg name="dynamic_transforms_broadcast_rate" default="1.0"
<arg name="proc_mask" default="IMG|PCL|IMU|SCAN|TLM" doc="
<arg name="scan_ring" default="0" doc="
<arg name="point_type" default="original" doc="point type for the generated point cloud;
<arg name="organized" default="true"
<arg name="destagger" default="true"
<arg name="min_range" default="0.0"
<arg name="max_range" default="10000.0"
<arg name="azimuth_window_start" default="0" doc="azimuth window start,
<arg name="azimuth_window_end" default="360000" doc="azimuth window end,
<arg name="persist_config" default="false"
<arg name="attempt_reconnect" default="false"
<arg name="dormant_period_between_reconnects" default="1.0"
<arg name="max_failed_reconnect_attempts" default="2147483647"
<arg name="min_scan_valid_columns_ratio" default="0.0"
<arg name="v_reduction" default="1"
<arg name="mask_path" default=""
<node pkg="nodelet" type="nodelet" name="os_nodelet_mgr"
<node pkg="nodelet" type="nodelet" name="os_node"
<param name="~/sensor_hostname" type="str" value="$(arg sensor_hostname)"/>
<param name="~/udp_dest" type="str" value="$(arg udp_dest)"/>
<param name="~/lidar_port" type="int" value="$(arg lidar_port)"/>
<param name="~/imu_port" type="int" value="$(arg imu_port)"/>
<param name="~/udp_profile_lidar" type="str" value="$(arg udp_profile_lidar)"/>
<param name="~/lidar_mode" type="str" value="$(arg lidar_mode)"/>
<param name="~/timestamp_mode" type="str" value="$(arg timestamp_mode)"/>
<param name="~/ptp_utc_tai_offset" type="double"
<param name="~/metadata" type="str" value="$(arg metadata)"/>
<param name="~/proc_mask" type="str" value="$(arg proc_mask)"/>
<param name="~/azimuth_window_start" value="$(arg azimuth_window_start)"/>
<param name="~/azimuth_window_end" value="$(arg azimuth_window_end)"/>
<param name="~/persist_config" value="$(arg persist_config)"/>
<param name="~/attempt_reconnect" value="$(arg attempt_reconnect)"/>
<param name="~/dormant_period_between_reconnects"
<param name="~/max_failed_reconnect_attempts"
<arg name="ouster_ns" value="$(arg ouster_ns)"/>
<arg name="viz" value="$(arg viz)"/>
<arg name="rviz_config" value="$(arg rviz_config)"/>
<arg name="tf_prefix" value="$(arg tf_prefix)"/>
<arg name="sensor_frame" value="$(arg sensor_frame)"/>
<arg name="lidar_frame" value="$(arg lidar_frame)"/>
<arg name="imu_frame" value="$(arg imu_frame)"/>
<arg name="point_cloud_frame" value="$(arg point_cloud_frame)"/>
<arg name="pub_static_tf" value="$(arg pub_static_tf)"/>
<arg name="timestamp_mode" value="$(arg timestamp_mode)"/>
<arg name="ptp_utc_tai_offset" value="$(arg ptp_utc_tai_offset)"/>
<arg name="dynamic_transforms_broadcast"
<arg name="dynamic_transforms_broadcast_rate"
<arg name="proc_mask" value="$(arg proc_mask)"/>
<arg name="scan_ring" value="$(arg scan_ring)"/>
<arg name="point_type" value="$(arg point_type)"/>
<arg name="organized" value="$(arg organized)"/>
<arg name="destagger" value="$(arg destagger)"/>
<arg name="min_range" value="$(arg min_range)"/>
<arg name="max_range" value="$(arg max_range)"/>
<arg name="v_reduction" value="$(arg v_reduction)"/>
<arg name="~/mask_path" value="$(arg mask_path)"/>
<arg name="min_scan_valid_columns_ratio"
<arg name="_use_bag_file_name" value="$(eval not (bag_file == ''))"/>
<arg name="_topics_to_record" value="
<node if="$(arg _use_bag_file_name)" pkg="rosbag" type="record"
<node unless="$(arg _use_bag_file_name)" pkg="rosbag" type="record"
```

### 📄 driver.launch (in /home/hi/ouster_ws/src/ouster-ros/launch)
```
<arg name="ouster_ns" default="ouster" doc="Override the default namespace of all ouster nodes"/>
<arg name="sensor_hostname" doc="hostname or IP in dotted decimal form of the sensor"/>
<arg name="udp_dest" default=" " doc="hostname or IP where the sensor will send data packets"/>
<arg name="lidar_port" default="0" doc="port to which the sensor should send lidar data"/>
<arg name="imu_port" default="0" doc="port to which the sensor should send imu data"/>
<arg name="udp_profile_lidar" default=" "
<arg name="lidar_mode" default=" "
<arg name="timestamp_mode" default="TIME_FROM_PTP_1588"
<arg name="ptp_utc_tai_offset" default="-37.0"
<arg name="metadata" default=" " doc="path to write metadata file when receiving sensor data"/>
<arg name="viz" default="true" doc="whether to run a rviz"/>
<arg name="rviz_config" default="$(find ouster_ros)/config/viz.rviz" doc="optional rviz config file"/>
<arg name="tf_prefix" default=" " doc="namespace for tf transforms"/>
<arg name="sensor_frame" default="os_sensor"
<arg name="lidar_frame" default="os_lidar"
<arg name="imu_frame" default="os_imu"
<arg name="point_cloud_frame" default=" "
<arg name="pub_static_tf" default="true" doc="
<arg name="no_bond" default="false"
<arg if="$(arg no_bond)" name="_no_bond" value="--no-bond"/>
<arg unless="$(arg no_bond)" name="_no_bond" value=" "/>
<arg name="proc_mask" default="IMU|PCL|SCAN|IMG|RAW|TLM" doc="
<arg name="scan_ring" default="0" doc="
<arg name="point_type" default="original"
<arg name="azimuth_window_start" default="0" doc="azimuth window start;
<arg name="azimuth_window_end" default="360000" doc="azimuth window end;
<arg name="persist_config" default="false"
<arg name="attempt_reconnect" default="false"
<arg name="dormant_period_between_reconnects" default="1.0"
<arg name="max_failed_reconnect_attempts" default="2147483647"
<arg name="organized" default="true"
<arg name="destagger" default="true"
<arg name="min_range" default="0.0"
<arg name="max_range" default="10000.0"
<arg name="min_scan_valid_columns_ratio" default="0.0"
<arg name="v_reduction" default="1"
<arg name="mask_path" default=""
<node pkg="nodelet" type="nodelet" name="os_nodelet_mgr"
<node pkg="nodelet" type="nodelet" name="os_driver"
<param name="~/sensor_hostname" type="str" value="$(arg sensor_hostname)"/>
<param name="~/udp_dest" type="str" value="$(arg udp_dest)"/>
<param name="~/lidar_port" type="int" value="$(arg lidar_port)"/>
<param name="~/imu_port" type="int" value="$(arg imu_port)"/>
<param name="~/udp_profile_lidar" type="str" value="$(arg udp_profile_lidar)"/>
<param name="~/lidar_mode" type="str" value="$(arg lidar_mode)"/>
<param name="~/timestamp_mode" type="str" value="$(arg timestamp_mode)"/>
<param name="~/ptp_utc_tai_offset" type="double"
<param name="~/metadata" type="str" value="$(arg metadata)"/>
<param name="~/tf_prefix" value="$(arg tf_prefix)"/>
<param name="~/sensor_frame" value="$(arg sensor_frame)"/>
<param name="~/lidar_frame" value="$(arg lidar_frame)"/>
<param name="~/imu_frame" value="$(arg imu_frame)"/>
<param name="~/point_cloud_frame" value="$(arg point_cloud_frame)"/>
<param name="~/pub_static_tf" value="$(arg pub_static_tf)"/>
<param name="~/proc_mask" value="$(arg proc_mask)"/>
<param name="~/scan_ring" value="$(arg scan_ring)"/>
<param name="~/point_type" value="$(arg point_type)"/>
<param name="~/azimuth_window_start" value="$(arg azimuth_window_start)"/>
<param name="~/azimuth_window_end" value="$(arg azimuth_window_end)"/>
<param name="~/persist_config" value="$(arg persist_config)"/>
<param name="~/attempt_reconnect" value="$(arg attempt_reconnect)"/>
<param name="~/dormant_period_between_reconnects"
<param name="~/max_failed_reconnect_attempts"
<param name="~/organized" value="$(arg organized)"/>
<param name="~/destagger" value="$(arg destagger)"/>
<param name="~/min_range" value="$(arg min_range)"/>
<param name="~/max_range" value="$(arg max_range)"/>
<param name="~/v_reduction" value="$(arg v_reduction)"/>
<param name="~/mask_path" value="$(arg mask_path)"/>
<param name="~/min_scan_valid_columns_ratio"
<node if="$(arg viz)" pkg="rviz" name="rviz" type="rviz"
```

### 📄 replay.launch (in /home/hi/ouster_ws/src/ouster-ros/launch)
```
<param name="use_sim_time" value="true"/>
<arg name="loop" default="false" doc="request loop playback"/>
<arg name="play_delay" default="0" doc="playback start delay in seconds"/>
<arg name="play_rate" default="1.0"/>
<arg if="$(arg loop)" name="_loop" value="--loop"/>
<arg unless="$(arg loop)" name="_loop" value=" "/>
<arg name="ouster_ns" default="ouster" doc="Override the default namespace of all ouster nodes"/>
<arg name="metadata" default="" doc="path to read metadata file when replaying sensor data"/>
<arg name="bag_file" default="" doc="file name to use for the recorded bag file"/>
<arg name="timestamp_mode" default="TIME_FROM_INTERNAL_OSC"
<arg name="ptp_utc_tai_offset" default="-37.0"
<arg name="viz" default="true" doc="whether to run a rviz"/>
<arg name="rviz_config" default="$(find ouster_ros)/config/viz.rviz" doc="optional rviz config file"/>
<arg name="tf_prefix" default=" " doc="namespace for tf transforms"/>
<arg name="sensor_frame" default="os_sensor"
<arg name="lidar_frame" default="os_lidar"
<arg name="imu_frame" default="os_imu"
<arg name="point_cloud_frame" default=" "
<arg name="pub_static_tf" default="true" doc="
<arg name="dynamic_transforms_broadcast" default="false"
<arg name="dynamic_transforms_broadcast_rate" default="1.0"
<arg name="proc_mask" default="IMG|PCL|IMU|SCAN" doc="
<arg name="scan_ring" default="0" doc="
<arg name="point_type" default="original"
<arg name="organized" default="true"
<arg name="destagger" default="true"
<arg name="min_range" default="0.0"
<arg name="max_range" default="10000.0"
<arg name="min_scan_valid_columns_ratio" default="0.0"
<arg name="v_reduction" default="1"
<arg name="mask_path" default=""
<node pkg="nodelet" type="nodelet" name="os_nodelet_mgr"
<arg name="_use_metadata_file" value="$(eval not (metadata == ''))"/>
<node if="$(arg _use_metadata_file)" pkg="nodelet" type="nodelet"
<param name="~/metadata" value="$(arg metadata)"/>
<arg name="ouster_ns" value="$(arg ouster_ns)"/>
<arg name="viz" value="$(arg viz)"/>
<arg name="rviz_config" value="$(arg rviz_config)"/>
<arg name="tf_prefix" value="$(arg tf_prefix)"/>
<arg name="sensor_frame" value="$(arg sensor_frame)"/>
<arg name="lidar_frame" value="$(arg lidar_frame)"/>
<arg name="imu_frame" value="$(arg imu_frame)"/>
<arg name="point_cloud_frame" value="$(arg point_cloud_frame)"/>
<arg name="pub_static_tf" value="$(arg pub_static_tf)"/>
<arg name="timestamp_mode" value="$(arg timestamp_mode)"/>
<arg name="ptp_utc_tai_offset" value="$(arg ptp_utc_tai_offset)"/>
<arg name="dynamic_transforms_broadcast"
<arg name="dynamic_transforms_broadcast_rate"
<arg name="proc_mask" value="$(arg proc_mask)"/>
<arg name="scan_ring" value="$(arg scan_ring)"/>
<arg name="point_type" value="$(arg point_type)"/>
<arg name="organized" value="$(arg organized)"/>
<arg name="destagger" value="$(arg destagger)"/>
<arg name="min_range" value="$(arg min_range)"/>
<arg name="max_range" value="$(arg max_range)"/>
<arg name="v_reduction" value="$(arg v_reduction)"/>
<arg name="mask_path" value="$(arg mask_path)"/>
<arg name="min_scan_valid_columns_ratio"
<arg name="_use_bag_file_name" value="$(eval not (bag_file == ''))"/>
<node if="$(arg _use_bag_file_name)" pkg="rosbag" type="play"
```

### 📄 replay_pcap.launch (in /home/hi/ouster_ws/src/ouster-ros/launch)
```
<param name="use_sim_time" value="false"/>
<arg name="loop" default="false" doc="request loop playback"/>
<arg name="play_delay" default="0" doc="playback start delay in seconds"/>
<arg name="progress_update_freq" default="3.0"
<arg name="ouster_ns" default="ouster" doc="Override the default namespace of all ouster nodes"/>
<arg name="metadata" doc="path to read metadata file when replaying sensor data"/>
<arg name="pcap_file" doc="file name to use for the recorded pcap file"/>
<arg name="timestamp_mode" default="TIME_FROM_INTERNAL_OSC"
<arg name="ptp_utc_tai_offset" default="-37.0"
<arg name="viz" default="true" doc="whether to run a rviz"/>
<arg name="rviz_config" default="$(find ouster_ros)/config/viz.rviz" doc="optional rviz config file"/>
<arg name="tf_prefix" default=" " doc="namespace for tf transforms"/>
<arg name="sensor_frame" default="os_sensor"
<arg name="lidar_frame" default="os_lidar"
<arg name="imu_frame" default="os_imu"
<arg name="point_cloud_frame" default=" "
<arg name="pub_static_tf" default="true" doc="
<arg name="dynamic_transforms_broadcast" default="false"
<arg name="dynamic_transforms_broadcast_rate" default="1.0"
<arg name="proc_mask" default="IMG|PCL|IMU|SCAN" doc="
<arg name="scan_ring" default="0" doc="
<arg name="point_type" default="original"
<arg name="organized" default="true"
<arg name="destagger" default="true"
<arg name="min_range" default="0.0"
<arg name="max_range" default="10000.0"
<arg name="min_scan_valid_columns_ratio" default="0.0"
<arg name="v_reduction" default="1"
<arg name="mask_path" default=""
<node pkg="nodelet" type="nodelet" name="os_nodelet_mgr"
<node pkg="nodelet" type="nodelet"
<param name="~/metadata" value="$(arg metadata)"/>
<param name="~/pcap_file" value="$(arg pcap_file)"/>
<param name="~/loop" value="$(arg loop)"/>
<param name="~/progress_update_freq" value="$(arg progress_update_freq)"/>
<arg name="ouster_ns" value="$(arg ouster_ns)"/>
<arg name="viz" value="$(arg viz)"/>
<arg name="rviz_config" value="$(arg rviz_config)"/>
<arg name="tf_prefix" value="$(arg tf_prefix)"/>
<arg name="sensor_frame" value="$(arg sensor_frame)"/>
<arg name="lidar_frame" value="$(arg lidar_frame)"/>
<arg name="imu_frame" value="$(arg imu_frame)"/>
<arg name="point_cloud_frame" value="$(arg point_cloud_frame)"/>
<arg name="pub_static_tf" value="$(arg pub_static_tf)"/>
<arg name="timestamp_mode" value="$(arg timestamp_mode)"/>
<arg name="ptp_utc_tai_offset" value="$(arg ptp_utc_tai_offset)"/>
<arg name="dynamic_transforms_broadcast"
<arg name="dynamic_transforms_broadcast_rate"
<arg name="proc_mask" value="$(arg proc_mask)"/>
<arg name="scan_ring" value="$(arg scan_ring)"/>
<arg name="point_type" value="$(arg point_type)"/>
<arg name="organized" value="$(arg organized)"/>
<arg name="destagger" value="$(arg destagger)"/>
<arg name="min_range" value="$(arg min_range)"/>
<arg name="max_range" value="$(arg max_range)"/>
<arg name="v_reduction" value="$(arg v_reduction)"/>
<arg name="mask_path" value="$(arg mask_path)"/>
<arg name="min_scan_valid_columns_ratio"
```

### 📄 sensor.launch (in /home/hi/ouster_ws/src/ouster-ros/launch)
```
<arg name="ouster_ns" default="ouster" doc="Override the default namespace of all ouster nodes"/>
<arg name="sensor_hostname" doc="hostname or IP in dotted decimal form of the sensor"/>
<arg name="udp_dest" default=" " doc="hostname or IP where the sensor will send data packets"/>
<arg name="lidar_port" default="0" doc="port to which the sensor should send lidar data"/>
<arg name="imu_port" default="0" doc="port to which the sensor should send imu data"/>
<arg name="udp_profile_lidar" default=" "
<arg name="lidar_mode" default=" "
<arg name="timestamp_mode" default=" "
<arg name="ptp_utc_tai_offset" default="-37.0"
<arg name="metadata" default=" " doc="path to write metadata file when receiving sensor data"/>
<arg name="viz" default="true" doc="whether to run a rviz"/>
<arg name="rviz_config" default="$(find ouster_ros)/config/viz.rviz" doc="optional rviz config file"/>
<arg name="tf_prefix" default=" " doc="namespace for tf transforms"/>
<arg name="sensor_frame" default="os_sensor"
<arg name="lidar_frame" default="os_lidar"
<arg name="imu_frame" default="os_imu"
<arg name="point_cloud_frame" default=" "
<arg name="pub_static_tf" default="true" doc="
<arg name="no_bond" default="false"
<arg if="$(arg no_bond)" name="_no_bond" value="--no-bond"/>
<arg unless="$(arg no_bond)" name="_no_bond" value=" "/>
<arg name="dynamic_transforms_broadcast" default="false"
<arg name="dynamic_transforms_broadcast_rate" default="1.0"
<arg name="proc_mask" default="IMG|PCL|IMU|SCAN|TLM" doc="
<arg name="scan_ring" default="0" doc="
<arg name="point_type" default="original"
<arg name="azimuth_window_start" default="0" doc="azimuth window start;
<arg name="azimuth_window_end" default="360000" doc="azimuth window end;
<arg name="persist_config" default="false"
<arg name="attempt_reconnect" default="false"
<arg name="dormant_period_between_reconnects" default="1.0"
<arg name="max_failed_reconnect_attempts" default="2147483647"
<arg name="organized" default="true"
<arg name="destagger" default="true"
<arg name="min_range" default="0.0"
<arg name="max_range" default="10000.0"
<arg name="min_scan_valid_columns_ratio" default="0.0"
<arg name="v_reduction" default="1"
<arg name="mask_path" default=""
<node pkg="nodelet" type="nodelet" name="os_nodelet_mgr"
<node pkg="nodelet" type="nodelet" name="os_node"
<param name="~/sensor_hostname" type="str" value="$(arg sensor_hostname)"/>
<param name="~/udp_dest" type="str" value="$(arg udp_dest)"/>
<param name="~/lidar_port" type="int" value="$(arg lidar_port)"/>
<param name="~/imu_port" type="int" value="$(arg imu_port)"/>
<param name="~/udp_profile_lidar" type="str" value="$(arg udp_profile_lidar)"/>
<param name="~/lidar_mode" type="str" value="$(arg lidar_mode)"/>
<param name="~/timestamp_mode" type="str" value="$(arg timestamp_mode)"/>
<param name="~/ptp_utc_tai_offset" type="double"
<param name="~/metadata" type="str" value="$(arg metadata)"/>
<param name="~/proc_mask" type="str" value="$(arg proc_mask)"/>
<param name="~/azimuth_window_start" value="$(arg azimuth_window_start)"/>
<param name="~/azimuth_window_end" value="$(arg azimuth_window_end)"/>
<param name="~/persist_config" value="$(arg persist_config)"/>
<param name="~/attempt_reconnect" value="$(arg attempt_reconnect)"/>
<param name="~/dormant_period_between_reconnects"
<param name="~/max_failed_reconnect_attempts"
<arg name="ouster_ns" value="$(arg ouster_ns)"/>
<arg name="viz" value="$(arg viz)"/>
<arg name="rviz_config" value="$(arg rviz_config)"/>
<arg name="tf_prefix" value="$(arg tf_prefix)"/>
<arg name="sensor_frame" value="$(arg sensor_frame)"/>
<arg name="lidar_frame" value="$(arg lidar_frame)"/>
<arg name="imu_frame" value="$(arg imu_frame)"/>
<arg name="point_cloud_frame" value="$(arg point_cloud_frame)"/>
<arg name="pub_static_tf" value="$(arg pub_static_tf)"/>
<arg name="timestamp_mode" value="$(arg timestamp_mode)"/>
<arg name="ptp_utc_tai_offset" value="$(arg ptp_utc_tai_offset)"/>
<arg name="_no_bond" value="$(arg _no_bond)"/>
<arg name="dynamic_transforms_broadcast"
<arg name="dynamic_transforms_broadcast_rate"
<arg name="proc_mask" value="$(arg proc_mask)"/>
<arg name="scan_ring" value="$(arg scan_ring)"/>
<arg name="point_type" value="$(arg point_type)"/>
<arg name="organized" value="$(arg organized)"/>
<arg name="destagger" value="$(arg destagger)"/>
<arg name="min_range" value="$(arg min_range)"/>
<arg name="max_range" value="$(arg max_range)"/>
<arg name="v_reduction" value="$(arg v_reduction)"/>
<arg name="mask_path" value="$(arg mask_path)"/>
<arg name="min_scan_valid_columns_ratio"
```

### 📄 CMakeLists.txt (in /home/hi/ouster_ws/src/ouster-ros/ouster-sdk/tests)
```
find_package(GTest REQUIRED)
find_package(jsoncpp REQUIRED)
add_executable(bcompat_meta_json_test
target_link_libraries(bcompat_meta_json_test PRIVATE
add_executable(metadata_test metadata_test.cpp)
target_link_libraries(metadata_test
add_executable(lidar_scan_test lidar_scan_test.cpp)
target_link_libraries(lidar_scan_test PRIVATE OusterSDK::ouster_client GTest::gtest GTest::gtest_main)
add_executable(cartesian_test cartesian_test.cpp util.h)
target_link_libraries(cartesian_test PRIVATE OusterSDK::ouster_client GTest::gtest GTest::gtest_main)
add_executable(metadata_errors_test metadata_errors_test.cpp)
target_link_libraries(metadata_errors_test PRIVATE OusterSDK::ouster_client GTest::gtest GTest::gtest_main)
add_executable(pcap_test pcap_test.cpp)
target_link_libraries(pcap_test PRIVATE OusterSDK::ouster_pcap GTest::gtest GTest::gtest_main)
add_executable(profile_extension_test profile_extension_test.cpp)
target_link_libraries(profile_extension_test PRIVATE OusterSDK::ouster_client GTest::gtest GTest::gtest_main)
add_executable(fusa_profile_test fusa_profile_test.cpp)
target_link_libraries(fusa_profile_test PRIVATE OusterSDK::ouster_client OusterSDK::ouster_pcap GTest::gtest GTest::gtest_main)
add_executable(parsing_benchmark_test parsing_benchmark_test.cpp util.h)
target_link_libraries(parsing_benchmark_test PRIVATE OusterSDK::ouster_client OusterSDK::ouster_pcap GTest::gtest GTest::gtest_main)
add_executable(scan_batcher_test scan_batcher_test.cpp util.h)
target_link_libraries(scan_batcher_test PRIVATE OusterSDK::ouster_client OusterSDK::ouster_pcap GTest::gtest GTest::gtest_main)
add_executable(packet_writer_test packet_writer_test.cpp util.h)
target_link_libraries(packet_writer_test PRIVATE OusterSDK::ouster_client OusterSDK::ouster_pcap GTest::gtest GTest::gtest_main)
add_executable(udp_queue_test udp_queue_test.cpp)
target_link_libraries(udp_queue_test PRIVATE OusterSDK::ouster_client OusterSDK::ouster_pcap GTest::gtest GTest::gtest_main)
```

### 📄 CMakeLists.txt (in /home/hi/ouster_ws/src/ouster-ros/ouster-sdk/examples/linking_example)
```
find_package(OusterSDK REQUIRED)
add_executable(pcap_test main.cpp)
target_link_libraries(pcap_test
```

### 📄 CMakeLists.txt (in /home/hi/ouster_ws/src/ouster-ros/ouster-sdk/examples)
```
add_executable(client_example client_example.cpp)
target_link_libraries(client_example PRIVATE OusterSDK::ouster_client)
add_executable(async_client_example async_client_example.cpp)
target_link_libraries(async_client_example PRIVATE OusterSDK::ouster_client)
add_executable(mtp_client_example mtp_client_example.cpp)
target_link_libraries(mtp_client_example PRIVATE OusterSDK::ouster_client)
add_executable(config_example config_example.cpp)
target_link_libraries(config_example PRIVATE OusterSDK::ouster_client)
  add_executable(lidar_scan_example lidar_scan_example.cpp helpers.cpp)
  target_link_libraries(lidar_scan_example PRIVATE OusterSDK::ouster_client OusterSDK::ouster_pcap)
  add_executable(representations_example representations_example.cpp helpers.cpp)
  target_link_libraries(representations_example PRIVATE OusterSDK::ouster_client OusterSDK::ouster_pcap)
  add_executable(osf_reader_example osf_reader_example.cpp)
  target_link_libraries(osf_reader_example PRIVATE OusterSDK::ouster_osf)
  add_executable(osf_writer_example osf_writer_example.cpp)
  target_link_libraries(osf_writer_example PRIVATE OusterSDK::ouster_osf)
  add_executable(viz_example viz_example.cpp)
  target_link_libraries(viz_example PRIVATE OusterSDK::ouster_client OusterSDK::ouster_viz)
```

### 📄 CMakeLists.txt (in /home/hi/ouster_ws/src/ouster-ros/ouster-sdk)
```
```

### 📄 CMakeLists.txt (in /home/hi/ouster_ws/src/ouster-ros/ouster-sdk/ouster_osf/tests)
```
find_package(GTest REQUIRED)
find_package(OpenSSL REQUIRED)
  add_executable(osf_${TEST_FILENAME} ${TEST_FULL_NAME})
  target_link_libraries(osf_${TEST_FILENAME} PRIVATE ouster_osf
```

### 📄 CMakeLists.txt (in /home/hi/ouster_ws/src/ouster-ros/ouster-sdk/ouster_osf)
```
find_package(ZLIB REQUIRED)
find_package(PNG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(spdlog REQUIRED)
  find_package(Flatbuffers REQUIRED)
  find_package(flatbuffers REQUIRED)
target_link_libraries(ouster_osf
```

### 📄 CMakeLists.txt (in /home/hi/ouster_ws/src/ouster-ros/ouster-sdk/ouster_pcap)
```
find_package(Pcap REQUIRED)
find_package(libtins REQUIRED)
  target_link_libraries(ouster_pcap PUBLIC ws2_32)
target_link_libraries(ouster_pcap
```

### 📄 CMakeLists.txt (in /home/hi/ouster_ws/src/ouster-ros/ouster-sdk/ouster_client)
```
find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(CURL REQUIRED)
find_package(spdlog REQUIRED)
target_link_libraries(ouster_client
  target_link_libraries(ouster_client PUBLIC ws2_32)
```

### 📄 CMakeLists.txt (in /home/hi/ouster_ws/src/ouster-ros/ouster-sdk/python)
```
find_package(Pybind11Internal)
find_package(Eigen3 REQUIRED)
find_package(OusterSDK REQUIRED)
find_package(spdlog REQUIRED)
target_link_libraries(_client
target_link_libraries(_pcap PRIVATE ouster_pcap ouster_build)
target_link_libraries(_viz PRIVATE ouster_client ouster_viz ouster_build)
target_link_libraries(_osf PRIVATE ouster_osf ouster_build)
```

### 📄 CMakeLists.txt (in /home/hi/ouster_ws/src/ouster-ros/ouster-sdk/conan/test_package)
```
find_package(OusterSDK REQUIRED)
```

### 📄 CMakeLists.txt (in /home/hi/ouster_ws/src/ouster-ros/ouster-sdk/ouster_viz)
```
find_package(OpenGL REQUIRED)
find_package(glad QUIET)
  find_package(glad REQUIRED)
  find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)
target_link_libraries(ouster_viz
```

### 📄 package.xml (in /home/hi/ouster_ws/src/ouster-ros)
```
  <name>ouster_ros</name>
  <version>0.13.7</version>
  <description>Ouster ROS driver</description>
  <depend>roscpp</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>pcl_ros</depend>
  <depend>pcl_conversions</depend>
  <depend>cv_bridge</depend>
```

### 📄 params.yaml (in /home/hi/ouster_ws/src/LIO-SAM/config)
```
lio_sam:
  pointCloudTopic: "/points_filtered"
  imuTopic: "/imu/data_synced"                        # IMU data, /imu/data ->/imu/data/synced
  odomTopic: "odometry/imu"                   # IMU pre-preintegration odometry, same frequency as IMU
  gpsTopic: "odometry/gpsz"                   # GPS odometry topic from navsat, see module_navsat.launch file
  lidarFrame: "os_sensor"
  baselinkFrame: "imu_link"  # 만약 IMU가 base_link 역할이라면
  odometryFrame: "odom"
  mapFrame: "map"
  useImuHeadingInitialization: true           # if using GPS data, set to "true"
  useGpsElevation: false                      # if GPS elevation is bad, set to "false"
  gpsCovThreshold: 2.0                        # m^2, threshold for using GPS data
  poseCovThreshold: 25.0                      # m^2, threshold for using GPS data
  
  savePCD: false                              # https://github.com/TixiaoShan/LIO-SAM/issues/3
  savePCDDirectory: "/Downloads/LOAM/"        # in your home folder, starts and ends with "/". Warning: the code deletes "LOAM" folder then recreates it. See "mapOptimization" for implementation
  sensor: ouster                           # lidar sensor type, 'velodyne' or 'ouster' or 'livox'
  N_SCAN: 128                                  # number of lidar channel (i.e., Velodyne/Ouster: 16, 32, 64, 128, Livox Horizon: 6)
  Horizon_SCAN: 1024                          # lidar horizontal resolution (Velodyne:1800, Ouster:512,1024,2048, Livox Horizon: 4000)
  downsampleRate: 1                           # default: 1. Downsample your data if too many points. i.e., 16 = 64 / 4, 16 = 16 / 1
```

### 📄 CMakeLists.txt (in /home/hi/ouster_ws/src/LIO-SAM)
```
find_package(catkin REQUIRED COMPONENTS
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)          # PCL 패키지 찾기
find_package(OpenCV REQUIRED QUIET)       # OpenCV 찾기
find_package(GTSAM REQUIRED QUIET)        # GTSAM 찾기
find_package(Boost REQUIRED COMPONENTS timer serialization)  # Boost.timer + Boost.serialization
catkin_package(
add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
target_link_libraries(
add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
target_link_libraries(
add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmization.cpp)
target_link_libraries(
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
target_link_libraries(
add_executable(lidar_imu_sync src/lidar_imu_sync.cpp)
target_link_libraries(lidar_imu_sync ${catkin_LIBRARIES})
```

### 📄 module_robot_state_publisher.launch (in /home/hi/ouster_ws/src/LIO-SAM/launch/include)
```
<arg name="project" default="lio_sam"/>
<param name="robot_description" command="$(find xacro)/xacro $(find lio_sam)/launch/include/config/robot.urdf.xacro --inorder" />
<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="true">
<!-- <param name="tf_prefix" value="$(env ROS_HOSTNAME)"/> -->
```

### 📄 module_navsat.launch (in /home/hi/ouster_ws/src/LIO-SAM/launch/include)
```
<arg name="project" default="lio_sam"/>
<node pkg="robot_localization" type="ekf_localization_node" name="ekf_gps" respawn="true">
<node pkg="robot_localization" type="navsat_transform_node" name="navsat" respawn="true">
```

### 📄 module_rviz.launch (in /home/hi/ouster_ws/src/LIO-SAM/launch/include)
```
<arg name="project" default="lio_sam"/>
<node pkg="rviz" type="rviz" name="$(arg project)_rviz" args="-d $(find lio_sam)/launch/include/config/rviz.rviz" />
```

### 📄 module_loam.launch (in /home/hi/ouster_ws/src/LIO-SAM/launch/include)
```
<arg name="project" default="lio_sam"/>
<node pkg="$(arg project)" type="$(arg project)_imuPreintegration"   name="$(arg project)_imuPreintegration"    output="screen" 	respawn="true"/>
<node pkg="$(arg project)" type="$(arg project)_imageProjection"     name="$(arg project)_imageProjection"      output="screen"     respawn="true"/>
<node pkg="$(arg project)" type="$(arg project)_featureExtraction"   name="$(arg project)_featureExtraction"    output="screen"     respawn="true"/>
<node pkg="$(arg project)" type="$(arg project)_mapOptmization"      name="$(arg project)_mapOptmization"       output="screen"     respawn="true">
<param name="publishTF" value="true" />
```

### 📄 run.launch (in /home/hi/ouster_ws/src/LIO-SAM/launch)
```
<arg name="project" default="lio_sam"/>
```

### 📄 package.xml (in /home/hi/ouster_ws/src/LIO-SAM)
```
  <name>lio_sam</name>
  <version>1.0.0</version>
  <description>Lidar Odometry</description>
```

### 📄 CMakeLists.txt (in /home/hi/ouster_ws/src/xsens_ros_mti_driver)
```
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
find_package(catkin REQUIRED COMPONENTS
## The catkin_package macro generates cmake config files for your package
## CATKIN_DEPENDS: catkin_packages dependent projects also need
catkin_package(
add_executable(
target_link_libraries(
```

### 📄 display.launch (in /home/hi/ouster_ws/src/xsens_ros_mti_driver/launch)
```
<arg name="model" default="$(find xsens_mti_driver)/urdf/MTi_6xx.urdf"/>
<arg name="rvizconfig" default="$(find xsens_mti_driver)/rviz/display.rviz" />
<param name="robot_description" command="$(find xacro)/xacro --inorder $(arg model)" />
<node name="rviz" pkg="rviz" type="rviz" args="-d $(arg rvizconfig)" required="true" />
```

### 📄 xsens_mti_node.launch (in /home/hi/ouster_ws/src/xsens_ros_mti_driver/launch)
```
<node  name="xsens_mti_node" pkg="xsens_mti_driver" type="xsens_mti_node" output="screen">
```

### 📄 package.xml (in /home/hi/ouster_ws/src/xsens_ros_mti_driver)
```
  <name>xsens_mti_driver</name>
  <version>1.0.0</version>
  <description>ROS driver for Xsens MTi IMU sensors</description>
```
