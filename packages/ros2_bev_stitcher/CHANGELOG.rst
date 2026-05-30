^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_bev_stitcher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-03-15)
-------------------
* Initial release: multi-camera Bird's Eye View compositor for ROS 2
* Extracted and generalized from the OmniBot omnibot_lerobot package
* ``bev_stitcher_node.py``: subscribes to N perspective cameras, publishes
  a single top-down BEV image

  * Camera list fully configurable via ``camera_names`` ROS 2 parameter
    (not hardcoded to front/rear/left/right)
  * Topic pattern configurable: ``input_topic_pattern`` (default
    ``/camera/{name}/image_raw``)
  * Canvas size, output resolution, publish rate all parameterized
  * Per-camera homography warp via OpenCV ``warpPerspective``
  * Distance-weighted alpha blending in overlap regions
  * Graceful fallback to tiled 2×N grid when calibration file absent
  * ``calibration_file`` ROS 2 parameter (default ``~/bev_calibration.npz``)

* ``bev_calibrate.py``: checkerboard-based one-time calibration tool

  * Subscribes to all camera topics, captures one frame each
  * Detects inner corners of a flat checkerboard (configurable pattern size
    and square size in metres)
  * Computes RANSAC homography per camera to shared BEV ground plane
  * Shows warped preview per camera before saving
  * Saves all homographies to ``.npz`` file loaded by the stitcher node
  * CLI: ``ros2 run ros2_bev_stitcher bev_calibrate --help``

* Launch file: ``bev_stitcher.launch.py``
* Param YAML: ``config/bev_params.yaml``
* Contributors: Varun Vaidhiya
