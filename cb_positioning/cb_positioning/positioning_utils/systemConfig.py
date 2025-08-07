"""
@file       systemConfig.py
@brief      System-wide configuration settings for human position estimation node
@details    This configuration file stores global constants used across multiple modules in the human 
            positioning system. It defines the root path for data dependencies such as models, 
            calibration files, and other assets required by the computer vision and tracking pipeline.
@par        Usage:
            Include this file wherever a consistent base path to model, label, or calibration files is needed.
@par        Example:
@code
import systemConfig

LABEL_PATH = os.path.join(BASE_PATH, "coral-models", "coco_labels.txt")
@endcode
"""

## This path should be updated according to the deployment system.
#  All other relative paths throughout the vision pipeline are constructed based on this value."""
# Base_path = "/home/cb/Desktop/human-position-publisher/positioning"
Base_path = "/home/cb/Desktop/cb_workspace/src/cb_ros2/cb_positioning/cb_positioning/positioning_utils"