class ConfigROS():
    map_frame: str = "map"

    pointcloud_topic: str = "/global_points"
    layer_G_topic: str = "/layer_G_"
    layer_C_topic: str = "/layer_C_"
    tomogram_topic: str = "/tomogram"


class ConfigMap():
    export_dir: str = "/rsc/tomogram/"


class Config():
    ros = ConfigROS()
    map = ConfigMap()
