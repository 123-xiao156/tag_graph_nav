# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/jsoncpp".split(';') if "${prefix}/include;/usr/include/jsoncpp" != "" else []
PROJECT_CATKIN_DEPENDS = "geometry_msgs;nav_msgs;roscpp;std_msgs;tf".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ltag_route_graph;-ljsoncpp".split(';') if "-ltag_route_graph;-ljsoncpp" != "" else []
PROJECT_NAME = "tag_graph_nav"
PROJECT_SPACE_DIR = "/home/iimt/dtx_ws/install"
PROJECT_VERSION = "0.1.0"
