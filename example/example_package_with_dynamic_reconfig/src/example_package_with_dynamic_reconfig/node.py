#! /usr/bin/env python3
from typing import Any, Dict
import rospy
import dynamic_reconfigure.server
from example_package_with_dynamic_reconfig.cfg import ExampleDynamicParametersConfig


def dynamic_reconfigure_callback(config: Dict[str, Any], level: Any) -> Dict[str, Any]:
    return config


if __name__ == "__main__":

    try:
        rospy.init_node("package_with_dynamic_reconfig", log_level=rospy.WARN)
        dynamic_reconfigure_srv = dynamic_reconfigure.server.Server(ExampleDynamicParametersConfig,
                                                                    dynamic_reconfigure_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down.")
