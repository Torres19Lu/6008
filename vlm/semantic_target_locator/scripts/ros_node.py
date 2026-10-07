"""ROS1 entry point for the target-only semantic locator."""

from contracts import TargetQuery
from locator import LocatorConfig, SemanticTargetLocator
from ros_adapter import (
    AlignedDepthProvider, RosJsonResultSink, RosQuerySource,
    RosSynchronizedFrameSource, RosTfProvider, RosTopics,
    RosVisionLanguageBackend,
)


def main() -> None:
    import rospy

    rospy.init_node("semantic_target_locator")
    topics = RosTopics(
        rgb=rospy.get_param("~rgb_topic", RosTopics.rgb),
        depth=rospy.get_param("~depth_topic", RosTopics.depth),
        camera_info=rospy.get_param("~camera_info_topic", RosTopics.camera_info),
        query=rospy.get_param("~query_topic", RosTopics.query),
        status=rospy.get_param("~status_topic", RosTopics.status),
        map_frame=rospy.get_param("~map_frame", RosTopics.map_frame),
    )
    frames = RosSynchronizedFrameSource(
        topics,
        queue_size=int(rospy.get_param("~sync_queue_size", 5)),
        slop_s=float(rospy.get_param("~sync_slop_s", 0.05)),
    )
    sink = RosJsonResultSink(topics)
    locator = SemanticTargetLocator(
        frames=frames,
        vlm=RosVisionLanguageBackend(
            yolo_port=int(rospy.get_param("~yolo_port", 12184)),
            clip_port=int(rospy.get_param("~clip_port", 12182)),
        ),
        depth=AlignedDepthProvider(
            depth_scale_to_m=float(rospy.get_param("~depth_scale_to_m", 0.001)),
            min_depth_m=float(rospy.get_param("~min_depth_m", 0.2)),
            max_depth_m=float(rospy.get_param("~max_depth_m", 8.0)),
            pixel_stride=int(rospy.get_param("~depth_pixel_stride", 4)),
            min_points=int(rospy.get_param("~min_depth_points", 20)),
        ),
        transforms=RosTfProvider(
            topics, timeout_s=float(rospy.get_param("~tf_timeout_s", 0.2))
        ),
        sink=sink,
        config=LocatorConfig(
            max_frames=int(rospy.get_param("~max_frames", 120)),
            frame_timeout_s=float(rospy.get_param("~frame_timeout_s", 0.5)),
            detector_confidence=float(rospy.get_param("~detector_confidence", 0.25)),
            text_similarity=float(rospy.get_param("~text_similarity", 0.2)),
            min_observations=int(rospy.get_param("~min_observations", 3)),
            association_radius_m=float(rospy.get_param("~association_radius_m", 0.6)),
            max_position_spread_m=float(rospy.get_param("~max_position_spread_m", 0.25)),
        ),
    )
    queries = RosQuerySource(topics)
    rospy.loginfo("semantic_target_locator ready; waiting on %s", topics.query)
    while not rospy.is_shutdown():
        query = queries.next_query()
        if query is not None:
            locator.locate(TargetQuery(query.target_text, query.base_class))


if __name__ == "__main__":
    main()
