"""ROS1 entry point for resource-limited fixed target localization."""

from contracts import TargetQuery
from lite import LiteVisionLanguageBackend, RateLimitedFrameSource
from locator import LocatorConfig, SemanticTargetLocator
from ros_adapter import (
    AlignedDepthProvider, RosJsonResultSink, RosSynchronizedFrameSource,
    RosTfProvider, RosTopics,
)
from ros_lite_adapter import RosLiteQuerySource, YoloV7ObjectDetector


def main() -> None:
    import rospy

    rospy.init_node("semantic_target_locator_lite")
    topics = RosTopics(
        rgb=rospy.get_param("~rgb_topic", RosTopics.rgb),
        depth=rospy.get_param("~depth_topic", RosTopics.depth),
        camera_info=rospy.get_param("~camera_info_topic", RosTopics.camera_info),
        query=rospy.get_param("~query_topic", RosTopics.query),
        status=rospy.get_param("~status_topic", RosTopics.status),
        map_frame=rospy.get_param("~map_frame", RosTopics.map_frame),
    )
    synchronized_frames = RosSynchronizedFrameSource(
        topics,
        queue_size=int(rospy.get_param("~sync_queue_size", 3)),
        slop_s=float(rospy.get_param("~sync_slop_s", 0.05)),
    )
    frames = RateLimitedFrameSource(
        synchronized_frames,
        rate_hz=float(rospy.get_param("~inference_rate_hz", 2.0)),
    )
    sink = RosJsonResultSink(topics)
    locator = SemanticTargetLocator(
        frames=frames,
        vlm=LiteVisionLanguageBackend(YoloV7ObjectDetector(
            port=int(rospy.get_param("~yolo_port", 12184))
        )),
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
            max_frames=int(rospy.get_param("~max_frames", 60)),
            frame_timeout_s=float(rospy.get_param("~frame_timeout_s", 0.75)),
            detector_confidence=float(rospy.get_param("~detector_confidence", 0.25)),
            # In lite mode this field is the minimum matching-color pixel fraction.
            text_similarity=float(rospy.get_param("~color_score_threshold", 0.03)),
            min_observations=int(rospy.get_param("~min_observations", 3)),
            association_radius_m=float(rospy.get_param("~association_radius_m", 0.6)),
            max_position_spread_m=float(rospy.get_param("~max_position_spread_m", 0.25)),
        ),
    )
    queries = RosLiteQuerySource(topics.query)
    rospy.loginfo(
        "semantic_target_locator_lite ready; YOLO + HSV color, %.2f Hz",
        1.0 / frames.min_interval_s,
    )
    while not rospy.is_shutdown():
        query = queries.next_query()
        if query is not None:
            locator.locate(TargetQuery(query.target_text, query.base_class))


if __name__ == "__main__":
    main()
