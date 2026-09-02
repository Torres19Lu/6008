# Quality of a relocalization attempt (diagnostic fields are filled even on failure).
float64 inlier_ratio    # fraction of source points within inlier_dist (the accept gate)
float64 fitness         # ICP fitness score (mean sq correspondence distance)
float64 sc_distance     # Scan-Context distance of the chosen candidate
uint64  match_id        # matched keyframe id (global acquire)
bool    accepted        # true iff the attempt passed the inlier gate
