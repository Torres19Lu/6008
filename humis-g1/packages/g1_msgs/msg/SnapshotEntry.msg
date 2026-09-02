# One snapshot of a map (g1_map_manager).
string id            # e.g. "20260624-093000-pre-incremental"
string label
string reason        # initial | incremental | edit | imported | milestone
string parent        # snapshot id this geometry derived from, or ""
string created_at     # ISO-8601
uint32 num_keyframes  # read from the backend manifest (0 if unknown)
