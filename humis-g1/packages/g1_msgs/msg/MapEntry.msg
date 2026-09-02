string name
string label             # current geometry's lineage label
string created_at        # current geometry's lineage created_at
string description       # from overlays/metadata.yaml (when present)
float64[] extent         # [xmin, ymin, xmax, ymax]; empty when unknown
uint32 num_keyframes     # current geometry (backend manifest)
uint32 num_snapshots
