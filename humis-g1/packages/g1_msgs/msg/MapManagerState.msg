# g1_map_manager runtime state (latched on /map_manager/state)
uint8 MODE_IDLE=0
uint8 MODE_MAPPING=1
uint8 MODE_INCREMENTAL=2
uint8 MODE_LOCALIZATION=3
uint8 MODE_EDITING=4
std_msgs/Header header
string active_map        # "" when none
string current_label     # current geometry's lineage label; "" when none
uint8 mode
bool localized           # backend relocalization lock state
bool transition          # true while a cross-map switch is in progress
string detail            # human-readable status
