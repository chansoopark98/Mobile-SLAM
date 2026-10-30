# Module target. Created by WP0b; owned by WP2 from the contract freeze (architecture.md §8.1).
# The glob picks up the .cc files in this directory.
sr_add_module(sr_frame_frame_tracker
  DEPS sr_map sr_vendor_geometry)
