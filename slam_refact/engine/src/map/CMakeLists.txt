# Module target. Created by WP0b; owned by WP2 from the contract freeze (architecture.md §8.1).
# The glob picks up the .cc files in this directory.
sr_add_module(sr_map
  DEPS sr_common sr_compat sr_config sr_vendor_features sr_vendor_geometry)
