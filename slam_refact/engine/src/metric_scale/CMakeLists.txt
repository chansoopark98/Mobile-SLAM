# Module target. Created by WP0b; owned by WP4 from the contract freeze (architecture.md §8.1).
# The glob picks up the .cc files in this directory.
sr_add_module(sr_metric_scale
  DEPS sr_common sr_config Eigen3::Eigen)
