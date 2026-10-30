# Engine core + glue (architecture.md §3.9). Created by WP0b; owned by WP5 from the contract
# freeze (architecture.md §8.1). Add .cc files to this directory; the glob picks them up.
sr_add_module(sr_engine
  DEPS sr_vio_tracker sr_image_target)
add_library(slam_refact::engine ALIAS sr_engine)
