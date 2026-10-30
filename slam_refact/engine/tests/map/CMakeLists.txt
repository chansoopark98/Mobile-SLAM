# WP2 world map tests (architecture.md §7.3 "map").
sr_add_unit_test(sr_map_world_map_test
  SOURCES world_map_test.cc map_test_util.h
  LIBS sr_map)

sr_add_unit_test(sr_map_visibility_test
  SOURCES visibility_test.cc map_test_util.h
  LIBS sr_map)
