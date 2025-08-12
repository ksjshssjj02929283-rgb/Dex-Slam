include "test_mapping_hz.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
    max_submaps_to_keep = 3,
}
POSE_GRAPH.optimize_every_n_nodes = 80

return options