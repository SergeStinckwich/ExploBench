FILE(REMOVE_RECURSE
  "../src/explore2/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/explore2/ExploreGoal.h"
  "../msg_gen/cpp/include/explore2/ExploreState.h"
  "../msg_gen/cpp/include/explore2/ExploreFeedback.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
