FILE(REMOVE_RECURSE
  "../src/explore2/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/explore2/msg/__init__.py"
  "../src/explore2/msg/_ExploreGoal.py"
  "../src/explore2/msg/_ExploreState.py"
  "../src/explore2/msg/_ExploreFeedback.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
