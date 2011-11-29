FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/gradient_map/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/gradient_map/msg/__init__.py"
  "../src/gradient_map/msg/_GradientMap.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
