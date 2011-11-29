FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/gradient_map/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/GradientMap.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_GradientMap.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
