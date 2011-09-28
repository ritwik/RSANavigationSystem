FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/beaconfinder/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/Beacon.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Beacon.lisp"
  "../msg_gen/lisp/Beacons.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Beacons.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
