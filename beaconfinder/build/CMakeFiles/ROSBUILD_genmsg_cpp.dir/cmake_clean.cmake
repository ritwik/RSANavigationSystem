FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/beaconfinder/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/beaconfinder/Beacon.h"
  "../msg_gen/cpp/include/beaconfinder/Beacons.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
