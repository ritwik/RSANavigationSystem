FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/beaconfinder/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/beaconfinder/msg/__init__.py"
  "../src/beaconfinder/msg/_Beacon.py"
  "../src/beaconfinder/msg/_Beacons.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
