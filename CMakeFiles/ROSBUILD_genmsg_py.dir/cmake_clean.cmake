FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/createNav/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/createNav/msg/__init__.py"
  "src/createNav/msg/_SensorPacket.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
