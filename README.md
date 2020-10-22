# htl
なにか汎用性高そうなものをおいとく保管庫。
# Contents
## OpenCV
- OpenCVにて回転に関する変換を行う関数([transform.hpp](opencv/transform.hpp))(ex:回転行列、クォータニオン、ロドリゲス回転ベクトル、オイラー角を相互に変換する関数)
- 三角測量の関数([triangulate.hpp](opencv/triangulate.hpp))
## ROS
- ROSでの色情報をxacroにて扱いたい時用のxacroファイル([color.urdf.xacro](ros/xacro/color.urdf.xacro))
- ROS2でのメッセージ用変換関数([msg_converter.hpp](ros/msg_converter.hpp))
## std
- std::vectorでの平均値や分散などを計算する関数([vector.hpp](std/vector.hpp))
