# htlライブラリ
なにか汎用性高そうなものをおいとく保管庫。
# Contents
## OpenCV
- OpenCVにて回転を表す回転行列、クォータニオン、ロドリゲス回転ベクトル、オイラー角を相互に変換する関数([transform.hpp](htl/opencv/transform.hpp))
- 三角測量の関数([triangulate.hpp](htl/opencv/triangulate.hpp))
## ROS
- ROS1/2での色情報をxacroにて扱いたい時用のxacroファイル([color.urdf.xacro](htl/ros/xacro/color.urdf.xacro))
- ROS2でのメッセージ用変換関数([msg_converter.hpp](htl/ros/msg_converter.hpp))
## std
- std::vectorでの平均値や分散などを計算する関数([vector.hpp](htl/std/vector.hpp))
