# UR Drawing

`ur_drawing` là một package bài tập độc lập dành cho ROS 2 Humble, cho
phép robot UR3 hoặc UR3e mô phỏng vẽ các chữ cái tiếng Anh được định
nghĩa thủ công và một đường tròn trong hệ tọa độ Descartes. 

## Kiến trúc

``` text
ur_drawing launch

  -> launch tương thích cục bộ cho ur_simulation_gz (Gazebo Sim, robot, controllers)

  -> launch chính thức của ur_moveit_config (move_group và RViz tùy chọn)

  -> letter_drawer hoặc circle_drawer

  -> service compute_cartesian_path của MoveIt và action execute_trajectory

  -> joint_trajectory_controller trong Gazebo
```


## Robot và quy ước hệ tọa độ Descartes

-   Robot mặc định: `ur3e`; sử dụng `ur_type:=ur3` để dùng UR3.
-   Planning group: `ur_drawing`.
-   Hệ quy chiếu: `base_link`.
-   Link đầu công tác (end-effector): `tool0`.
-   Mặt phẳng vẽ: mặt phẳng XY tại độ cao `draw_height`, đơn vị mét;
    trục Z được nâng lên tới `lift_height` khi di chuyển giữa các nét
    vẽ.
-   Hướng cố định mặc định của tool là quaternion `(0, 1, 0, 0)`, làm
    cho trục Z của tool hướng xuống mặt phẳng vẽ.

Các chữ cái được định nghĩa thủ công bằng các polyline trong hệ tọa độ
chuẩn hóa `[-0.5, 0.5]`. Tham số `scale` chuyển các tọa độ này sang mét
và cộng thêm `center_x`, `center_y`. Package có đầy đủ các ký tự A--Z;
chữ thường đầu vào sẽ được chuyển thành chữ hoa.

Đường tròn được tạo bằng cách lấy mẫu `num_points + 1` điểm theo công
thức `x=r*cos(theta)`, `y=r*sin(theta)`, trong đó có cả điểm cuối cùng
để khép kín đường tròn.

## Các dependency và quá trình build

Workspace phải có sẵn ROS 2 Humble, MoveIt 2 và các package chính thức
`ur_description`, `ur_moveit_config` và `ur_simulation_gz`.

### Cài đặt 

Để chạy package này từ source, hãy đặt các repository Universal Robots tương thích với Humble sau đây cùng với `ur_drawing` trong thư mục `src/` của workspace:

-   `Universal_Robots_ROS2_Description`: cung cấp `ur_description` (mô
    hình UR3/UR3e, mesh, thông số động học và giới hạn khớp).
    ```bash
    git clone -b $(echo $ROS_DISTRO) https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
    ```
-   `Universal_Robots_ROS2_Driver`: cung cấp `ur_moveit_config` và
    `ur_controllers` (cấu hình MoveIt và các định nghĩa controller).
    Node `ur_robot_driver` dành cho robot thật không được package mô
    phỏng này khởi chạy.
    ```bash
    git clone -b $(echo $ROS_DISTRO) https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
    ```
-   `Universal_Robots_ROS2_GZ_Simulation`: cung cấp `ur_simulation_gz`
    (cấu hình controller cho Gazebo Sim được sử dụng trong project).
    ```bash
    git clone -b $(echo $ROS_DISTRO) https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git
    ```

Các package ROS còn lại, bao gồm MoveIt 2, tích hợp Gazebo Sim
(`ros_gz_sim`, `ros_gz_bridge` và `ign_ros2_control`), RViz và Xacro,
nên được cài đặt dựa trên các dependency của workspace:

``` bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash

rosdep install --ignore-src --from-paths src -r -y

colcon build --symlink-install

source install/setup.bash
```

`ur_drawing` hỗ trợ `ur3e` theo mặc định và hỗ trợ `ur3` thông qua
`ur_type:=ur3`.

``` bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash

colcon build --packages-select ur_drawing --symlink-install

source install/setup.bash
```

## Khởi chạy

- Terminal 1: Chạy launch sim gazebo + rviz

``` bash
ros2 launch ur_drawing sim_moveit.launch.py
```

- Terminal 2: Chạy launch vẽ chữ hoặc vẽ đường tròn
```bash
ros2 launch ur_drawing draw_letter.launch.py letter:=A

ros2 launch ur_drawing draw_letter.launch.py letter:=m

ros2 launch ur_drawing draw_circle.launch.py
```
Một số tham số hữu ích khi vẽ:

``` bash
ros2 launch ur_drawing draw_letter.launch.py letter:=B scale:=0.10 draw_height:=0.30 lift_height:=0.36 frame_id:=base_link

ros2 launch ur_drawing draw_circle.launch.py radius:=0.06 center_x:=0.30 center_y:=0.0 draw_height:=0.30 num_points:=72
```

`sim_moveit.launch.py` chấp nhận `ur_type` (`ur3` hoặc `ur3e`),
`launch_rviz`, `gazebo_gui`, `world_file` và các tham số safety/prefix
chính thức.

## Lập kế hoạch, thực thi và trực quan hóa

Mỗi nét vẽ được thực thi theo các giai đoạn: nâng tool/di chuyển, hạ
tool, vẽ và sau đó nâng tool.

Node yêu cầu MoveIt lập kế hoạch cho từng đoạn với tính năng tránh va
chạm được bật, tham số `eef_step` có thể cấu hình (mặc định 0.008 m),
cùng hệ số giới hạn vận tốc và gia tốc ở mức thận trọng (0.15).

Node sẽ từ chối trajectory nếu MoveIt trả về trạng thái không thành công
hoặc nếu tỷ lệ Cartesian nhỏ hơn `required_fraction` (mặc định 1.0). Vì
vậy, trajectory không hoàn chỉnh sẽ không được thực thi.

Cấu hình RViz cục bộ tự động bao gồm phần hiển thị `drawing_markers`
kiểu MarkerArray. Các đường màu cyan biểu diễn những nét đã được lập kế
hoạch; nét hiện đang được thực thi được hiển thị bằng màu cam.

