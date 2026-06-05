# point-cloud-preprocessing

차량 측위 (특히 NDT scan matching) 의 입력 점군을 정제하기 위한
ROS 2 (Humble) preprocessing 패키지 모음.

전처리는 측위 정확도와 직결됨에도 자주 간과되는 단계이므로,
재사용 가능한 단일 책임 노드들을 한 곳에 모아둔다.
앞으로 ground / dynamic object / range crop / voxel downsample / ROI
같은 단계의 패키지가 추가될 예정.

---

## 현재 포함된 패키지

### 1. `pointcloud_ground_filter_cpp` — RANSAC 기반 지면 제거

차량 측위에서 LiDAR 점군 중 지면 점들은 NDT 매칭에 거의 도움이 되지
않으며, 오히려 score landscape 의 cell density 를 부풀려 잘못된
local minimum 으로 빠지는 원인이 된다. 평탄한 도로 위에서 지면은
"어디서나 매칭" 되므로 차량의 x / y / yaw 에 대한 제약을 거의 주지
않기 때문이다.

본 패키지는 PCL 의 `pcl::SACSegmentation` (`SACMODEL_PERPENDICULAR_PLANE`)
을 사용해 매 스캔마다 지면 평면을 RANSAC 으로 찾아 inlier 들을
제거하고, **수직 구조물 (건물 벽 · 기둥 · 가로수 등) 만 남은 점군**
을 publish 한다. NDT 는 이 점군에 대해서만 정렬을 수행하므로
훨씬 강한 SE(2) 제약을 받는다.

#### 알고리즘 요지
- 모델: `SACMODEL_PERPENDICULAR_PLANE` — 평면의 법선이
  `axis = (0, 0, 1)` 기준 `eps_angle` 이내일 때만 후보로 채택.
  → 사실상 "거의 수평인 평면" 만 지면 후보.
- RANSAC: `max_iter` (기본 50) 회 반복, 거리 `distance_threshold`
  (기본 0.20 m) 이내를 inlier 로 카운트.
- 사전 z-band clip: `z_min_for_ransac ~ z_max_for_ransac` 안의
  점만 RANSAC 대상으로 → 멀리 있는 벽 / 차량 지붕이 RANSAC vote 를
  훔쳐가는 일을 방지. 발견된 평면은 *전체 점군* 에 대해 적용되어
  지면 점이 모두 제거됨.
- 평면 fit 실패 (`min_inliers` 미달) 시 입력을 그대로 통과.

#### 인터페이스

| 토픽 | 타입 | 방향 | 기본값 |
|---|---|---|---|
| `input_topic` | `sensor_msgs/PointCloud2` | sub | `/morai/lidar/points` |
| `output_topic` | `sensor_msgs/PointCloud2` | pub | `/lidar/no_ground` |
| `diagnostics_topic` | `std_msgs/Float64MultiArray` | pub | `/lidar/ground_filter/diagnostics` |

진단 메시지 `diagnostics_topic` 은 `[total_pts, kept_pts, kept_ratio,
plane_inliers, dt_ms]` 5 개 값을 publish 한다. `dt_ms` 는 해당
스캔의 필터 처리 시간으로, 실시간 예산 (보통 10 Hz 라면 100 ms)
안에 들어오는지 즉시 모니터링 가능하다.

#### 파라미터

| 파라미터 | 기본 | 의미 |
|---|---|---|
| `max_iter` | 50 | RANSAC 반복 횟수 |
| `distance_threshold` | 0.20 m | inlier 판정 거리 |
| `eps_angle_deg` | 30.0 | 지면 평면 법선이 +z 와 이룰 수 있는 최대 각도 |
| `min_inliers` | 500 | 이보다 적게 잡히면 통과 (지면 미발견) |
| `z_min_for_ransac` | -3.0 m | RANSAC 대상 점의 z 하한 |
| `z_max_for_ransac` | 0.5 m | RANSAC 대상 점의 z 상한 |

#### 사용 예

```bash
# 가장 단순한 standalone 실행 (원본 cloud 사용)
ros2 launch pointcloud_ground_filter_cpp ransac_ground_filter.launch.py

# fault 토픽 (다른 노드가 만든 /lidar/points_faulty) 을 입력으로
ros2 launch pointcloud_ground_filter_cpp ransac_ground_filter.launch.py use_faulty:=true

# 직접 input / output 지정
ros2 launch pointcloud_ground_filter_cpp ransac_ground_filter.launch.py \
    --ros-args -p input_topic:=/my/raw_cloud \
               -p output_topic:=/my/no_ground

# 처리 시간 + 제거 비율 확인
ros2 topic echo /lidar/ground_filter/diagnostics
# data: [total_pts, kept_pts, kept_ratio, plane_inliers, dt_ms]
# 정상값 예시: [120000, 80000, 0.67, 35000, 4.5]
```

#### 성능 — 왜 C++ 인가
초기 prototype 은 numpy + `sensor_msgs_py.point_cloud2` 를 사용한
Python 구현이었으나, **점군 ↔ ROS 메시지 변환 비용** 만으로
100k 점 기준 약 500 ms 가 소요되어 전체 파이프라인이 backlog 에
빠졌다 (scanmatcher / RViz / EKF 모두 message drop). C++ 버전은
`pcl::fromROSMsg` + PCL native segmentation 으로 동일 작업을 **3-8 ms**
에 완료한다 (10 Hz 예산의 5 % 이하). 즉 단순한 알고리즘 차이가
아니라 **PCL 직접 호출이 가능한 C++ 가 사실상 유일한 실시간 선택지**
였다.

---

## 빌드

```bash
mkdir -p ~/pcp_ws/src
cd ~/pcp_ws/src
git clone https://github.com/jeonseoknam/point-cloud-preprocessing.git
cd ~/pcp_ws
colcon build --symlink-install
source install/setup.bash
```

의존성: ROS 2 Humble, PCL 1.12+, `pcl_conversions`. 표준 Autoware /
nav2 환경이면 대부분 이미 설치되어 있다.

---

## 의존 / 활용 프로젝트

- [jeonseoknam/fault-tolerant-localization-pipeline](https://github.com/jeonseoknam/fault-tolerant-localization-pipeline)
  — `use_ground_filter:=true` 한 launch 인자로 본 패키지를 자동
  띄워 NDT scanmatcher 의 입력으로 사용한다.

---

## 라이선스

Apache-2.0. 자세한 내용은 [LICENSE](LICENSE) 참조.

각 패키지의 `package.xml` 에 명시된 라이선스가 우선한다.
