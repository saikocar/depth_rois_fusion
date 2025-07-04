# depth\_rois\_fusion

## 概要

`depth_rois_fusion` は、カメラ画像、LiDAR点群、および ROI（Region of Interest）情報を用いて、物体の3次元位置と形状を推定し、視覚的およびメッセージ形式で出力する ROS 2 ノードを提供します。

このノードは以下の機能を提供します：

* LiDAR点群をカメラフレームに変換し、depth画像を生成
* depth画像に ROI を重ねて対象物の距離を推定
* 物体の位置・寸法を算出し、`DetectedObjectsWithFeature` としてパブリッシュ
* RViz 用の Marker による可視化

## ノード名

```
depth_image_generator
```

## 対応トピック

### サブスクライブ（入力）

* `/sensing/lidar/concatenated/pointcloud` (`sensor_msgs/msg/PointCloud2`): LiDAR 点群
* `/sensing/camera/roscube/front_wide/camera_info` (`sensor_msgs/msg/CameraInfo`): カメラ内部パラメータ
* `/sensing/camera/roscube/front_wide/image_rect` (`sensor_msgs/msg/Image`): 正規化カメラ画像
* `/perception/object_recognition/detection/rois0` (`tier4_perception_msgs/msg/DetectedObjectsWithFeature`): 検出された物体の ROI 情報

### パブリッシュ（出力）

* `/camera/depth_image` (`sensor_msgs/msg/Image`): 生成された depth 画像（32FC1）
* `/camera/depth_rois_image` (`sensor_msgs/msg/Image`): ROI 付きのデバッグ画像（可視化用）
* `/camera/camera_info` (`sensor_msgs/msg/CameraInfo`): 入力カメラ情報の再出力
* `/camera/rois_depth` (`tier4_perception_msgs/msg/DetectedObjectsWithFeature`): depth 付加済み物体情報
* `/camera/rois_depth_marker` (`visualization_msgs/msg/MarkerArray`): RViz 用の可視化マーカー

## 主な処理内容

1. **Depth画像生成**：

   * LiDAR点群をカメラ座標系に変換し、各画素に最小depthを割り当てる形式で depth 画像（32FC1）を生成。

2. **ROIとの重畳処理**：

   * 指定された ROI 領域内の depth 値を取得し、最小 depth を算出。
   * カメラモデルに基づいて3D座標を復元。

3. **物体情報の更新**：

   * `DetectedObject` の `pose` と `shape` フィールドを depth に基づき更新。
   * 結果を `DetectedObjectsWithFeature` として出力。

4. **デバッグ用可視化**：

   * ROI 矩形と depth を記載したラベルを画像に描画。
   * `MarkerArray` として直方体を RViz に出力。

## パラメータ一覧（デフォルト値）

以下は `config/config.yaml` で設定される代表的なパラメータです：

```yaml
pointcloud_topic: /sensing/lidar/concatenated/pointcloud
camera_info_topic: /sensing/camera/roscube/front_wide/camera_info
image_topic: /sensing/camera/roscube/front_wide/image_rect
rois_topic: /perception/object_recognition/detection/rois0
depth_image_topic: /camera/depth_image
depth_rois_image_topic: /camera/depth_rois_image
out_camera_info_topic: /camera/camera_info
rois_depth_msg_topic: /camera/rois_depth
depth_rois_marker_topic: /camera/rois_depth_marker
rectangle_color: [0, 255, 0]
text_color: [255, 0, 0]
```

## ビルド方法

```bash
colcon build --packages-select depth_rois_fusion
```

必要に応じて `--symlink-install` や `--cmake-args -DCMAKE_BUILD_TYPE=Release` を指定してください。

## 起動方法

以下のような launch ファイルでノードを起動できます：

```bash
ros2 launch depth_rois_fusion depth_rois_fusion.launch.py
```

`depth_rois_fusion.launch.py` 内で `config/config.yaml` を読み込む設定がされているため、明示的にファイルパスを渡す必要はありません。

## 備考

* TF変換のタイムスタンプ誤差が0.2秒以上の場合は処理をスキップします。
* ROI内のdepth値が存在しない場合はスキップされます。
* 寸法はカメラモデルの焦点距離に基づいて計算されます（例：物体の幅\[m] = ROI幅\[px] × depth / fx）。
* 検出された物体は、1平面がカメラ平面に正対する直方体として可視化されますが、これは物体そのものの厳密な形状ではなく、干渉リスクの高い領域とみなしてください。

---

本ノードは、カメラ画像と LiDAR を組み合わせて高精度な物体位置推定を実現するための基盤コンポーネントとして設計されています。
