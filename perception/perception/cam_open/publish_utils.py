import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()  # CvBridge 전역 생성

def publish_points_and_depth(picker,depth_frame,color_image, get_saved_points, point_pub,color_pub,depth_pub, node):

    """
    클릭한 2D 픽셀 → 3D world 좌표 변환 후 발행
    Depth와 Color 이미지도 동시에 발행
    """
    if depth_frame is None or color_image is None:
        print("Warning: depth_frame 또는 color_image가 None입니다")
        return

    # 1. 클릭 포인트 → 3D world 좌표 변환 후 발행
    world_points = []
    saved_points = get_saved_points()

    for i, (x, y, _) in enumerate(saved_points, 1):
        try:
            Pw = picker.pixel_to_world(x, y, depth_frame)
            if Pw is not None and len(Pw) == 3:
                world_points.append([float(Pw[0]), float(Pw[1]), float(Pw[2])])
            else:
                print(f"Point {i} 변환 실패 (x={x:.1f}, y={y:.1f})")
        except Exception as e:
            print(f"Point {i} 변환 중 오류: {e}")

    if world_points:
        flat_points = [v for p in world_points for v in p]
        point_pub.publish(Float32MultiArray(data=flat_points))
        print(f"3D 포인트 {len(world_points)}개 발행 완료 → {world_points}")
    else:
        print("발행할 3D 포인트가 없습니다")

    # 2. Depth 이미지 발행
    try:
        depth_msg = bridge.cv2_to_imgmsg(
            np.asanyarray(depth_frame.get_data()),
            encoding="16UC1",
            header=depth_frame.get_header() if hasattr(depth_frame, 'get_header') else None
        )
        if depth_msg.header.stamp.sec == 0:
            depth_msg.header.stamp = node.get_clock().now().to_msg()
        depth_msg.header.frame_id = "camera_depth_optical_frame"
        depth_pub.publish(depth_msg)
        print("Depth 이미지 발행 완료")
    except Exception as e:
        print(f"Depth 퍼블리시 실패: {e}")

    # 3. Color 이미지 발행
    try:
        color_msg = bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        color_msg.header.stamp = node.get_clock().now().to_msg()
        color_msg.header.frame_id = "camera_color_optical_frame"
        color_pub.publish(color_msg)
        print("Color 이미지 발행 완료")
    except Exception as e:
        print(f"Color 퍼블리시 실패: {e}")
