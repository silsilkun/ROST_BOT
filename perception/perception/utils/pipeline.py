# pipeline.py
import os
import numpy as np
import cv2

import click_points
from detector import DepthDBSCANVisualizer
import depth_utils
from coordinate import Coordinate
from settings import OUTPUT_DIR, COLOR_PATH, DEPTH_PATH

# detector 인스턴스 (기존과 동일)
detector = DepthDBSCANVisualizer()

color_image = None
depth_image = None
points_3d = []

processed_result = {
    "color": None,
    "depth": None,
    "points_3d": None,

    "vis": None,
    "boxes": None,
    "green_items": None,
    "world_list": None,
    "clicked_world_xy_list": None,
    "flat_clicked_xy": None,
    "flat_world_list": None,
}


def save_cam():
    """
    return:
      processed_result (dict)
      keys:
        color, depth, points_3d, vis, boxes, world_list,
        clicked_world_xy_list, flat_clicked_xy, flat_world_list
    """
    global color_image, depth_image, points_3d, processed_result

    # 1) 스냅샷 + 클릭포인트 3D 계산
    color, depth, points = click_points.Save_Cam()
    color_image = color
    depth_image = depth
    points_3d = points  # [(X,Y,Z), ...] (클릭 기반 world)

    # ✅ dict에 항상 기록 (실패해도 None으로 남고, 호출부에서 안전)
    processed_result["color"] = color
    processed_result["depth"] = depth
    processed_result["points_3d"] = points_3d

    if color is None or depth is None:
        print("save_cam: color/depth 없음")
        # ✅ 튜플 None 나열 대신 dict 그대로 반환
        return processed_result

    # 1-1) 클릭 world에서 XY만 뽑아 저장 + 평탄화
    clicked_world_xy_list = [[float(p[0]), float(p[1])] for p in points_3d]

<<<<<<< HEAD:perception/perception/utils/pipeline.py
=======
    flat_clicked_xy = clicked_world_xy_list

>>>>>>> feature/perception-승준:perception/perception/final/pipeline.py
    # 2) detect 실행 (ID 그려진 vis 생성)
    detector.update(color, depth)
    vis, items = detector.run()

    # 3) 저장 (ID가 포함된 이미지로 저장!)
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    cv2.imwrite(COLOR_PATH, vis)   # ✅ color 대신 vis
    np.save(DEPTH_PATH, depth)

    # 그리기용 boxes (poly)
    boxes = [it["poly"] for it in items]

    # 4) world_list 생성 (초록 + 파랑, id 유지)
    fake_depth = depth_utils.FakeDepthFrameFromNpy(depth)
    coord = Coordinate()

    world_list = []
    for it in items:
        obj_id = int(it["id"])
        obj_type = it["type"]

        Pw = None
        if obj_type == "green":
            cx, cy = depth_utils.box_center_pixel(it["poly"])
            Pw = coord.pixel_to_world(cx, cy, fake_depth)
            ang = float(it["angle"])
        else:
            # blue: 안전 탐색 + angle=0.0
            Pw = depth_utils.blue_rect_to_world_safe(
                it["rect"], depth, search_step=2
            )
            ang = 0.0

        # 실패해도 id 유지
        if Pw is None:
            X = Y = Z = 0.0
        else:
            X, Y, Z = map(float, Pw[:3])

        world_list.append({
            "id": obj_id,
            "type": obj_type,
            "world": (float(X), float(Y), float(Z)),
            "angle": float(ang),
        })

    # 4-1) flat_world_list (ID 순서 유지)
    flat_world_list = []
    for it in sorted(world_list, key=lambda d: d["id"]):
        X, Y, Z = it["world"]
        flat_world_list.extend([
            it["id"], X, Y, Z, float(it["angle"])
        ])

    # 5) 결과 저장 (dict)
    processed_result["vis"] = vis
    processed_result["boxes"] = boxes
    processed_result["world_list"] = world_list
    processed_result["clicked_world_xy_list"] = clicked_world_xy_list
    processed_result["flat_clicked_xy"] = flat_clicked_xy
    processed_result["flat_world_list"] = flat_world_list
    processed_result["green_items"] = None

    # 6) 결과 창 표시
    if boxes:
        cv2.imshow("Detect Result", vis)
        cv2.waitKey(1)

    # 유지 요구사항
    print("flat_world_list:", flat_world_list)
    print("flat_clicked_xy:", flat_clicked_xy)

    return processed_result
'''
result = save_cam()
color = result["color"]
flat_world_list = result["flat_world_list"]
'''