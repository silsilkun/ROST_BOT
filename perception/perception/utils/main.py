import realsense_viewer
import cam_event
import processed_data

# RealSense 카메라 실행 (스페이스바 누르면 save_cam 호출)
realsense_viewer.run(
    on_save=processed_data.save_cam,
    on_reset=cam_event.reset_points,
    on_click=cam_event.mouse_callback,
    update_depth_frame=cam_event.update_depth_frame,
    update_color_image=cam_event.update_color_image,
    get_points=cam_event.get_saved_points,
)
