import cv2
import threading
import os
from datetime import datetime
import time

class Camera:
    def __init__(self, camera_index=0):
        self.video_capture = cv2.VideoCapture(camera_index)
        if not self.video_capture.isOpened():
            raise IOError(f"无法打开摄像头 (索引: {camera_index})")
        self.frame_lock = threading.Lock()
        self.current_frame = None
        self.is_running = True
        self.camera_thread = threading.Thread(target=self._update_frame, daemon=True)
        self.camera_thread.start()
        print("摄像头处理线程已启动。")

    def _update_frame(self):
        while self.is_running:
            ret, frame = self.video_capture.read()
            if ret:
                with self.frame_lock:
                    self.current_frame = frame.copy()
            else:
                # 如果读取失败，稍等一下再试，避免CPU空转
                time.sleep(0.1)
            time.sleep(1/30) # 控制帧率

    def get_jpeg_frame(self):
        """获取用于视频流的JPEG编码帧"""
        with self.frame_lock:
            if self.current_frame is None:
                return None
            ret, jpeg = cv2.imencode('.jpg', self.current_frame)
            return jpeg.tobytes() if ret else None

    def capture_image_to_file(self, upload_folder, session_id):
        """
        捕获当前帧并保存到指定路径。
        返回web可访问路径和服务器文件系统路径。
        """
        with self.frame_lock:
            if self.current_frame is None:
                return None, None
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"capture_{timestamp}.jpg"
            filepath = os.path.join(upload_folder, filename)
            
            # 确保目录存在
            os.makedirs(upload_folder, exist_ok=True)
            
            cv2.imwrite(filepath, self.current_frame)
            
            # 构建前端可访问的路径
            web_path = os.path.join('static', 'sessions', session_id, 'uploads', filename).replace('\\', '/')
            return web_path, filepath

    def release(self):
        print("正在释放摄像头资源...")
        self.is_running = False
        if self.camera_thread.is_alive():
            self.camera_thread.join(timeout=2)
        if self.video_capture.isOpened():
            self.video_capture.release()
        print("摄像头已释放。")
