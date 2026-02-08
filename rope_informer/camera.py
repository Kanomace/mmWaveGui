import cv2

def record_video(output_file, duration, camera_index=0):
    # 打开相机
    cap = cv2.VideoCapture(camera_index)

    # 获取相机的宽度和高度
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # 创建 VideoWriter 对象，用于写入视频
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(output_file, fourcc, 20.0, (width, height))

    # 录制指定时长的视频
    start_time = cv2.getTickCount()
    while (cv2.getTickCount() - start_time) / cv2.getTickFrequency() < duration:
        ret, frame = cap.read()  # 读取一帧图像
        if ret:
            out.write(frame)  # 写入当前帧到视频文件

            # 显示当前帧
            cv2.imshow('Recording', frame)
            if cv2.waitKey(1) == ord('q'):  # 按下 'q' 键退出录制
                break
        else:
            break

    # 释放资源
    cap.release()
    out.release()
    cv2.destroyAllWindows()

# 测试录制视频
output_file = 'output.avi'  # 输出文件名
duration = 10  # 录制时长（秒）

record_video(output_file, duration)