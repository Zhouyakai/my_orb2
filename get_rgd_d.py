#运行方式python3 get_rgd_d.py /home/yakai/SLAM/Date/D435/test_data/ 5
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import sys
import os
import shutil

# 创建RealSense管道
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# 开始RealSense管道
pipeline.start(config)

# 设置保存图像的目录
output_dir = sys.argv[1]

# 创建或删除文件夹
def create_or_delete_folders():
	folder_list = ["depth_raw", "yolov5_rgb", "depth"]

	for folder in folder_list:
		folder_path = os.path.join(output_dir, folder)
		if os.path.exists(folder_path):
			shutil.rmtree(folder_path)  # 删除文件夹及其内容
		os.makedirs(folder_path)  # 创建空文件夹
        
	# 创建或重新创建associations.txt文件
	associations_txt_path = os.path.join(sys.argv[1], "associations.txt")
	with open(associations_txt_path, "w") as associations_txt:
		pass

try:
	# 创建或删除文件夹
	create_or_delete_folders()

	start_time = time.time()  # 记录程序启动时间
	save_images = False  # 用于判断是否开始进行图像保存

	while True:
		# 等待新的帧
		frames = pipeline.wait_for_frames()

		# 获取深度和RGB图像帧
		depth_frame = frames.get_depth_frame()
		color_frame = frames.get_color_frame()

		if not depth_frame or not color_frame:
			continue

		# 将帧转换为Numpy数组
		depth_image = np.asanyarray(depth_frame.get_data())
		color_image = np.asanyarray(color_frame.get_data())

		# 将深度图像转换为灰度
		depth_image_gray = cv2.cvtColor(depth_image, cv2.COLOR_GRAY2BGR)

		# 获取当前时间戳
		current_time = time.time()

		# 检查是否超过10秒，如果是，则将save_images设置为True
		if not save_images and current_time - start_time > int(sys.argv[2]):
			save_images = True
		
		if save_images:
			# 获取当前时间戳
			timestamp = str(time.time())[0:17].ljust(17, '0')
			
			# 生成associations.txt文件
			associations_txt_path = os.path.join(output_dir, "associations.txt")
			with open(associations_txt_path, "a") as associations_txt:
				associations_txt.write(timestamp + ' ' + "yolov5_rgb/" + timestamp + ".png" + ' ' + timestamp + ' ' + "depth/" + timestamp + ".png" + '\n')

			# 保存图像
			depth_output = output_dir + "/depth_raw/" + timestamp + ".png"
			color_output = output_dir + "/yolov5_rgb/" + timestamp + ".png"
			depth_gray_output = output_dir + "/depth/" + timestamp + ".png"
			cv2.imwrite(depth_output, depth_image)
			cv2.imwrite(color_output, color_image)
			cv2.imwrite(depth_gray_output, depth_image_gray)
			
			# 显示RGB和深度图像
			cv2.imshow('Color Image', color_image)
			cv2.imshow('Depth Image', depth_image)

			# 按下Esc键退出循环
			if cv2.waitKey(1) == 27:
			    break

			# 打印保存的文件路径
			print("图像保存完成：Depth - " + depth_output + ", Color - " + color_output + ", Depth (gray) - " + depth_gray_output)
		else:
			# 计算剩余的秒数
			remaining_seconds = int(int(sys.argv[2]) + 1 - (current_time - start_time))
			print(f"还剩 {remaining_seconds} 秒数据集开始保存")
			time.sleep(1)

except KeyboardInterrupt:
    # 强制退出时停止管道
    pipeline.stop()

# 关闭图像窗口
cv2.destroyAllWindows()










