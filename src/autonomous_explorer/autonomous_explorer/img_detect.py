import cv2
import numpy as np
  

def stereo_depth_estimation(focal_length, baseline, img_width,img_height,left_image_point, right_image_point):
    """
    计算双目相机测距，并返回物体的三维坐标。
    Args:
        focal_length (float): 相机的焦距 (单位：像素，或与基线单位保持一致).
        baseline (float): 双目相机光心之间的距离 (即基线，单位：米或毫米).
        left_image_point (tuple): 物体在左相机成像平面上的坐标 (u_L, v_L).
        right_image_point (tuple): 物体在右相机成像平面上的坐标 (u_R, v_R).

    Returns:
        tuple: 物体在相机坐标系下的三维坐标 (X, Y, Z)，单位与基线单位保持一致。
            如果视差为零，则返回 None。
    """
    u_L, v_L = left_image_point
    u_R, v_R = right_image_point

    # 计算视差
    disparity = u_L - u_R

    if disparity == 0:
        print("视差为零，无法计算深度")
        return 10000,10000,10000

    # 计算深度 Z
    Z = (focal_length * baseline) / disparity
    
    # 实际应用中，更精确的 X, Y 计算需要考虑相机的主点 (cx, cy)
    # X = (u_L - cx) * Z / focal_length
    # Y = (v_L - cy) * Z / focal_length
    
    X = ((u_L-img_width/4) * Z) / focal_length - baseline/2
    Y = ((v_L-img_height/2)  * Z) / focal_length

    return X, Y, Z

def detect_green_objects(image, min_area=300):
    """
    检测图像中的绿色物体
    
    参数:
        image: 输入的BGR格式图像
        min_area: 最小有效区域面积，用于过滤小噪点
    
    返回:
        result: 标注后的图像
        contours: 检测到的轮廓列表
    """
    # 转换到HSV色彩空间，更容易分离颜色
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # 定义绿色在HSV空间中的范围
    lower_green = np.array([40, 70, 70])   # H(色调): 40-80对应绿色范围
    upper_green = np.array([80, 255, 255]) # S(饱和度)和V(亮度)适当调整
    
    # 创建掩码，只保留绿色区域
    mask = cv2.inRange(hsv, lower_green, upper_green)
    
    # 执行形态学操作，消除小噪点并连接相邻区域
    kernel = np.ones((10, 10), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # 开运算：先腐蚀后膨胀
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel) # 闭运算：先膨胀后腐蚀
    
    # 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # 过滤小面积轮廓
    result = image.copy()
    valid_contours = []
    center_points = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area >= min_area:
            valid_contours.append(contour)
            
            # 计算边界框
            x, y, w, h = cv2.boundingRect(contour)
            # 计算轮廓中心
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                center_points.append((cX, cY))
    
    if center_points is not None:
        return result, center_points
    else:
        return None

# 主函数：处理视频流或单张图像
def process_video(focal_length, baseline, img_width,img_height, source=2):
    """
    主函数，支持视频流或单张图像处理
    
    参数:
        mode: 'video' 或 'image'
        source: 视频设备ID或图像路径
    返回值：
        [X,Y,Z]: 三维坐标列表
    """
    # 检测是否成功
    detect_success = False
    X = None
    Y = None
    Z = None


        # 打开视频捕获设备
    cap = cv2.VideoCapture(source)
    
    if not cap.isOpened():
        print("无法打开视频设备")
        return None
    print("成功打开camera设备")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, img_width)   # 宽度
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, img_height)  # 高度
    left_point = (0,0)
    right_point = (0,0)
    green_num = 10
    while green_num > 0:
        green_num -= 1
        ret, frame = cap.read()
        if not ret:
            print("无法获取帧")
            continue
        
        # 检测绿色物体
        _, center_point = detect_green_objects(frame)
        # 若检测到的点为两个点，则计算深度
        if len(center_point)%2 == 0 and center_point is not None:
            for point in center_point:
                # 如果是右目相机的点，则减去图像宽度的一半
                if point[0] > img_width/2:
                    right_point = (point[0]-img_width/2,point[1])
                else:
                    left_point = point
            
            X,Y,Z = stereo_depth_estimation(focal_length,baseline,img_width,img_height,left_point, right_point)
            # 如果视差不为零，则认为检测成功
            if (X,Y,Z) != (10000,10000,10000):
                detect_success = True
                print("X: %f, Y: %f, Z: %f" % (X, Y, Z))
                break
            else:
                print("检测到视差为0,重新检测")
        else:
            print("仅单个相机检测到物体")

    cap.release()

    if detect_success:
        return X, Y, Z
    else:
        return None
    

