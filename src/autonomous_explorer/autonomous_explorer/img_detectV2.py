import cv2
import numpy as np
  


def detect_green_objects(image, img_width,img_height,min_area=300):
    """
    检测图像中的绿色物体
    
    参数:
        image: 输入的BGR格式图像
        min_area: 最小有效区域面积，用于过滤小噪点
    
    返回:
        result: 标注后的图像
        contours: 检测到的轮廓列表
    """
    # 记录左右目图像中的绿色物体位置，x_l,w_l,x_r,w_r
    disparity = [img_width//2,img_width//2,-1,-1]
    hsv = []
    detected = False

    # 转换到HSV色彩空间，更容易分离颜色
    image_left = image[0:img_height, 0:img_width//2]
    image_right = image[0:img_height, img_width//2:img_width]
    hsv.append(cv2.cvtColor(image_left, cv2.COLOR_BGR2HSV))
    hsv.append(cv2.cvtColor(image_right, cv2.COLOR_BGR2HSV))
    # 定义绿色在HSV空间中的范围
    lower_green = np.array([40, 80, 70])   # H(色调): 40-80对应绿色范围
    upper_green = np.array([80, 255, 255]) # S(饱和度)和V(亮度)适当调整
    
    # 创建掩码，只保留绿色区域
    for i in range(2):
        mask = cv2.inRange(hsv[i], lower_green, upper_green)
    
    # 执行形态学操作，消除小噪点并连接相邻区域
        kernel = np.ones((20, 20), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # 开运算：先腐蚀后膨胀
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel) # 闭运算：先膨胀后腐蚀
        
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        

        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= min_area:
                detected = True

                # 计算边界框
                x, _, w, _ = cv2.boundingRect(contour)
                disparity[2*i] = x
                disparity[2*i+1] = x+w
                
    return detected,disparity

    

    