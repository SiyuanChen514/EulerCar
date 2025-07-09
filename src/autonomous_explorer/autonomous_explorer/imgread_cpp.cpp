#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <thread>      // 添加这个头文件用于线程操作
#include <chrono> 

using namespace cv;
using namespace std;

tuple<double, double, double> stereo_depth_estimation(double focal_length, double baseline, 
                                                     int img_width, int img_height,
                                                     Point2f left_image_point, Point2f right_image_point) {
    double u_L = left_image_point.x;
    double v_L = left_image_point.y;
    double u_R = right_image_point.x;
    double v_R = right_image_point.y;

    // 计算视差
    double disparity = u_L - u_R;

    if (disparity == 0) {
        printf("视差为零，无法计算深度");
        cout << "视差为零，无法计算深度" << endl;
        return make_tuple(10000.0, 10000.0, 10000.0);
    }

    // 计算深度 Z
    double Z = (focal_length * baseline) / disparity;
    
    double X = ((u_L - img_width / 4) * Z) / focal_length - baseline / 2;
    double Y = ((v_L - img_height / 2) * Z) / focal_length;

    return make_tuple(X, Y, Z);
}


void detect_green_objects(Mat& image, int img_width, int min_area, 
                          Mat& result, vector<Point2f>& center_points, bool& green_left) {
    result = image.clone();
    green_left = true;
    bool green_first = false;

    // 转换到HSV色彩空间
    Mat hsv;
    cvtColor(image, hsv, COLOR_BGR2HSV);
    
    // 定义绿色在HSV空间中的范围
    Scalar lower_green(40, 70, 70);
    Scalar upper_green(80, 255, 255);
    
    // 创建掩码
    Mat mask;
    inRange(hsv, lower_green, upper_green, mask);
    
    // 执行形态学操作
    Mat kernel = getStructuringElement(MORPH_RECT, Size(10, 10));
    morphologyEx(mask, mask, MORPH_OPEN, kernel);
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);
    
    // 查找轮廓
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    // 过滤小面积轮廓
    for (auto& contour : contours) {
        double area = contourArea(contour);
        if (area >= min_area) {
            Rect rect = boundingRect(contour);
            rectangle(result, rect, Scalar(0, 255, 0), 2);
            
            // 判断物块在左侧相机中是否完全显示
            if (rect.x < img_width / 2 && rect.x + rect.width > img_width / 2 - 5) {
                green_left = false;
                green_first = true;
            }
            if (green_first) {
                green_first = false;
                green_left = false;
            }
            
            if (green_left) {
                Point2f center(rect.x + rect.width, rect.y + rect.height / 2);
                center_points.push_back(center);
                circle(result, Point(rect.x + rect.width, rect.y), 5, Scalar(255, 0, 0), -1);
                putText(result, to_string(rect.x + rect.width) + ", " + to_string(rect.y), 
                        Point(rect.x, rect.y - 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
            } else {
                Point2f center(rect.x, rect.y);
                center_points.push_back(center);
                circle(result, Point(rect.x, rect.y), 5, Scalar(255, 0, 0), -1);
                putText(result, to_string(rect.x) + ", " + to_string(rect.y), 
                        Point(rect.x, rect.y - 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
            }
        }
    }
}


vector<double> process_video(double focal_length, double baseline, int img_width, int img_height, int source) {
    bool detect_success = false;
    double X = 0, Y = 0, Z = 0;

    VideoCapture cap(source);
    if (!cap.isOpened()) {
        printf("无法打开视频设备");
        cerr << "无法打开视频设备" << endl;
        return {};
    }

    cap.set(CAP_PROP_FRAME_WIDTH, img_width);
    cap.set(CAP_PROP_FRAME_HEIGHT, img_height);

    Point2f left_point(0, 0);
    Point2f right_point(0, 0);
    int green_num = 10;

    while (green_num > 0) {
        green_num--;
        Mat frame;
        if (!cap.read(frame)) {
            printf("无法获取帧");
            cerr << "无法获取帧" << endl;
            continue;
        }
        
        Mat result;
        vector<Point2f> center_points;
        bool green_left;
        
        detect_green_objects(frame, img_width, 300, result, center_points, green_left);
        
        if (center_points.size() % 2 == 0 && !center_points.empty()) {
            for (auto& point : center_points) {
                if (point.x > img_width / 2) {
                    right_point = Point2f(point.x - img_width / 2, point.y);
                } else {
                    left_point = point;
                }
            }
            
            tie(X, Y, Z) = stereo_depth_estimation(focal_length, baseline, img_width, img_height, left_point, right_point);
            
            if (X != 10000 && Y != 10000 && Z != 10000) {
                detect_success = true;
                if (green_left) {
                    X -= 0.019;
                } else {
                    X += 0.019;
                }
                cout << "物体坐标  X: " << X << ", Y: " << Z << ", Z: " << Y << endl;
                break;
            } else {
                cout << "检测到视差为0,重新检测" << endl;
            }
        } else {
            cout << "仅单个相机检测到物体或未检测到物体" << endl;
        }
    }

    cap.release();

    if (detect_success) {
        return {X, Z, Y};
    } else {
        return {};
    }
}

// 主函数示例
int main() {
    // 示例参数，实际应用中需要根据相机校准结果设置
    double focal_length = 800.0;  // 焦距，单位像素
    double baseline = 0.1;        // 基线距离，单位米
    int img_width = 1280;         // 图像宽度
    int img_height = 720;         // 图像高度
    int video_source = 0;         // 视频源，0通常是默认摄像头
    
    // 使用ESC键(ASCII码27)或q键退出循环
    while (true) {
        vector<double> coordinates = process_video(focal_length, baseline, img_width, img_height, video_source);
        
        if (!coordinates.empty()) {
            cout << "最终物体坐标: X=" << coordinates[0] 
                 << ", Y=" << coordinates[1] 
                 << ", Z=" << coordinates[2] << endl;
        } else {
            cout << "未能检测到物体" << endl;
        }
        
        // 添加延时避免CPU占用过高
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 检查是否有退出信号(如键盘输入)
        if (cv::waitKey(1) == 27 || cv::waitKey(1) == 'q') {
            break;
        }
    }
    
    return 0;
}