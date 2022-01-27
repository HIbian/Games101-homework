#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) {
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
                  << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) {
    auto p_0 = points[0];
    auto p_1 = points[1];
    auto p_2 = points[2];
    auto p_3 = points[3];
    p_0.x+=30.f;
    p_1.x+=30.f;
    p_2.x+=30.f;
    p_3.x+=30.f;

    for (double t = 0.0; t <= 1.0; t += 0.001) {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                     3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) {
    // TODO: Implement de Casteljau's algorithm
    //维护一个控制点列表
    std::vector<cv::Point2f> c_points = control_points;
    //计算下一组控制点位置
    while (c_points.size() != 1) {
        for (int i = 0; i < c_points.size() - 1; ++i) {
            c_points[i] = (1 - t) * c_points[i] + t * c_points[i + 1];
        }
        c_points.pop_back();
    }
    std::cout << c_points[0] << std::endl;
    return c_points[0];
}

int round_double(double number) {
    return (number > 0.0) ? (number + 0.5) : (number - 0.5);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) {
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    const double max_distence_2 = 0.5;
    for (double t = 0.0; t <= 1.0; t += 0.0001) {
        auto point = recursive_bezier(control_points, t);
        //反走样
        double r_2 = std::pow(round_double(point.x) - point.x, 2) + std::pow(round_double(point.y) - point.y, 2);
        double color = std::sqrt(r_2 / max_distence_2) * 255;
        //因为有重复的点在同一的像素上，所以需要取颜色最深的
        if (window.at<cv::Vec3b>(point.y, point.x)[1] < color)
            window.at<cv::Vec3b>(point.y, point.x)[1] = color;
    }

}

int main() {
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) {
        for (auto &point: control_points) {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) {
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
