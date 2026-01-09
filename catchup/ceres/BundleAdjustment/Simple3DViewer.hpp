#ifndef SIMPLE_3D_VIEWER_HPP
#define SIMPLE_3D_VIEWER_HPP

#include <opencv2/opencv.hpp>
#include <vector>

class Simple3DViewer
{
private:
    struct ColoredCloud
    {
        std::vector<cv::Point3f> points;
        cv::Scalar color;
        int pointSize;
    };

    struct ColoredCamera
    {
        cv::Affine3f pose;
        cv::Scalar color;
        float scale;
        int thickness;
    };

    std::vector<ColoredCloud> clouds;
    std::vector<ColoredCamera> cameras;

    struct State
    {
        cv::Vec3f rotation = {0.5f, 0.5f, 0.0f};
        cv::Vec3f center = {0, 0, 0};
        float zoom = 5.0f;
        cv::Point2f lastMouse;
        bool isDragging = false;
    } state;

public:
    void addCloud(const std::vector<cv::Point3f> &points,
                  const cv::Scalar &color = cv::Scalar(0, 255, 0),
                  int pointSize = 1)
    {
        clouds.push_back({points, color, pointSize});
    }

    void addCamera(const cv::Affine3f &pose,
                   const cv::Scalar &color = cv::Scalar(0, 0, 255),
                   float scale = 0.1f,
                   int thickness = 1)
    {
        cameras.push_back({pose, color, scale, thickness});
    }

    void clear()
    {
        clouds.clear();
        cameras.clear();
    }

    void show(const std::string &winName = "3D Viewer")
    {
        // 1. 重心とズームの自動調整
        cv::Vec3f sum(0, 0, 0);
        size_t totalCount = 0;
        std::vector<cv::Vec3f> allPoints;

        for (const auto &c : clouds)
        {
            for (const auto &p : c.points)
            {
                cv::Vec3f v(p.x, p.y, p.z);
                sum += v;
                allPoints.push_back(v);
                totalCount++;
            }
        }
        for (const auto &cam : cameras)
        {
            cv::Vec3f v = cam.pose.translation();
            sum += v;
            allPoints.push_back(v);
            totalCount++;
        }

        if (totalCount > 0)
        {
            state.center = sum * (1.0f / totalCount);
            float maxDistSq = 0;
            for (const auto &p : allPoints)
            {
                float d2 = cv::norm(p - state.center, cv::NORM_L2SQR);
                if (d2 > maxDistSq)
                    maxDistSq = d2;
            }
            state.zoom = std::sqrt(maxDistSq) * 2.5f;
            if (state.zoom < 0.1f)
                state.zoom = 5.0f;
        }

        // 2. ウィンドウとマウスコールバックの設定 (重要！)
        cv::namedWindow(winName);
        cv::setMouseCallback(winName, [](int event, int x, int y, int flags, void *userdata)
                             {
            State* s = (State*)userdata;
            if (event == cv::EVENT_LBUTTONDOWN) {
                s->isDragging = true;
                s->lastMouse = cv::Point2f((float)x, (float)y);
            }
            else if (event == cv::EVENT_LBUTTONUP) {
                s->isDragging = false;
            }
            else if (event == cv::EVENT_MOUSEMOVE && s->isDragging) {
                cv::Point2f current((float)x, (float)y);
                cv::Point2f diff = current - s->lastMouse;
                s->rotation[1] += diff.x * 0.01f;
                s->rotation[0] -= diff.y * 0.01f;
                s->lastMouse = current;
            }
            else if (event == cv::EVENT_MOUSEWHEEL) {
                int delta = cv::getMouseWheelDelta(flags);
                if (delta > 0) s->zoom *= 0.9f;
                else if (delta < 0) s->zoom *= 1.1f;
            } }, &state);

        // 3. メイン描画ループ
        while (cv::waitKey(1) != 'q')
        {
            cv::Mat img = cv::Mat::zeros(600, 800, CV_8UC3);
            cv::Mat K = (cv::Mat_<double>(3, 3) << 800, 0, 400, 0, 800, 300, 0, 0, 1);

            cv::Mat R;
            cv::Rodrigues(state.rotation, R);
            // 重心を中心に回転させる行列計算
            cv::Mat t_final = R * cv::Mat(-state.center) + cv::Mat(cv::Vec3f(0, 0, state.zoom));

            // 点群の投影描画
            for (const auto &c : clouds)
            {
                if (c.points.empty())
                    continue;
                std::vector<cv::Point2f> imgPts;
                cv::projectPoints(c.points, state.rotation, t_final, K, cv::noArray(), imgPts);
                for (const auto &p : imgPts)
                {
                    if (p.x >= 0 && p.x < img.cols && p.y >= 0 && p.y < img.rows)
                    {
                        if (c.pointSize <= 1)
                            img.at<cv::Vec3b>((int)p.y, (int)p.x) = cv::Vec3b((uchar)c.color[0], (uchar)c.color[1], (uchar)c.color[2]);
                        else
                            cv::circle(img, p, c.pointSize, c.color, -1);
                    }
                }
            }

            // カメラの投影描画
            for (const auto &cam : cameras)
            {
                std::vector<cv::Point3f> model = {{0, 0, 0}, {1, 1, 2}, {1, -1, 2}, {-1, -1, 2}, {-1, 1, 2}};
                std::vector<cv::Point3f> world;
                for (auto p : model)
                    world.push_back(cam.pose * (p * cam.scale));

                std::vector<cv::Point2f> camPts;
                cv::projectPoints(world, state.rotation, t_final, K, cv::noArray(), camPts);

                for (int i = 1; i <= 4; ++i)
                {
                    cv::line(img, camPts[0], camPts[i], cam.color, cam.thickness);
                    cv::line(img, camPts[i], camPts[i % 4 + 1], cam.color, cam.thickness);
                }
            }

            cv::putText(img, "Drag:Rotate, Wheel:Zoom, Q:Quit", {10, 25}, 0, 0.5, {255, 255, 255});
            cv::imshow(winName, img);
        }
    }
};

#endif