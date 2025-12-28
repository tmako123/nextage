#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

// ----------------------------------------------------
// OpenCV で簡易プロットを行う汎用ヘッダー
// ----------------------------------------------------
namespace ocvplot {

inline cv::Point toScreen(
    double x, double y,
    double minX, double maxX,
    double minY, double maxY,
    int width, int height)
{
    int px = (int)((x - minX) / (maxX - minX) * width);
    int py = (int)(height - (y - minY) / (maxY - minY) * height);
    return cv::Point(px, py);
}

inline void plot(
    const std::vector<std::vector<cv::Point2d>>& lines,
    const std::vector<std::vector<cv::Point2d>>& scatters,
    double minX, double maxX,
    double minY, double maxY,
    int width = 800, int height = 600,
    const std::string& winName = "Plot")
{
    cv::Mat img(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

    std::vector<cv::Scalar> colors = {
        {255,0,0}, {0,255,0}, {0,0,255},
        {255,128,0}, {128,0,255}, {0,255,255}
    };

    // ---- 線を描画 ----
    for (size_t i = 0; i < lines.size(); i++) {
        const auto& line = lines[i];
        for (size_t j = 1; j < line.size(); j++) {
            cv::line(img,
                toScreen(line[j-1].x, line[j-1].y, minX, maxX, minY, maxY, width, height),
                toScreen(line[j].x,   line[j].y,   minX, maxX, minY, maxY, width, height),
                colors[i % colors.size()], 2);
        }
    }

    // ---- 散布図を描画 ----
    for (size_t i = 0; i < scatters.size(); i++) {
        const auto& pts = scatters[i];
        for (auto& p : pts) {
            cv::circle(img,
                toScreen(p.x, p.y, minX, maxX, minY, maxY, width, height),
                3,
                colors[(i + lines.size()) % colors.size()],
                -1);
        }
    }

    cv::imshow(winName, img);
    cv::waitKey(0);
}

} // namespace ocvplot
