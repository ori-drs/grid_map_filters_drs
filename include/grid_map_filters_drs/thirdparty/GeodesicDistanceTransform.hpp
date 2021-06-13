/**
 * This is based on Alex Yu's Fast Marching Method implementation
 * Original repo: https://github.com/sxyu/FMM
 * 
 *  Copyright 2019 Alex Yu
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  Fast marching method implementation
 *  usage: fmm::fmm(image, seeds, weight_map_type = IDENTITY,
 *                  segmentation_threshold = disabled,
 *                  normalize_output_geodesic_distances = true,
 *                  output)
 *  > returns an image, either geodesic distance map or, if
 *    segmentation_threshold is given, a segmentation mask
 *  image: input image (OpenCV Mat)
 *  seeds: std::vector of OpenCV Points
 *  weight_map_type: transformation to apply to input image to use as FMM
*                    weight function. Can be one of:
 *                    fmm::weight::IDENTITY  no transformation (avoids a copy, faster)
 *                    fmm::weight::GRADIENT  recommended: image gradient magnitude (using Sobel gradients) 
 *                    fmm::weight::ABSDIFF   absolute difference from average
 *                                           grayscale value of seeds
 *                    fmm::weight::LAPLACIAN absolute value of image Laplacian
 *  segmentation_threshold: if specified, sets pixels with geodesic value less
 *                          than or equal to this threshold to 1 and others to 0
 *                          in the output image. If not given,the geodesic
 *                          distances map will be returned.
 *  max_visits: maximum number of points to visit. Can help speed up the computation
 *              if objects larger than a certain area are eliminated (e.g. background)
 *  normalize_output_geodesic_distances: if true, normalizes geodesic distances
 *                                       values to be in the interval [0, 1].
 *                                       If segmentation_threshold is specified,
 *                                       this occurs prior to segmentation.
 *                                       Default true.
 *  output: optionally, an already-allocated OpenCV Mat.
 *          This allows you to avoid a copy if you already have one
 *          allocated. By default a new image is created, and this
 *          is not necessary.
 * */
#pragma once

#include <cmath>
#include <cstring>
#include <algorithm>
#include <vector>
#include <queue>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace __internal {
    // Cell for priority queue
    template<class T> struct __queue_cell {
        int id, x; 
        T dist;
        __queue_cell() {}
        __queue_cell(int id, int x, T dist) : id(id), x(x), dist(dist) { }
    };
}

template<class T>
void GeodesicDistance(const cv::Mat& image,
         const std::vector<cv::Point>& seeds,
         cv::Mat& output,
         int max_visits = -1) {

    using namespace __internal;

    // Preallocate output
    static constexpr T INF = std::numeric_limits<T>::max(); 
    const int area = image.cols * image.rows;

    if (output.empty()) {
        output.create(image.rows, image.cols, image.type());
    }
    output.setTo(INF);

    T* dist = reinterpret_cast<T*>(output.data);
    
    // Define some inline methods to compare cells
    auto __compare = [] (const __queue_cell<T>& a, const __queue_cell<T>& b) {
            return a.dist > b.dist;
        };
    std::priority_queue<__queue_cell<T>,
                        std::vector<__queue_cell<T>>,
                        decltype(__compare)> que(__compare);
    std::vector<unsigned char> cell_status(area);

    for (const auto& seed : seeds) {
        const auto id = seed.x + seed.y * image.cols;
        que.emplace(id, seed.x, 0.f);
        dist[id] = 0.;
    }

    const T* image_data = reinterpret_cast<T*>(image.data);
    __queue_cell<T> u;

    // Eikonal update for the Fast Marching Method
    auto __eikonal_update = [&u, &image, area, dist, image_data]() {
        const T dleft  = u.x > 0                  ? dist[u.id - 1]    : INF;
        const T dright = u.x + 1 < image.cols     ? dist[u.id + 1]    : INF;
        const T dup    = u.id - image.cols >= 0   ? dist[u.id - image.cols] : INF;
        const T ddown  = u.id + image.cols < area ? dist[u.id + image.cols] : INF;

        const T dhoriz = std::min(dleft, dright), dvert = std::min(dup, ddown);
        const T cell_val = image_data[u.id];
        const T det = 2*dvert*dhoriz - dvert*dvert - dhoriz*dhoriz + 2*cell_val*cell_val;
        if(det >= 0.) {
            return 0.5f * (dhoriz+dvert + std::sqrt(det));
        } else {
            return std::min(dhoriz, dvert) + cell_val;
        }
    };

    constexpr unsigned char __CELL_VISITED = (unsigned char)(255),
                   __CELL_NOT_VISITED = (unsigned char)(0);
    auto __update_cell = [area, &image, &u, image_data, &que, dist, &cell_status, &__eikonal_update,
                          __CELL_VISITED, __CELL_NOT_VISITED]() {
        if (cell_status[u.id] == __CELL_VISITED) return;
        const T estimate = __eikonal_update();
        if (estimate < dist[u.id]) {
            dist[u.id] = estimate;
            if (cell_status[u.id] == __CELL_NOT_VISITED) {
                u.dist = estimate;
                que.emplace(u);
                if (!cell_status[u.id]) cell_status[u.id] = 1;
            }
        }
    };

    while (!que.empty() && max_visits--) {
        u = que.top(); que.pop();

        if (cell_status[u.id] == __CELL_VISITED) continue;
        cell_status[u.id] = __CELL_VISITED;

        --u.id; --u.x;
        if (u.x >= 0) __update_cell();
        u.id += 2; u.x += 2;
        if (u.x < image.cols) __update_cell();
        --u.id; --u.x;

        u.id -= image.cols;
        if (u.id >= 0) __update_cell();
        u.id += image.cols * 2;
        if (u.id < area) __update_cell();
    }
}
