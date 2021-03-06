// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions) {
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices) {
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols) {
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f &v3, float w = 1.0f) {
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(float x, float y, const Vector3f *_v) {
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f a = _v[0], b = _v[1], c = _v[2];
    Vector3f p(x, y, 1);
    a.z() = 1;
    b.z() = 1;
    c.z() = 1;

    Vector3f ab = b - a, bc = c - b, ca = a - c,
            ap = p - a, bp = p - b, cp = p - c;
    if ((ab.cross(ap).z() > 0 && bc.cross(bp).z() > 0 && ca.cross(cp).z() > 0) ||
        (ab.cross(ap).z() < 0 && bc.cross(bp).z() < 0 && ca.cross(cp).z() < 0)) {
        return true;
    }
    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f *v) {
    float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) /
               (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() -
                v[2].x() * v[1].y());
    float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) /
               (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() -
                v[0].x() * v[2].y());
    float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) /
               (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() -
                v[1].x() * v[0].y());
    return {c1, c2, c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type) {
    auto &buf = pos_buf[pos_buffer.pos_id];
    auto &ind = ind_buf[ind_buffer.ind_id];
    auto &col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto &i: ind) {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto &vec: v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto &vert: v) {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i) {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle &t) {
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    //init bounding box
    float boundingBox[4] = {v.at(0).x(), v.at(0).y(), v.at(0).x(), v.at(0).y()};
    for (Vector4f vc: v) {
        if (vc.x() <= boundingBox[0])
            boundingBox[0] = floorf(vc.x());
        if (vc.x() >= boundingBox[2])
            boundingBox[2] = ceilf(vc.x());

        if (vc.y() <= boundingBox[1])
            boundingBox[1] = floorf(vc.y());
        if (vc.y() >= boundingBox[3])
            boundingBox[3] = ceilf(vc.y());
    }

    // iterate through the pixel and find if the current pixel is inside the triangle
    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
    for (int x = (int) boundingBox[0]; x <= (int) boundingBox[2]; x++) {
        for (int y = (int) boundingBox[1]; y <= (int) boundingBox[3]; y++) {
            //?????????(x,y)?????????
            auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
            float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            float z_interpolated =
                    alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            z_interpolated *= w_reciprocal;

            //MSAA 2*2
            //todo ??????????????????????????????????????????????????? (????????????????????????????????????????????????)
            //??????4?????????????????????
            float gap = 0.25f;
            Vector3f plist[2 * 2] = {Vector3f(x - gap, y + gap, 1),
                                     Vector3f(x + gap, y + gap, 1),
                                     Vector3f(x - gap, y - gap, 1),
                                     Vector3f(x + gap, y - gap, 1)};
            //??????????????????????????????????????????,??????????????????????????????????????????????????????????????????
            for (int i = 0; i < 4; i++) {
                if (!insideTriangle(plist[i].x(), plist[i].y(), t.v))
                    continue;
                //????????????
                if (z_interpolated < mass_depth_buf[get_index(x, y)][i]) {
                    mass_depth_buf[get_index(x, y)][i] = z_interpolated;
                    mass_color_buf[get_index(x, y)][i] = t.getColor();
                }
            }
            //??????(x,y)??????????????????
            Vector3f color(0, 0, 0);
            for (Vector3f c: mass_color_buf[get_index(x, y)]) {
                color = color + c;
            }
            set_pixel(Vector3f(x, y, 0), color / 4);
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f &m) {
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f &v) {
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f &p) {
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff) {
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color) {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        for (auto & i : mass_color_buf) {
            for (auto &j:i) {
                j = Vector3f(0,0,0);
            }
        }
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth) {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        for (auto & i : mass_depth_buf) {
            for (auto &j:i) {
                j = 0;
            }
        }
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h) {
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);

    //?????????mass
    mass_color_buf.resize(w * h);
    mass_depth_buf.resize(w * h);
    for (int i = 0; i < w*h; ++i) {
       mass_color_buf[i].resize(4,Vector3f(0,0,0));
       mass_depth_buf[i].resize(4,std::numeric_limits<float>::infinity());
    }

}

int rst::rasterizer::get_index(int x, int y) {
    return (height - 1 - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f &point, const Eigen::Vector3f &color) {
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height - 1 - point.y()) * width + point.x();
    frame_buf[ind] = color;

}

// clang-format on