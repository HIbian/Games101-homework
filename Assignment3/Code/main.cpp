#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos) {
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
            0, 1, 0, -eye_pos[1],
            0, 0, 1, -eye_pos[2],
            0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle) {
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
            0, 1, 0, 0,
            -sin(angle), 0, cos(angle), 0,
            0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
            0, 2.5, 0, 0,
            0, 0, 2.5, 0,
            0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar) {
    // TODO: Use the same projection matrix from the previous assignments

    Eigen::Matrix4f projection;

    float n = zNear;
    float f = zFar;
    Eigen::Matrix4f persp_to_ortho;
    persp_to_ortho << n, 0, 0, 0,
            0, n, 0, 0,
            0, 0, n + f, -n * f,
            0, 0, 1, 0;

    float b, t, l, r;
    // 保证不会上下颠倒.
    t = -zNear * tan(eye_fov / 2.0f / 180.0f * MY_PI);
    b = -t;
    r = t * aspect_ratio;
    l = -r;
    Eigen::Matrix4f ortho, ortho_a, ortho_b;
    ortho_a << 2.0f / (r - l), 0, 0, 0,
            0, 2.0f / (t - b), 0, 0,
            0, 0, 2.0f / (n - f), 0,
            0, 0, 0, 1;
    ortho_b << 1, 0, 0, -(r + l) / 2.0f,
            0, 1, 0, -(t + b) / 2.0f,
            0, 0, 1, -(n + f) / 2.0f,
            0, 0, 0, 1;
    ortho << ortho_a * ortho_b;

    projection << ortho * persp_to_ortho * projection;
    return projection;
}

Eigen::Vector3f vertex_shader(const vertex_shader_payload &payload) {
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload &payload) {
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f &vec, const Eigen::Vector3f &axis) {
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light {
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload &payload) {
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture) {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        return_color = payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y());
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20,  20,  20},
                    {500, 500, 500}};
    auto l2 = light{{-20, 20,  0},
                    {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto &light: lights) {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.

        //get l,n,v,r
        Eigen::Vector3f l = light.position - point;
        Eigen::Vector3f v = eye_pos - point;
        Eigen::Vector3f n = normal;
        float r_2 = std::pow(l.x(), 2) + std::pow(l.y(), 2) + std::pow(l.z(), 2);
        l.normalize();
        v.normalize();
        n.normalize();

        //镜面高光
        Eigen::Vector3f h = l + v;
        h.normalize();
        //cwiseproduct 是将两个向量的每个项单独相乘，如k*b=（kxbx,kyby,kzbz），因为环境光，漫反射系数是分别作用于每个分量上的，
        Eigen::Vector3f specular = ks.cwiseProduct(light.intensity / r_2) * std::pow(std::max(0.0f, n.dot(h)), p);
        //漫反射
        Eigen::Vector3f diffuse = kd.cwiseProduct(light.intensity / r_2) * std::max(0.0f, l.dot(n));
        //环境光
        Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);

        result_color += specular + diffuse + ambient;
    }

    return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload &payload) {
    //环境光项，漫反射项，高光项
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    //note: position:光线起点 intensity:光线强度
    auto l1 = light{{20,  20,  20},
                    {500, 500, 500}};
    auto l2 = light{{-20, 20,  0},
                    {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    //note:环境光强度
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    //note:观察方向
    Eigen::Vector3f eye_pos{0, 0, 10};
    //note:高光系数
    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto &light: lights) {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.

        //get l,n,v,r
        Eigen::Vector3f l = light.position - point;
        Eigen::Vector3f v = eye_pos - point;
        Eigen::Vector3f n = normal;
        float r_2 = std::pow(l.x(), 2) + std::pow(l.y(), 2) + std::pow(l.z(), 2);
        l.normalize();
        v.normalize();
        n.normalize();

        //镜面高光
        Eigen::Vector3f h = l + v;
        h.normalize();
        //cwiseproduct 是将两个向量的每个项单独相乘，如k*b=（kxbx,kyby,kzbz），因为环境光，漫反射系数是分别作用于每个分量上的，
        Eigen::Vector3f specular = ks.cwiseProduct(light.intensity / r_2) * std::pow(std::max(0.0f, n.dot(h)), p);
        //漫反射
        Eigen::Vector3f diffuse = kd.cwiseProduct(light.intensity / r_2) * std::max(0.0f, l.dot(n));
        //环境光
        Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);

        result_color += specular + diffuse + ambient;
    }
    return result_color * 255.f;
}


Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload &payload) {

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20,  20,  20},
                    {500, 500, 500}};
    auto l2 = light{{-20, 20,  0},
                    {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;

    // TODO: Implement displacement mapping here
    Eigen::Vector3f n = normal;
    Eigen::Vector3f t(n.x() * n.y() / std::sqrt(n.x() * n.x() + n.z() * n.z()),
                      std::sqrt(n.x() * n.x() + n.z() * n.z()),
                      n.z() * n.y() / std::sqrt(n.x() * n.x() + n.z() * n.z()));
    Eigen::Vector3f b = n.cross(t);
    //坐标系变换,推导 http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-13-normal-mapping/
    Eigen::Matrix3f TBN;
    TBN << t, b, n;
    float u = payload.tex_coords[0];
    float v = payload.tex_coords[1];
    float w = payload.texture->width;
    float h = payload.texture->height;
    //贴图提供的法线信息,norm()可以看作高度,转换为以为坐标.https://blog.csdn.net/m0_56348460/article/details/117386857
    float dU = kh * kn * (payload.texture->getColor(u + 1 / w, v).norm() - payload.texture->getColor(u, v).norm());
    float dV = kh * kn * (payload.texture->getColor(u, v + 1 / h).norm() - payload.texture->getColor(u, v).norm());
    Vector3f ln(-dU, -dV, 1);
    normal = (TBN * ln).normalized();

    // Position p = p + kn * n * h(u,v)
    point = point + kn * n * payload.texture->getColor(u, v).norm();

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto &light: lights) {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        //get l_vect,n_vec,v_vec,r
        Eigen::Vector3f l_vect = light.position - point;
        Eigen::Vector3f v_vec = eye_pos - point;
        Eigen::Vector3f n_vec = normal;
        float r_2 = std::pow(l_vect.x(), 2) + std::pow(l_vect.y(), 2) + std::pow(l_vect.z(), 2);
        l_vect.normalize();
        v_vec.normalize();
        n_vec.normalize();

        //镜面高光
        Eigen::Vector3f h_light = l_vect + v_vec;
        h_light.normalize();
        //cwiseproduct 是将两个向量的每个项单独相乘，如k*b=（kxbx,kyby,kzbz），因为环境光，漫反射系数是分别作用于每个分量上的，
        Eigen::Vector3f specular =
                ks.cwiseProduct(light.intensity / r_2) * std::pow(std::max(0.0f, n_vec.dot(h_light)), p);
        //漫反射
        Eigen::Vector3f diffuse = kd.cwiseProduct(light.intensity / r_2) * std::max(0.0f, l_vect.dot(n_vec));
        //环境光
        Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);

        result_color += specular + diffuse + ambient;

    }

    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload &payload) {

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20,  20,  20},
                    {500, 500, 500}};
    auto l2 = light{{-20, 20,  0},
                    {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    // get h_map


    float kh = 0.2, kn = 0.1;

    // TODO: Implement bump mapping here
//     Let n = normal = (x, y, z)
//     Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
//     Vector b = n cross product t
//     Matrix TBN = [t b n]
//     dU = kh * kn * (h(u+1/w,v)-h(u,v))
//     dV = kh * kn * (h(u,v+1/h)-h(u,v))
//     Vector ln = (-dU, -dV, 1)
//     Normal n = normalize(TBN * ln)
    Eigen::Vector3f n = normal;
    Eigen::Vector3f t(n.x() * n.y() / std::sqrt(n.x() * n.x() + n.z() * n.z()),
                      std::sqrt(n.x() * n.x() + n.z() * n.z()),
                      n.z() * n.y() / std::sqrt(n.x() * n.x() + n.z() * n.z()));
    Eigen::Vector3f b = n.cross(t);
    //坐标系变换,推导 http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-13-normal-mapping/
    Eigen::Matrix3f TBN;
    TBN << t, b, n;
    float u = payload.tex_coords[0];
    float v = payload.tex_coords[1];
    float w = payload.texture->width;
    float h = payload.texture->height;
    //贴图提供的法线信息,norm()可以看作高度,转换为以为坐标.https://blog.csdn.net/m0_56348460/article/details/117386857
    float dU = kh * kn * (payload.texture->getColor(u + 1 / w, v).norm() - payload.texture->getColor(u, v).norm());
    float dV = kh * kn * (payload.texture->getColor(u, v + 1 / h).norm() - payload.texture->getColor(u, v).norm());
    Vector3f ln(-dU, -dV, 1);
    normal = (TBN * ln).normalized();

    Eigen::Vector3f result_color = {0, 0, 0};
    result_color = normal;

    return result_color * 255.f;
}

int main(int argc, const char **argv) {
    std::vector<Triangle *> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
//    std::string obj_path = "../models/spot/";
    std::string obj_path = "../models/rock/";

    // Load .obj File
//    bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    bool loadout = Loader.LoadFile("../models/rock/rock.obj");
    for (auto mesh: Loader.LoadedMeshes) {
        for (int i = 0; i < mesh.Vertices.size(); i += 3) {
            Triangle *t = new Triangle();
            for (int j = 0; j < 3; j++) {
                t->setVertex(j, Vector4f(mesh.Vertices[i + j].Position.X, mesh.Vertices[i + j].Position.Y,
                                         mesh.Vertices[i + j].Position.Z, 1.0));
                t->setNormal(j, Vector3f(mesh.Vertices[i + j].Normal.X, mesh.Vertices[i + j].Normal.Y,
                                         mesh.Vertices[i + j].Normal.Z));
                t->setTexCoord(j, Vector2f(mesh.Vertices[i + j].TextureCoordinate.X,
                                           mesh.Vertices[i + j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);

//    auto texture_path = "hmap.jpg";
    auto texture_path = "rock.png";
    r.set_texture(Texture(obj_path + texture_path));

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

    if (argc >= 3) {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture") {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        } else if (argc == 3 && std::string(argv[2]) == "normal") {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        } else if (argc == 3 && std::string(argv[2]) == "phong") {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        } else if (argc == 3 && std::string(argv[2]) == "bump") {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        } else if (argc == 3 && std::string(argv[2]) == "displacement") {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = displacement_fragment_shader;
        }
    } else if (argc == 2) {
        //used when running in terminal
        if (std::string(argv[1]) == "texture") {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
//            texture_path = "spot_texture.png";
            texture_path = "rock.png";
            r.set_texture(Texture(obj_path + texture_path));
        } else if (std::string(argv[1]) == "normal") {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        } else if (std::string(argv[1]) == "phong") {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        } else if (std::string(argv[1]) == "bump") {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        } else if (std::string(argv[1]) == "displacement") {
            std::cout << "Rasterizing using the displacement shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    Eigen::Vector3f eye_pos = {0, 0, 10};
    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a') {
            angle -= 5;
        } else if (key == 'd') {
            angle += 5;
        }
        printf("frame:%d\n", ++frame_count);
    }
    return 0;
}
