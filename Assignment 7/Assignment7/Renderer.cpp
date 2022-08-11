//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"

#include <thread>
#include <mutex>

std::mutex mutex_prog;
inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    // change the spp value to change sample ammount
    int spp = 16;
    std::cout << "SPP: " << spp << "\n";
    // for (uint32_t j = 0; j < scene.height; ++j) {
    //     for (uint32_t i = 0; i < scene.width; ++i) {
    //         // generate primary ray direction
    //         float x = (2 * (i + 0.5) / (float)scene.width - 1) *
    //                   imageAspectRatio * scale;
    //         float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

    //         Vector3f dir = normalize(Vector3f(-x, y, 1));
    //         for (int k = 0; k < spp; k++){
    //             framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
    //         }
    //         m++;
    //     }
    //     UpdateProgress(j / (float)scene.height);
    // }

    int process = 0;
    //匿名函数
    auto castRayMultiThread = [&](uint32_t rowStart,uint32_t rowEnd,
                                uint32_t colStart, uint32_t colEnd)
    {
        for(uint32_t j = colStart;j <= colEnd;j++)
        {
            int m = j * scene.width + rowStart;
            for(uint32_t i = rowStart;i<=rowEnd;i++)
            {
                float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                          imageAspectRatio * scale;
                float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

                Vector3f dir = normalize(Vector3f(-x, y, 1));
                for (int k = 0; k < spp; k++)
                {
                    framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
                }
                m++;
                process++;
            }
            std::lock_guard<std::mutex> lock1(mutex_prog);  //互斥锁，用于打印处理进度
            UpdateProgress(1.f*process/scene.width/scene.height);
        }
    };

    int id = 0;
    const int dx = 8;
    const int dy = 8;
    std::thread my_thread[dx*dy];
    int x_block = (scene.width+dx-1) / dx;
    int y_block = (scene.height+dy-1) / dy;
    //分块进行计算路径追踪
    for(int i=0;i<scene.width;i+=x_block)
    {
        for(int j=0;j<scene.height;j+=y_block)
        {
            my_thread[id] = std::thread(castRayMultiThread,
                            i,std::min(i+x_block,scene.width)-1,
                            j,std::min(j+y_block,scene.height)-1);
            id++;
        }
    }
    for(int i =0;i<dx*dy;i++)
        my_thread[i].join();

    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("16spp.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
