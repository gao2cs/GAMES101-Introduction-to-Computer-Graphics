//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <mutex>
#include <thread>

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 1e-4;

// The main function also uses a thread called "main thread"
// Helpful References: https://www.youtube.com/watch?v=lncSwlsDhdk&list=PLoCMsyE1cvdUJvvBjBOJKf3rc1xj7_G7g&index=23
//                     https://blueflame.org.cn/archives/439
//                     https://github.com/ysj1173886760/Learning/tree/master/graphics/GAMES101/PA7

std::mutex mtx;
void Renderer::MultiThreadRender(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);

    // change the spp value to change sample ammount
    int spp = 16;
    std::cout << "SPP: " << spp << "\n";

    // Initialize Multi-Threads
    std::vector<std::thread> threads;
    const int kNumThreads = 20;

    int rowsPerThread = scene.height / kNumThreads;
    int renderProgress = 0;

    auto renderRow = [&](int start_row, int end_row) { // &: pass by reference; =: pass by value

        int m = scene.width * start_row;
        for (uint32_t j = start_row; j < end_row; ++j) {
            for (uint32_t i = 0; i < scene.width; ++i) {
                // generate primary ray direction
                float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                    imageAspectRatio * scale;
                float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

                Vector3f dir = normalize(Vector3f(-x, y, 1));
                for (int k = 0; k < spp; k++) {
                    framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
                }
                m++;
            }

            // RAII Compliant: Handles lock & unlock
            std::lock_guard<std::mutex> lg(mtx);
            UpdateProgress(renderProgress++ / (float)scene.height); // Prevent racing for reporting progress
        }
    };

    // Distribute rendering jobs to kNumThreads of workers in parallel
    for (uint32_t i = 0; i < kNumThreads; ++i) {
        threads.emplace_back(renderRow, i * rowsPerThread, (i + 1) * rowsPerThread);
    }

    // "join" ensures that the main thread waits at this point until all other threads have completed their execution.
    for (std::thread& t : threads) {
        t.join();
    }

    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
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

 //The main render function. This where we iterate over all pixels in the image,
 //generate primary rays and cast these rays into the scene. The content of the
 //framebuffer is saved to a file.
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
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            // generate primary ray direction
            float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

            Vector3f dir = normalize(Vector3f(-x, y, 1));
            for (int k = 0; k < spp; k++) {
                framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
            }
            m++;
        }
        UpdateProgress(j / (float)scene.height);
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
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


