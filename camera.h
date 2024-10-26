#ifndef CAMERA_H
#define CAMERA_H

#include "common.h"
#include "hittable.h"
#include "material.h"

#include <fstream>
#include <thread>
#include <vector>

class camera {
    public:
        double aspect_ratio = 1.0;  
        int    image_width  = 2;  
        int    samples_per_pixel = 10;   
        int    max_depth         = 10;   

        double vfov = 90;  
        point3 lookfrom = point3(0,0,0);   
        point3 lookat   = point3(0,0,-1);  
        vec3   vup      = vec3(0,1,0);     


        double defocus_angle = 0;  
        double focus_dist = 10;    

        void render_section(hittable& world, int start, int end) {

            for (int j = start; j < end; j++) {
                for (int i = 0; i < image_width; i++) {
                    color pixel_color(0,0,0);
                    for (int sample = 0; sample < samples_per_pixel; sample++) {
                        ray r = get_ray(i, j);
                        pixel_color += ray_color(r, max_depth, world);
                    }
                    image[j][i] = pixel_samples_scale * pixel_color;

                    completed++;
                }
                auto progress = 100.0 * completed / (image_height * image_width);
                std::clog << progress << "% complete\n" << std::flush;
            }
        }

        void render(hittable& world, int i) {
            std::ofstream ofs("output/img" + std::to_string(i) + ".ppm");
            initialize();

            int section_height = image_height / threads.size();
            for (int i = 0; i < threads.size(); i++) {
                int start = i * section_height;
                int end = (i == threads.size() - 1) ? image_height : (i + 1) * section_height;
                threads[i] = std::thread(&camera::render_section, this, std::ref(world), start, end);
            }

            for (int i = 0; i < threads.size(); i++)
                threads[i].join();

            ofs << "P3\n" << image_width << ' ' << image_height << "\n255\n";

            for (int j = 0; j < image_height; j++) {
                std::clog << "\rScanlines remaining: " << (image_height - j) << ' ' << std::flush;
                for (int i = 0; i < image_width; i++) {
                    write_color(ofs, image[j][i]);
                }
            }

            std::clog << "\rDone.                 \n";

            world.update();

            completed = 0;
        }

    private:
        int    image_height;   
        double pixel_samples_scale;  
        point3 center;         
        point3 pixel00_loc;    
        vec3   pixel_delta_u;  
        vec3   pixel_delta_v;  
        vec3   u, v, w;              
        vec3   defocus_disk_u;       
        vec3   defocus_disk_v;       
        std::vector<std::thread> threads;
        std::vector<std::vector<color>> image;
        int completed = 0;

        void initialize() {
            image_height = int(image_width / aspect_ratio);
            image_height = (image_height < 1) ? 1 : image_height;

            pixel_samples_scale = 1.0 / samples_per_pixel;
            center = lookfrom;



            auto theta = degrees_to_radians(vfov);
            auto h = tan(theta/2);
            auto viewport_height = 2 * h * focus_dist;
            auto viewport_width = viewport_height * (double(image_width)/image_height);


            w = unit_vector(lookfrom - lookat);
            u = unit_vector(cross(vup, w));
            v = cross(w, u);


            vec3 viewport_u = viewport_width * u;    
            vec3 viewport_v = viewport_height * -v;  

            pixel_delta_u = viewport_u / image_width;
            pixel_delta_v = viewport_v / image_height;


            auto viewport_upper_left = center - (focus_dist * w) - viewport_u/2 - viewport_v/2;
            pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);


            auto defocus_radius = focus_dist * tan(degrees_to_radians(defocus_angle / 2));
            defocus_disk_u = u * defocus_radius;
            defocus_disk_v = v * defocus_radius;

            image.resize(image_height);
            for (int i = 0; i < image_height; i++)
                image[i].resize(image_width);

            int thread_count = std::thread::hardware_concurrency();
            threads.resize(thread_count);
        }

        ray get_ray(int i, int j) const {

            auto offset = sample_square();
            auto pixel_sample = pixel00_loc
                + ((i + offset.x()) * pixel_delta_u)
                + ((j + offset.y()) * pixel_delta_v);

            auto ray_origin = (defocus_angle <= 0) ? center : defocus_disk_sample();
            auto ray_direction = pixel_sample - ray_origin;

            return ray(ray_origin, ray_direction);
        }

        vec3 sample_square() const {

            return vec3(random_double() - 0.5, random_double() - 0.5, 0);
        }

        point3 defocus_disk_sample() const {

            auto p = random_in_unit_disk();
            return center + (p[0] * defocus_disk_u) + (p[1] * defocus_disk_v);
        }

        color ray_color(const ray& r, int depth, const hittable& world) const {

            if (depth <= 0)
                return color(0,0,0);

            hit_record rec;

            if (world.hit(r, interval(0.001, infinity), rec)) {
                ray scattered;
                color attenuation;
                if (rec.mat->scatter(r, rec, attenuation, scattered))
                    return attenuation * ray_color(scattered, depth-1, world);
                return color(0,0,0);
            }

            vec3 unit_direction = unit_vector(r.direction());
            auto a = 0.5*(unit_direction.y() + 1.0);
            return (1.0-a)*color(1.0, 1.0, 1.0) + a*color(0.5, 0.7, 1.0);
        }
};

#endif
