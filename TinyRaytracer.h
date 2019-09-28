// modified from https://github.com/ssloy/tinyraytracer
#ifndef _TINYRAYTRACER
#define _TINYRAYTRACER

#include <limits>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include "TinyRaytracer_Geometry.h"


uint16_t *imgBuffer = NULL; // output pixels
uint16_t  *bgBuffer = NULL; // 360 deg background pixels
uint8_t  *rgbBuffer = NULL;

// 0,2967 rad = 17 deg (ball tilt)
float st = sin( 0.2967 );
float ct = cos( 0.2967 );

extern bool renderfloor;
extern bool renderbg;
extern bool hasPsram;

bool rayhitsphere = false;
bool rayhitfloor = false;

float minx=INFINITY, miny=INFINITY;
float maxx=-INFINITY, maxy=-INFINITY;
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int envmap_width, envmap_height;


void tinyRayTracerInit() {
  // TODO : move this to tinytracer.h
  if( hasPsram ) {
    Serial.println("OK PSRAM");
    imgBuffer = (uint16_t*)ps_calloc( 320*240, sizeof( uint16_t ) );
    bgBuffer =  (uint16_t*)ps_calloc( 320*240, sizeof( uint16_t ) );
    rgbBuffer = (uint8_t*)ps_calloc( 320*240*3, sizeof( uint8_t ) );
  } else {
    Serial.println("NO PSRAM");
    imgBuffer = (uint16_t*)calloc( 128*128, sizeof( uint16_t ) );
    bgBuffer = (uint16_t*)calloc( 128*128, sizeof( uint16_t ) );
    rgbBuffer = (uint8_t*)calloc( 128*128*3, sizeof( uint8_t ) );
  }
}


struct Light {
    Light(const Vec3f &p, const float i) : position(p), intensity(i) {}
    Vec3f position;
    float intensity;
};

struct Material {
    Material(const float r, const Vec4f &a, const Vec3f &color, const float spec) : refractive_index(r), albedo(a), diffuse_color(color), specular_exponent(spec) {}
    Material() : refractive_index(1), albedo(1,0,0,0), diffuse_color(), specular_exponent() {}
    float refractive_index;
    Vec4f albedo;
    Vec3f diffuse_color;
    float specular_exponent;
};

struct Sphere {
    Vec3f center;
    float radius;
    Material material;

    Sphere(const Vec3f &c, const float r, const Material &m) : center(c), radius(r), material(m) {}

    bool ray_intersect(const Vec3f &orig, const Vec3f &dir, float &t0) const {
        Vec3f L = center - orig;
        float tca = L*dir;
        float d2 = L*L - tca*tca;
        if (d2 > radius*radius) return false;
        float thc = sqrtf(radius*radius - d2);
        t0       = tca - thc;
        float t1 = tca + thc;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return false;
        return true;
    }
};

Vec3f reflect(const Vec3f &I, const Vec3f &N) {
    return I - N*2.f*(I*N);
}

Vec3f refract(const Vec3f &I, const Vec3f &N, const float eta_t, const float eta_i=1.f) { // Snell's law
    float cosi = - std::max(-1.f, std::min(1.f, I*N));
    if (cosi<0) return refract(I, -N, eta_i, eta_t); // if the ray comes from the inside the object, swap the air and the media
    float eta = eta_i / eta_t;
    float k = 1 - eta*eta*(1 - cosi*cosi);
    return k<0 ? Vec3f(1,0,0) : I*eta + N*(eta*cosi - sqrtf(k)); // k<0 = total reflection, no ray to refract. I refract it anyways, this has no physical meaning
}



bool scene_intersect(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, Vec3f &hit, Vec3f &N, Material &material) {
    float spheres_dist = std::numeric_limits<float>::max();
    //rayhitsphere = false;
    for (size_t i=0; i < spheres.size(); i++) {
        float dist_i;
        if (spheres[i].ray_intersect(orig, dir, dist_i) && dist_i < spheres_dist) {
            spheres_dist = dist_i;
            hit = orig + dir*dist_i;
            N = (hit - spheres[i].center).normalize();
            material = spheres[i].material;

            float slices = PI*spheres[i].radius ;
            float islices = 1.5 / slices;
            float hslices = islices / 2;
            float angleX = (atan2(N.x*ct -N.y*st, N.z) / PI);
            float angleY = (acos(N.y*ct + N.x*st) / PI);
            float isredY = mapfloat( angleY, -PI, PI, 0., slices);
            float isredX = mapfloat( angleX, -PI, PI, 0., slices);
            isredY = fmod( isredY, islices );
            isredX = fmod( isredX, islices );

            bool togglecolor = false;
            if( isredY >= hslices ) {
              togglecolor = true;
            } else {
              togglecolor = false;
            }
            if( isredX >= hslices ) {
              togglecolor = !togglecolor;
            }
            material.diffuse_color = togglecolor ? Vec3f(1,0,0) : Vec3f(1, 1, 1);
            material.diffuse_color = material.diffuse_color*.5; // luminosity
            rayhitsphere = true;
        }
    }

    float checkerboard_dist = std::numeric_limits<float>::max();
    if (renderfloor && fabs(dir.y)>1e-3)  {
        float d = -(orig.y+4)/dir.y; // the checkerboard plane has equation y = -4
        Vec3f pt = orig + dir*d;
        if (d>0 && fabs(pt.x)<10 && pt.z<-10 && pt.z>-30 && d<spheres_dist) {
            checkerboard_dist = d;
            hit = pt;
            N = Vec3f(0,1,0);
            material.diffuse_color = (int(.5*hit.x+1000) + int(.5*hit.z)) & 1 ? Vec3f(1,0.,0) : Vec3f(1, 1, 1);
            material.diffuse_color = material.diffuse_color*.3; // luminosity
            rayhitfloor = true;
            rayhitsphere = false;
        }
    }
    return std::min(spheres_dist, checkerboard_dist)<1000;
}




Vec3f cast_ray(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, const std::vector<Light> &lights, size_t depth=0) {
    Vec3f point, N;
    Material material;
    if (depth>4 || !scene_intersect(orig, dir, spheres, point, N, material)) {
        if( renderbg && bgBuffer!=NULL ) {
          int x = std::max(0, std::min(envmap_width -1, static_cast<int>((atan2(dir.z, dir.x)/(2*M_PI) + .5)*envmap_width)));
          int y = std::max(0, std::min(envmap_height-1, static_cast<int>(acos(dir.y)/M_PI*envmap_height)));
          uint32_t bgIndex = x+y*envmap_width;
          uint16_t pixcolor = bgBuffer[bgIndex];
          uint8_t r = (((((pixcolor >> 11) & 0x1F) * 527) + 23) >> 6);
          uint8_t g = (((((pixcolor >> 5) & 0x3F) * 259) + 33) >> 6);
          uint8_t b = ((((pixcolor & 0x1F) * 527) + 23) >> 6);
          return Vec3f( r/255., g/255., b/255. );
        } else {
          return Vec3f(0.2, 0.7, 0.8); // background color
        }
    }

    Vec3f reflect_dir = reflect(dir, N).normalize();
    Vec3f refract_dir = refract(dir, N, material.refractive_index).normalize();
    Vec3f reflect_orig = reflect_dir*N < 0 ? point - N*1e-3 : point + N*1e-3; // offset the original point to avoid occlusion by the object itself
    Vec3f refract_orig = refract_dir*N < 0 ? point - N*1e-3 : point + N*1e-3;
    Vec3f reflect_color = cast_ray(reflect_orig, reflect_dir, spheres, lights, depth + 1);
    Vec3f refract_color = cast_ray(refract_orig, refract_dir, spheres, lights, depth + 1);

    float diffuse_light_intensity = 0, specular_light_intensity = 0;
    for (size_t i=0; i<lights.size(); i++) {
        Vec3f light_dir      = (lights[i].position - point).normalize();
        float light_distance = (lights[i].position - point).norm();

        Vec3f shadow_orig = light_dir*N < 0 ? point - N*1e-3 : point + N*1e-3; // checking if the point lies in the shadow of the lights[i]
        Vec3f shadow_pt, shadow_N;
        Material tmpmaterial;
        if (scene_intersect(shadow_orig, light_dir, spheres, shadow_pt, shadow_N, tmpmaterial) && (shadow_pt-shadow_orig).norm() < light_distance) {
          continue;
        }
        diffuse_light_intensity  += lights[i].intensity * std::max(0.f, light_dir*N);
        specular_light_intensity += powf(std::max(0.f, -reflect(-light_dir, N)*dir), material.specular_exponent)*lights[i].intensity;
    }

    return material.diffuse_color * diffuse_light_intensity * material.albedo[0] + Vec3f(1., 1., 1.)*specular_light_intensity * material.albedo[1] + reflect_color*material.albedo[2] + refract_color*material.albedo[3];
}


#endif
