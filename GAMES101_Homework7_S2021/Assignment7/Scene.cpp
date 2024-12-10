//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray& ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection& pos, float& pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum) {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
    const Ray& ray,
    const std::vector<Object*>& objects,
    float& tNear, uint32_t& index, Object** hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const 
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection p = intersect(ray);
    if (!p.happened) {
        return Vector3f(0.f, 0.f, 0.f);
    }
   
    Vector3f wo = ray.direction; // Note: wo is implemented opposite to that in the slide!
    return shade(p, wo);
}

// Very Helpful Reference: https://github.com/ysj1173886760/Learning/tree/master/graphics/GAMES101/PA7
Vector3f Scene::shade(const Intersection& p, const Vector3f& wo) const {

    if (p.obj->hasEmit()) {
        return p.m->getEmission(); 
    }

    // Part I: Direct lighting contribution from the light sources.
    Intersection x;
    float pdf_light;
    sampleLight(x, pdf_light);

    Vector3f ws = (x.coords - p.coords).normalized(); 
    float dist = (x.coords - p.coords).norm();
    Ray r1(p.coords, ws); 

    Intersection hit = intersect(r1);
    Vector3f L_dir(0.0f, 0.0f, 0.0f);
    if (hit.happened && (hit.coords - x.coords).norm() < EPSILON) { // If not blocked in the middle
        L_dir = x.emit * p.m->eval(wo, ws, p.normal) * dotProduct(p.normal, ws) * dotProduct(x.normal, -ws)
            / (dist * dist) / pdf_light; 
    }

    // Part II: Indirect lighting contribution from non-emitting objects
    Vector3f L_indir(0.f, 0.f, 0.f);
    if (get_random_float() < RussianRoulette) { // Want high probability => Capture of indirect effects. 

        Vector3f wi = p.m->sample(wo, p.normal);
        Ray r2(p.coords, wi);

        Intersection q;
        q = intersect(r2);

        if (q.happened && !q.obj->hasEmit()) {
            L_indir = shade(q, wi) * p.m->eval(wo, wi, p.normal) * dotProduct(wi, p.normal)
                / std::max(p.m->pdf(wo, wi, p.normal), EPSILON) / RussianRoulette;
        } 
    }

    // The point p considers both direct and indirect light sources by superposition
    return L_dir + L_indir;
}


