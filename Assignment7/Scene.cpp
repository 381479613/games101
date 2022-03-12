//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
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
//if __name__=='__main__'
// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    //intersect(const Ray ray) :求一条光线与场景的交点
    //sampleLight(Intersection pos,float pdf) :在场景的所有光源上按面积uniform地sample一个点
    //并计算sample的概率密度
    //sample(const Vector3f wi, const Vector3f N) :按照该材质的性质，给定入射方向与法向量，
    //采用某种分布采样一个出射方向
    //pdf(const Vector3f wi,const Vector3f w0,const Vector3f N) : 给定一对出射、入射方向与
    //法向量，计算sample方法得到该出射方向的概率密度
    //eval(const Vector3f wi,const Vector3f wo,const Vector3f N) :给定一队入射、出射方向与
    //法向量，计算这种情况下的f_r值
    //RussianRoulette: P_RR的概率
    Vector3f L_dir = {0,0,0}, L_indir = {0,0,0};
    Intersection intersection = Scene::intersect(ray);//求交
    if (!intersection.happened)
        return {};
    if (intersection.m->hasEmission())//交点是光源情况
        return intersection.m->getEmission();
    //交点是物体，向光源采样
    Intersection lightpos;
    float lightpdf = 0.0f;
    sampleLight(lightpos,lightpdf);//获得对光源的采样
    Vector3f collisionlight = lightpos.coords - intersection.coords;
    float dis = dotProduct(collisionlight,collisionlight);
    Vector3f collisionlightdir = collisionlight.normalized();
    Ray light_to_object_ray(intersection.coords,collisionlightdir);
    Intersection light_to_anything_ray = Scene::intersect(light_to_object_ray);
    auto f_r = intersection.m->eval(ray.direction,collisionlightdir,intersection.normal);
    if (light_to_anything_ray.distance - collisionlight.norm()>-0.005)
    {
        L_dir = lightpos.emit*f_r*dotProduct(collisionlightdir,intersection.normal)*dotProduct(-collisionlightdir,lightpos.normal)/lightpdf/dis;
    }
    //交点为物体，产生迭代
    if (get_random_float()>RussianRoulette)
        return L_dir;
    Vector3f w0=intersection.m->sample(ray.direction,intersection.normal).normalized();
    Ray object_to_object_ray(intersection.coords,w0);
    Intersection islight=Scene::intersect(object_to_object_ray);
    if (islight.happened && !islight.m->hasEmission())
    {
        float pdf = intersection.m->pdf(ray.direction,w0,intersection.normal);
        f_r=intersection.m->eval(ray.direction,w0,intersection.normal);
        L_indir=castRay(object_to_object_ray,depth+1)*f_r*dotProduct(w0,intersection.normal)/pdf/RussianRoulette;
    }
    return L_dir+L_indir;
}
//endl