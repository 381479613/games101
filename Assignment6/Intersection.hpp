//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_INTERSECTION_H
#define RAYTRACING_INTERSECTION_H
#include "Vector.hpp"
#include "Material.hpp"
class Object;
class Sphere;

struct Intersection
{
    Intersection(){
        happened=false;//光与三角形是否发生了碰撞的判断
        coords=Vector3f();//光首次打到三角形的坐标
        normal=Vector3f();//三角形的法线
        distance= std::numeric_limits<double>::max();//光从origin点出发抵达coords的距离，即tmax
        obj =nullptr;//BVH树中叶子节点包含的基元
        m=nullptr;//三角形的材质
    }
    bool happened;
    Vector3f coords;
    Vector3f normal;
    double distance;
    Object* obj;
    Material* m;
};
#endif //RAYTRACING_INTERSECTION_H
