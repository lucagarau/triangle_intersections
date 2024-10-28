#ifndef TRIANGLE_INTERSECTIONS_TRI_TRI_INTERSECT_H
#define TRIANGLE_INTERSECTIONS_TRI_TRI_INTERSECT_H

#include "orientTemplated.h"

typedef enum {
   VA_ON_VB = 0,
   VA_ON_EB = 1,
   VB_ON_EA = 2,
   VA_ON_TB = 3,
   VB_ON_TA = 4,
   EA_CROSS_EB = 5,
   EA_CROSS_TB = 6,
   EB_CROSS_TA = 7,
} intersectionType;

/*
typedef struct {
    vector<int> intersectionsPoints;
    vector<int> intersectionsEdges;
} intersectionResult;*/

class intersectionResult {
public:
    std::vector<int> intersectionsPoints = {};
    std::vector<int> intersectionsEdges = {};

    void addIntersectionPoint(intersectionType type, int index0, int index1){
        intersectionsPoints.push_back(type);
        intersectionsPoints.push_back(index0);
        intersectionsPoints.push_back(index1);
    };

    void addIntersectionEdge(int index0, int index1){
        intersectionsEdges.push_back(index0);
        intersectionsEdges.push_back(index1);
    };

    void printIntersectionPoints(){
        for (int i = 0; i < intersectionsPoints.size(); i+=3){
            std::cout << "Intersection type: " << parseIntersectionType((intersectionType)intersectionsPoints[i]) << ": " << intersectionsPoints[i+1] << " and " << intersectionsPoints[i+2] << std::endl;
        }
    };

    void printIntersectionEdges(){
        for (int i = 0; i < intersectionsEdges.size(); i+=2){
            std::cout << "Intersection edge: " << intersectionsEdges[i] << " and " << intersectionsEdges[i+1] << std::endl;
        }
    };

    string parseIntersectionType(intersectionType t){
        switch (t) {
            case VA_ON_VB: return "VA_ON_VB";
            case VA_ON_EB: return "VA_ON_EB";
            case VB_ON_EA: return "VB_ON_EA";
            case VA_ON_TB: return "VA_ON_TB";
            case VB_ON_TA: return "VB_ON_TA";
            case EA_CROSS_EB: return "EA_CROSS_EB";
            case EA_CROSS_TB: return "EA_CROSS_TB";
            case EB_CROSS_TA: return "EB_CROSS_TA";
        }
        return "Unknown type";
    }
};

template <typename T>
bool points_are_coincident(const T * p0, const T * p1);

template <typename T>
bool points_are_colinear_3d(const T * p0, const T * p1, const T * p2);

template <typename T>
bool point_in_inner_segment(const T * p, const T * a, const T * b);

template <typename T>
bool point_in_triangle_2d(const T * p,const T * t0,const T * t1,const T * t2);

template <typename T>
bool point_in_inner_triangle(const T * p, const T * v0, const T * v1, const T * v2);

template <typename T>
bool segment_cross_segment_2D (const T * a0, const T * a1, const T * b0, const T * b1);

template <typename T>
bool segment_cross_segment (const T * a0, const T * a1, const T * b0, const T * b1);

template <typename T>
bool segment_cross_triangle(const T * v1, const T * v2, const T * t1, const T * t2, const T * t3);

std::pair<int, int> getOppositeEdge(int vertexIndex);

template <typename T>
intersectionResult tri_intersect_tri(const T * t00, const T * t01, const T * t02, const T * t10, const T * t11, const T * t12);


#endif //TRIANGLE_INTERSECTIONS_TRI_TRI_INTERSECT_H
