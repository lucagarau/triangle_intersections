#include <iostream>
#include "tri_tri_intersect.h"
#include <implicit_point.h>
#undef Split
#include <cinolib/predicates.h>
#include <cinolib/drawable_triangle_soup.h>
#include <cinolib/gl/glcanvas.h>


using namespace cinolib;


int main() {

    double t1_0[3] = {0, 0, 0};
    double t1_1[3] = {2, 0, 0};
    double t1_2[3] = {0, 2, 0};

    double t2_0[3] = {1, 1.5, 0};
    double t2_1[3] = {0.5, -1, 0};
    double t2_2[3] = {0.5, 0.5, -2};

    std::vector<double> coords = {t1_0[0], t1_0[1], t1_0[2],
                                  t1_1[0], t1_1[1], t1_1[2],
                                  t1_2[0], t1_2[1], t1_2[2],
                                  t2_0[0], t2_0[1], t2_0[2],
                                  t2_1[0], t2_1[1], t2_1[2],
                                  t2_2[0], t2_2[1], t2_2[2]};
    std::vector<uint> tris = {0, 1, 2, 3, 4, 5};

    std::vector<cinolib::Color> colors = {cinolib::Color::PASTEL_CYAN(), cinolib::Color::PASTEL_ORANGE()};

    Marker m1;
    m1.pos_3d = vec3d(t1_0[0], t1_0[1], t1_0[2]);
    m1.text = "t00";

    Marker m2;
    m2.pos_3d = vec3d(t1_1[0], t1_1[1], t1_1[2]);
    m2.text = "t01";

    Marker m3;
    m3.pos_3d = vec3d (t1_2[0], t1_2[1], t1_2[2]);
    m3.text = "t02";

    Marker m4;
    m4.pos_3d = vec3d(t2_0[0], t2_0[1], t2_0[2]);
    m4.text = "t10";

    Marker m5;
    m5.pos_3d = vec3d(t2_1[0], t2_1[1], t2_1[2]);
    m5.text = "t11";

    Marker m6;
    m6.pos_3d = vec3d(t2_2[0], t2_2[1], t2_2[2]);
    m6.text = "t12";

    std::vector<Marker> markers = {m1, m2, m3, m4, m5, m6};



    DrawableTriangleSoup t(coords,tris,colors);

    GLcanvas gui;
    gui.push(&t);
    gui.markers = markers;


    intersectionResult result = tri_intersect_tri(t1_0, t1_1, t1_2, t2_0, t2_1, t2_2);
    result.printIntersectionPoints();
    result.printIntersectionEdges();



    return gui.launch();
}

