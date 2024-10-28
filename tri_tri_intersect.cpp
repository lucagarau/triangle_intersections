#include "tri_tri_intersect.h"

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//Simple intersections
//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

/**
 * @brief Check if two points are coincident
 *
 * @tparam T accept double, CGAL::Gmpq, genericPoint
 * @param p0 first point
 * @param p1 second point
 * @return if the points are coincident
 */
template bool points_are_coincident <double>(const double * p0, const double * p1);
template bool points_are_coincident <CGAL::Gmpq>(const CGAL::Gmpq * p0, const CGAL::Gmpq * p1);
template bool points_are_coincident <genericPoint> (const genericPoint * p0, const genericPoint * p1);

template <typename T>
bool points_are_coincident(const T * p0, const T * p1){
    using ElementType = typename std::remove_extent<T>::type;

    if constexpr (std::is_same<ElementType, genericPoint >::value){
        return genericPoint::coincident(*p0, *p1);
    } else {
        return p0[0] == p1[0] && p0[1] == p1[1] && p0[2] == p1[2];
    }
}


template bool points_are_colinear_3d <double>(const double * p0, const double * p1, const double * p2);
template bool points_are_colinear_3d <CGAL::Gmpq> (const CGAL::Gmpq * p0,const CGAL::Gmpq * p1,const CGAL::Gmpq * p2);

template <typename T>
bool points_are_colinear_3d(const T * p0,
                            const T * p1,
                            const T * p2)
{
    T p0_dropX[2] = { p0[1], p0[2] };
    T p0_dropY[2] = { p0[0], p0[2] };
    T p0_dropZ[2] = { p0[0], p0[1] };
    T p1_dropX[2] = { p1[1], p1[2] };
    T p1_dropY[2] = { p1[0], p1[2] };
    T p1_dropZ[2] = { p1[0], p1[1] };
    T p2_dropX[2] = { p2[1], p2[2] };
    T p2_dropY[2] = { p2[0], p2[2] };
    T p2_dropZ[2] = { p2[0], p2[1] };

    // check if all the 2d orthogonal projections of p are colinear
    if(orient2dT(p0_dropX, p1_dropX, p2_dropX) == 0 &&
       orient2dT(p0_dropY, p1_dropY, p2_dropY) == 0 &&
       orient2dT(p0_dropZ, p1_dropZ, p2_dropZ) == 0)
    {
        return true;
    }
    return false;
}


/**
 * @brief Check if a point is in a segment
 *
 * @tparam T accept double, CGAL::Gmpq, genericPoint
 * @param p query point
 * @param v0 segment first vertex
 * @param v1 segment second vertex
 * @return if the point is in the segment
 */

template bool point_in_inner_segment <double>(const double * p, const double * v0, const double * v1);
template bool point_in_inner_segment <CGAL::Gmpq>(const CGAL::Gmpq * p, const CGAL::Gmpq * v0, const CGAL::Gmpq * v1);
template bool point_in_inner_segment <genericPoint>(const genericPoint * p, const genericPoint * v0, const genericPoint * v1);

template <typename T>
bool point_in_inner_segment(const T * p, const T * v0, const T * v1){

    using ElementType = typename std::remove_extent<T>::type;

    if (points_are_coincident(p,v0)||points_are_coincident(p,v1)) return false;


    if constexpr (std::is_same<ElementType, genericPoint >::value){
        return genericPoint::pointInInnerSegment(*p, *v0, *v1);
    } else {

        if(!points_are_colinear_3d(v0,v1,p)) return false;

        if((p[0] > std::min(v0[0], v1[0]) && p[0] < std::max(v0[0], v1[0])) ||
           (p[1] > std::min(v0[1], v1[1]) && p[1] < std::max(v0[1], v1[1])) ||
           (p[2] > std::min(v0[2], v1[2]) && p[2] < std::max(v0[2], v1[2])))
        {
            return true;
        }
        return false;

    }
}

/**
 * @brief Check if a point is in a triangle
 *
 * @tparam T accept double, CGAL::Gmpq, genericPoint
 * @param p query point
 * @param a triangle first vertex
 * @param b triangle second vertex
 * @param c triangle third vertex
 * @return if the point is in the triangle
 */

template bool point_in_triangle_2d<double>(const double * p,const double * t0,const double * t1,const double * t2);
template bool point_in_triangle_2d<CGAL::Gmpq>(const CGAL::Gmpq * p,const CGAL::Gmpq * t0,const CGAL::Gmpq * t1,const CGAL::Gmpq * t2);

template <typename T>
bool point_in_triangle_2d(const T * p,const T * t0,const T * t1,const T * t2)
{

    double e0p_area = orient2dT(t0, t1, p);
    double e1p_area = orient2dT(t1, t2, p);
    double e2p_area = orient2dT(t2, t0, p);

    return (e0p_area >= 0 && e1p_area >= 0 && e2p_area >= 0) ||
               (e0p_area <= 0 && e1p_area <= 0 && e2p_area <= 0);

    /*if(hit)
    {
        if(e0p_area == 0) return false;
        if(e1p_area == 0) return false;
        if(e2p_area == 0) return false;

        return true;
    }

    return false;*/
}

template bool point_in_inner_triangle <double>(const double * p, const double * v0, const double * v1, const double * v2);
template bool point_in_inner_triangle <CGAL::Gmpq>(const CGAL::Gmpq * p, const CGAL::Gmpq * v0, const CGAL::Gmpq * v1, const CGAL::Gmpq * v2);
template bool point_in_inner_triangle <genericPoint>(const genericPoint * p, const genericPoint * v0, const genericPoint * v1, const genericPoint * v2);

template <typename T>
bool point_in_inner_triangle(const T * p, const T * v0, const T * v1, const T * v2){

    if (orient3dT(p, v0, v1, v2) != 0) return false;

    using ElementType = typename std::remove_extent<T>::type;

    if constexpr (std::is_same<ElementType, genericPoint >::value){
        return genericPoint::pointInInnerTriangle(*p, *v0, *v1, *v2);
    } else {

        if(point_in_inner_segment(p, v0, v1) || point_in_inner_segment(p, v1, v2) || point_in_inner_segment(p, v2, v0))
            return false;

        T p_dropX[2]  = { p[1],  p[2]};
        T t0_dropX[2] = {v0[1], v0[2]}, t1_dropX[2] = {v1[1], v1[2]}, t2_dropX[2] = {v2[1], v2[2]};

        if(point_in_triangle_2d(p_dropX, t0_dropX, t1_dropX, t2_dropX) == false)
            return false;

        T p_dropY[2]  = { p[0],  p[2]};
        T t0_dropY[2] = {v0[0], v0[2]}, t1_dropY[2] = {v1[0], v1[2]}, t2_dropY[2] = {v2[0], v2[2]};

        if(!point_in_triangle_2d(p_dropY, t0_dropY, t1_dropY, t2_dropY) )
            return false;

        T p_dropZ[2]  = { p[0],  p[1]};
        T t0_dropZ[2] = {v0[0], v0[1]}, t1_dropZ[2] = {v1[0], v1[1]}, t2_dropZ[2] = {v2[0], v2[1]};

        if(!point_in_triangle_2d(p_dropZ, t0_dropZ, t1_dropZ, t2_dropZ))
            return false;

        return true;
    }
}



/**
 * @brief Check if two segments intersect
 *
 * @tparam T accept double, CGAL::Gmpq, genericPoint
 * @param a0 first segment first vertex
 * @param a2 first segment second vertex
 * @param b0 second segment first vertex
 * @param b1 second segment second vertex
 * @return if the segments intersect
 */

template bool segment_cross_segment_2D <double>(const double * a0, const double * a1, const double * b0, const double * b1);
template bool segment_cross_segment_2D <CGAL::Gmpq>(const CGAL::Gmpq * a0, const CGAL::Gmpq * a1, const CGAL::Gmpq * b0, const CGAL::Gmpq * b1);
template bool segment_cross_segment_2D <genericPoint>(const genericPoint * a0, const genericPoint * a1, const genericPoint * b0, const genericPoint * b1);

template <typename T>
bool segment_cross_segment_2D (const T * s00, const T * s01, const T * s10, const T * s11){

    //check if the segments intersect
    using ElementType = typename std::remove_extent<T>::type;
    if constexpr (std::is_same<ElementType, genericPoint >::value){
        return genericPoint::segmentsCross(*s00, *s01, *s10, *s11);
    } else {
        // https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
        double det_s00 = orient2dT(s10, s11, s00);
        double det_s01 = orient2dT(s10, s11, s01);
        double det_s10 = orient2dT(s00, s01, s10);
        double det_s11 = orient2dT(s00, s01, s11);

        // Shewchuk's orient predicates return a rough approximation of the determinant.
        // I am converting values to { -1, 0, 1 } for a simpler check of intersection cases
        int s00_wrt_s1 = (det_s00 > 0) ? 1 : ((det_s00 < 0) ? -1 : 0);
        int s01_wrt_s1 = (det_s01 > 0) ? 1 : ((det_s01 < 0) ? -1 : 0);
        int s10_wrt_s0 = (det_s10 > 0) ? 1 : ((det_s10 < 0) ? -1 : 0);
        int s11_wrt_s0 = (det_s11 > 0) ? 1 : ((det_s11 < 0) ? -1 : 0);

        // segments intersect at a single point
        if(s00_wrt_s1 != s01_wrt_s1 && s10_wrt_s0 != s11_wrt_s0)
        {
                      // at least one segment endpoint is involved in the intersection
            return false;
        }

        // degenerate case: colinear segments
        if(s00_wrt_s1 == 0 && s01_wrt_s1 == 0 && s10_wrt_s0 == 0 && s11_wrt_s0 == 0)
        {
            // coincident segments
            if(points_are_coincident(s00, s10) && points_are_coincident(s01, s11) || points_are_coincident(s00, s11) && points_are_coincident(s01, s10))
            {
                return false;
            }


            T Xmin_s1 = std::min(s10[0], s11[0]);
            T Xmax_s1 = std::max(s10[0], s11[0]);
            T Ymin_s1 = std::min(s10[1], s11[1]);
            T Ymax_s1 = std::max(s10[1], s11[1]);
            T Xmin_s0 = std::min(s00[0], s01[0]);
            T Xmax_s0 = std::max(s00[0], s01[0]);
            T Ymin_s0 = std::min(s00[1], s01[1]);
            T Ymax_s0 = std::max(s00[1], s01[1]);

            if(// test s0 endpoints against s1 range
                    (s00[0] > Xmin_s1 && s00[0] < Xmax_s1) ||
                    (s00[1] > Ymin_s1 && s00[1] < Ymax_s1) ||
                    (s01[0] > Xmin_s1 && s01[0] < Xmax_s1) ||
                    (s01[1] > Ymin_s1 && s01[1] < Ymax_s1) ||
                    // test s1 endpoints against s0 range
                    (s10[0] > Xmin_s0 && s10[0] < Xmax_s0) ||
                    (s10[1] > Ymin_s0 && s10[1] < Ymax_s0) ||
                    (s11[0] > Xmin_s0 && s11[0] < Xmax_s0) ||
                    (s11[1] > Ymin_s0 && s11[1] < Ymax_s0))
            {
                return true;
            }
        }
        return false;
    }
}

template bool segment_cross_segment <double>(const double * a0, const double * a1, const double * b0, const double * b1);
template bool segment_cross_segment <CGAL::Gmpq>(const CGAL::Gmpq * a0, const CGAL::Gmpq * a1, const CGAL::Gmpq * b0, const CGAL::Gmpq * b1);
template bool segment_cross_segment <genericPoint>(const genericPoint * a0, const genericPoint * a1, const genericPoint * b0, const genericPoint * b1);

//todo: rimettere la versione segment 3d
template <typename T>
bool segment_cross_segment (const T * s00, const T * s01, const T * s10, const T * s11){

    //check if the segments intersect
    using ElementType = typename std::remove_extent<T>::type;
    if constexpr (std::is_same<ElementType, genericPoint >::value){
        return genericPoint::segmentsCross(*s00, *s01, *s10, *s11);
    } else {
        // Direzione segmento 1
        T u[3] = {s01[0] - s00[0], s01[1] - s00[1], s01[2] - s00[2]};
        // Direzione segmento 2
        T v[3] = {s11[0] - s10[0], s11[1] - s10[1], s11[2] - s10[2]};
        // Vettore tra gli estremi iniziali
        T w[3] = {s00[0] - s10[0], s00[1] - s10[1], s00[2] - s10[2]};

        // Calcolo dei prodotti misti
        T a = u[0] * u[0] + u[1] * u[1] + u[2] * u[2]; // ||u||^2
        T b = u[0] * v[0] + u[1] * v[1] + u[2] * v[2]; // u·v
        T c = v[0] * v[0] + v[1] * v[1] + v[2] * v[2]; // ||v||^2
        T d = u[0] * w[0] + u[1] * w[1] + u[2] * w[2]; // u·w
        T e = v[0] * w[0] + v[1] * w[1] + v[2] * w[2]; // v·w

        T denominator = a * c - b * b;

        // Se il denominatore è zero, i segmenti sono paralleli
        if (denominator == T(0)) {
            return false; // Segmenti paralleli o coincidenti
        }

        // Risolviamo per s e t (parametri delle due rette)
        T s = (b * e - c * d) / denominator;
        T t = (a * e - b * d) / denominator;

        // Verifica se i parametri s e t sono entro l'intervallo [0, 1]
        if (s <= T(0) || s >= T(1) || t <= T(0) || t >= T(1)) {
            return false; // I segmenti non si intersecano
        }

        // Calcoliamo i punti di intersezione lungo i segmenti
        T intersectionP[3] = {s00[0] + s * u[0], s00[1] + s * u[1], s00[2] + s * u[2]};
        T intersectionQ[3] = {s10[0] + t * v[0], s10[1] + t * v[1], s10[2] + t * v[2]};

        // Controlliamo se i punti di intersezione sono uguali (con una certa tolleranza per double)
        if constexpr (std::is_same<ElementType, double>::value) {
            if (std::fabs(intersectionP[0] - intersectionQ[0]) < 1e-6 &&
                std::fabs(intersectionP[1] - intersectionQ[1]) < 1e-6 &&
                std::fabs(intersectionP[2] - intersectionQ[2]) < 1e-6) {
                return true; // I segmenti si intersecano
            }
        } else {
            // Se T è CGAL::Gmpq, confrontiamo esattamente
            if (intersectionP[0] == intersectionQ[0] &&
                intersectionP[1] == intersectionQ[1] &&
                intersectionP[2] == intersectionQ[2]) {
                return true; // I segmenti si intersecano
            }
        }

        return false;
    }
}
/*bool segment_cross_segment (const T * s00, const T * s01, const T * s10, const T * s11){
    if(orient3dT(s00,s01,s10,s11) == 0 || orient3dT(s00,s01,s11,s10) == 0) return false;

    if(points_are_coincident(s00, s10) || points_are_coincident(s00, s11) || points_are_coincident(s01, s10) || points_are_coincident(s01, s11)) return false;

    if(point_in_inner_segment(s00, s10, s11) || point_in_inner_segment(s01, s10, s11)) return false;
    if(point_in_inner_segment(s10, s00, s01) || point_in_inner_segment(s11, s00, s01)) return false;

    // check 2D projections of the segments

    T s00_dropX[2] = {s00[1], s00[2]}, s01_dropX[2] = {s01[1], s01[2]};
    T s10_dropX[2] = {s10[1], s10[2]}, s11_dropX[2] = {s11[1], s11[2]};
    bool x_res = segment_cross_segment_2D(s00_dropX, s01_dropX, s10_dropX, s11_dropX);
    if (!x_res) return false;

    T s00_dropY[2] = {s00[0], s00[2]}, s01_dropY[2] = {s01[0], s01[2]};
    T s10_dropY[2] = {s10[0], s10[2]}, s11_dropY[2] = {s11[0], s11[2]};
    bool y_res = segment_cross_segment_2D(s00_dropY, s01_dropY, s10_dropY, s11_dropY);
    if (!y_res) return false;

    T s00_dropZ[2] = {s00[0], s00[1]}, s01_dropZ[2] = {s01[0], s01[1]};
    T s10_dropZ[2] = {s10[0], s10[1]}, s11_dropZ[2] = {s11[0], s11[1]};
    bool z_res = segment_cross_segment_2D(s00_dropZ, s01_dropZ, s10_dropZ, s11_dropZ);
    if (!z_res) return false;

    return true;
}
*/
/**
 * @brief Check if a segment intersects a triangle
 *
 * @tparam T accept double, CGAL::Gmpq, genericPoint
 * @param v0 segment first vertex
 * @param v1 segment second vertex
 * @param t1 triangle first vertex
 * @param t2 triangle second vertex
 * @param t3 triangle third vertex
 * @return if the segment intersects the triangle
 */

template bool segment_cross_triangle <double>(const double * v0, const double * v1, const double * t1, const double * t2, const double * t3);
template bool segment_cross_triangle <CGAL::Gmpq>(const CGAL::Gmpq * v0, const CGAL::Gmpq * v1, const CGAL::Gmpq * t1, const CGAL::Gmpq * t2, const CGAL::Gmpq * t3);
template bool segment_cross_triangle <genericPoint>(const genericPoint * v0, const genericPoint * v1, const genericPoint * t1, const genericPoint * t2, const genericPoint * t3);

template <typename T>
bool segment_cross_triangle(const T * v0, const T * v1, const T * t1, const T * t2, const T * t3){

    if(orient3dT(v0,t1,t2,t3) * orient3dT(v1,t1,t2,t3) > 0) return false;

    //check if segment cross triangle
    using ElementType = typename std::remove_extent<T>::type;
    if constexpr (std::is_same<ElementType, genericPoint >::value){
        return genericPoint::innerSegmentCrossesInnerTriangle(*v0, *v1, *t1, *t2, *t3);
    } else {
        if(points_are_coincident(v0, t1) || points_are_coincident(v0, t2) || points_are_coincident(v0, t3) ||
           points_are_coincident(v1, t1) || points_are_coincident(v1, t2) || points_are_coincident(v1, t3))
            return false;

        if(point_in_inner_segment(t1, v0, v1) || point_in_inner_segment(t2, v0, v1) || point_in_inner_segment(t3, v0, v1))
            return false;

        if(point_in_inner_segment(v0, t1, t2) || point_in_inner_segment(v0, t2, t3) || point_in_inner_segment(v0, t3, t1))
            return false;

        if(point_in_inner_segment(v1, t1, t2) || point_in_inner_segment(v1, t2, t3) || point_in_inner_segment(v1, t3, t1))
            return false;

        if(segment_cross_segment(v0, v1, t1, t2) || segment_cross_segment(v0, v1, t2, t3) || segment_cross_segment(v0, v1, t3, t1))
            return false;

        if(point_in_inner_triangle(v0, t1, t2, t3) || point_in_inner_triangle(v1, t1, t2, t3))
            return false;

        int vol_s_t01 = orient3dT(v0, v1, t1, t2);
        int vol_s_t12 = orient3dT(v0, v1, t2, t3);
        int vol_s_t20 = orient3dT(v0, v1, t3, t1);

        if((vol_s_t01 > 0 && vol_s_t12 < 0) || (vol_s_t01 < 0 && vol_s_t12 > 0)) return false;
        if((vol_s_t12 > 0 && vol_s_t20 < 0) || (vol_s_t12 < 0 && vol_s_t20 > 0)) return false;
        if((vol_s_t20 > 0 && vol_s_t01 < 0) || (vol_s_t20 < 0 && vol_s_t01 > 0)) return false;

        return true;
    }
}

std::pair<int, int> getOppositeEdge(int vertexIndex) {

    int v1 = (vertexIndex + 1) % 3;  // Primo vertice dell'edge opposto
    int v2 = (vertexIndex + 2) % 3;  // Secondo vertice dell'edge opposto
    return {v1, v2};

}


/**
 * @brief Check if two triangles intersect
 *
 * @tparam T accept double, CGAL::Gmpq, genericPoint
 * @param t00 first triangle first vertex
 * @param t01 first triangle second vertex
 * @param t02 first triangle third vertex
 * @param t10 second triangle first vertex
 * @param t11 second triangle second vertex
 * @param t12 second triangle third vertex
 * @return if the triangles intersect
 */


//todo: creare le versioni polimorifiche che accettano 2 vettori che rappresentano i triangoli o un unico vettore di punti

template intersectionResult tri_intersect_tri <double>(const double * t00, const double * t01, const double * t02, const double * t10, const double * t11, const double * t12);
//template intersectionResult tri_intersect_tri <CGAL::Gmpq>(const CGAL::Gmpq * t00, const CGAL::Gmpq * t01, const CGAL::Gmpq * t02, const CGAL::Gmpq * t10, const CGAL::Gmpq * t11, const CGAL::Gmpq * t12);
//template intersectionResult tri_intersect_tri <genericPoint>(const genericPoint * t00, const genericPoint * t01, const genericPoint * t02, const genericPoint * t10, const genericPoint * t11, const genericPoint * t12);

template <typename T>
intersectionResult tri_intersect_tri(const T * t00, const T * t01, const T * t02, const T * t10, const T * t11, const T * t12){


    intersectionResult intersection;

    std::array<const T*, 3> t0 = {t10, t11, t12};
    std::array<const T*, 3> t1 = {t00, t01, t02};


    int o_t0_t10 = orient3dT(t00,t01,t02,t10);
    int o_t0_t11 = orient3dT(t00,t01,t02,t11);
    int o_t0_t12 = orient3dT(t00,t01,t02,t12);

    std::vector<int> orient3DVector = {o_t0_t10, o_t0_t11, o_t0_t12};

    bool coplanar = count(orient3DVector.begin(), orient3DVector.end(), 0) == 3;
    //vector that contains the result of the check of the coincident points: row = t0 vertices, column = t1 vertices
    bool coincidentPoints[3][3] = {points_are_coincident(t00,t10), points_are_coincident(t00,t11), points_are_coincident(t00,t12),
                               points_are_coincident(t01,t10), points_are_coincident(t01,t11), points_are_coincident(t01,t12),
                               points_are_coincident(t02,t10), points_are_coincident(t02,t11), points_are_coincident(t02,t12)};

    bool t1VerticesCoplanar[3] = {o_t0_t10 == 0, o_t0_t11 == 0, o_t0_t12 == 0};

    //vector that contains the result of the check of the points in the inner segment: row = t1 vertices, column = t0 edges
    bool t1VerticesInT0Edges[3][3] = {point_in_inner_segment(t10,t00,t01), point_in_inner_segment(t11,t00,t01), point_in_inner_segment(t12,t00,t01),
                                        point_in_inner_segment(t10,t01,t02), point_in_inner_segment(t11,t01,t02), point_in_inner_segment(t12,t01,t02),
                                        point_in_inner_segment(t10,t02,t00), point_in_inner_segment(t11,t02,t00), point_in_inner_segment(t12,t02,t00)};

    bool t0VerticesInT1Edges[3][3] = {point_in_inner_segment(t00,t10,t11), point_in_inner_segment(t01,t10,t11), point_in_inner_segment(t02,t10,t11),
                                        point_in_inner_segment(t00,t11,t12), point_in_inner_segment(t01,t11,t12), point_in_inner_segment(t02,t11,t12),
                                        point_in_inner_segment(t00,t12,t10), point_in_inner_segment(t01,t12,t10), point_in_inner_segment(t02,t12,t10)};



    //vector that contains the result of the check of the points in the inner triangle: index = t1 vertex
    bool t1verticesInT0[3]= {point_in_inner_triangle(t10,t00,t01,t02), point_in_inner_triangle(t11,t00,t01,t02), point_in_inner_triangle(t12,t00,t01,t02)};

    //vector that contains the result of the check of the points in the inner triangle: index = t0 vertex
    bool t0verticesInT1[3]= {point_in_inner_triangle(t00,t10,t11,t12), point_in_inner_triangle(t01,t10,t11,t12), point_in_inner_triangle(t02,t10,t11,t12)};
    //vector that contains the result of the check of the segments intersection: row = t0 edges, column = t1 edges
    bool t0EdgesCrossT1Edges[3][3] = {segment_cross_segment(t00,t01,t10,t11), segment_cross_segment(t00,t01,t11,t12), segment_cross_segment(t00,t01,t12,t10),
                                        segment_cross_segment(t01,t02,t10,t11), segment_cross_segment(t01,t02,t11,t12), segment_cross_segment(t01,t02,t12,t10),
                                        segment_cross_segment(t02,t00,t10,t11), segment_cross_segment(t02,t00,t11,t12), segment_cross_segment(t02,t00,t12,t10)};


    //vector that contains the result of the check of the segments intersection with inner t0 triangle: index = t1 edge
    bool t1EdgesCrossT0InnerTriangle[3] = {segment_cross_triangle(t10,t11,t00,t01,t02), segment_cross_triangle(t11,t12,t00,t01,t02), segment_cross_triangle(t12,t10,t00,t01,t02)};
    bool t0EdgesCrossT1InnerTriangle[3] = {segment_cross_triangle(t00,t01,t10,t11,t12), segment_cross_triangle(t01,t02,t10,t11,t12), segment_cross_triangle(t02,t00,t10,t11,t12)};

    //All orient are positive or negative
    if(count(orient3DVector.begin(), orient3DVector.end(), 0) == 0 ){
        return intersection;
    }

    //check intersection
    if(coplanar){
        //todo: check if the triangles are coincident for coplanar triangles
    }
    else{
        //case 1: 1 orient3d == 0 and 2 orient3d with the same sign
        if((o_t0_t10 == 0 && o_t0_t11 * o_t0_t12 > 0) || (o_t0_t11 == 0 && o_t0_t10 * o_t0_t12 > 0) || (o_t0_t12 == 0 && o_t0_t10 * o_t0_t11 > 0)){
            //check if two vertices are coincident
            for(int i = 0; i < 3; i++){
                for(int j = 0; j < 3; j++){
                    if(coincidentPoints[i][j]){
                        intersection.addIntersectionPoint(VA_ON_VB, i, j);
                        return intersection;
                    }
                }
            }

            //check if a vertex of t1 is in the inner segment of t0
            for(int i = 0; i < 3; i++){
                for(int j = 0; j < 3; j++){
                    if(t1VerticesInT0Edges[j][i]){
                        intersection.addIntersectionPoint(VB_ON_EA, i,j);
                        return intersection;
                    }
                }
            }

            //check if a vertex of t1 is in the inner triangle of t0
            for(int i = 0; i < 3; i++){
                if(t1verticesInT0[i]){
                    intersection.addIntersectionPoint(VB_ON_TA, i, -1);
                    return intersection;
                }
            }

        }

        //case 2: 1 orient3d == 0 and 2 orient3d with the different sign
        if((o_t0_t10 == 0 && o_t0_t11 * o_t0_t12 < 0) || (o_t0_t11 == 0 && o_t0_t10 * o_t0_t12 < 0) || (o_t0_t12 == 0 && o_t0_t10 * o_t0_t11 < 0)){

            //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
            //t0 and t1 have a common vertex
            //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

            //t0 and t1 have a common vertex and the opposite edge of t1 intersects an edge of t0
            for(int i = 0; i < 3; i++){
                for(int j = 0; j < 3; j++){
                    if(coincidentPoints[i][j]){
                        std::pair<int, int> oppositeEdge = getOppositeEdge(j);
                        std::pair<int, int> oppositeEdgeT0 = getOppositeEdge(i);
                        std::pair<int, int> oppositeEdgeT1 = getOppositeEdge(j);

                        if(t0EdgesCrossT1Edges[i][oppositeEdge.first]){
                            intersection.addIntersectionPoint(VA_ON_VB, i, j);
                            intersection.addIntersectionPoint(EA_CROSS_EB, i, oppositeEdge.first);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }
                        if(t0EdgesCrossT1Edges[(i-1)%3][oppositeEdge.first]){
                            intersection.addIntersectionPoint(VA_ON_VB, (i-1)%3, j);
                            intersection.addIntersectionPoint(EA_CROSS_EB, (i-1)%3, oppositeEdge.first);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }

                        //t0 and t1 have a common vertex and the opposite edge of t1 intersects a vertex of t0

                        for(int v = 0;v<3;v++){
                            if(t0VerticesInT1Edges[oppositeEdge.first][v]){
                                intersection.addIntersectionPoint(VA_ON_VB, i, j);
                                intersection.addIntersectionPoint(VA_ON_EB,v,oppositeEdge.first);
                                intersection.addIntersectionEdge(0,1);
                                return intersection;
                            }
                        }

                        //t0 and t1 have a common vertex and another vertex of t1 is in the inner triangle of t0
                        if(t0verticesInT1[(i+1)%3]){
                            intersection.addIntersectionPoint(VA_ON_VB, i, j);
                            intersection.addIntersectionPoint(VA_ON_TB, (i+1)%3, -1);
                            intersection.addIntersectionEdge(0,1);

                            return intersection;
                        }
                        else if(t0verticesInT1[(i+2)%3]){
                            intersection.addIntersectionPoint(VA_ON_VB, i, j);
                            intersection.addIntersectionPoint(VA_ON_TB, (i+2)%3, -1);
                            intersection.addIntersectionEdge(0,1);

                            return intersection;
                        }

                        //t0 and t1 have a common vertex and the opposite edge of t1 is in the inner triangle of t0
                        if(t1EdgesCrossT0InnerTriangle[oppositeEdge.first]){
                            intersection.addIntersectionPoint(VA_ON_VB, i, j);
                            intersection.addIntersectionPoint(EB_CROSS_TA, oppositeEdge.first, -1);
                            intersection.addIntersectionEdge(0,1);

                            return intersection;
                        }

                        //t0 and t1 have a common vertex and the opposite edge of t0 crosses the opposite edge of t1
                        if(t0EdgesCrossT1Edges[oppositeEdgeT0.first][oppositeEdgeT1.first]){
                            intersection.addIntersectionPoint(VA_ON_VB, i, j);
                            intersection.addIntersectionPoint(EA_CROSS_EB, oppositeEdgeT0.first, oppositeEdgeT1.first);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }

                        //t0 and t1 have a common vertex and the opposite edge of t0 is in the inner triangle of t1
                        if(t0EdgesCrossT1InnerTriangle[oppositeEdge.first]){
                            intersection.addIntersectionPoint(VA_ON_VB, i, j);
                            intersection.addIntersectionPoint(EA_CROSS_TB, oppositeEdge.first, -1);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }

                        //t0 and t1 have only a common vertex
                        intersection.addIntersectionPoint(VA_ON_VB, i, j);
                        return intersection;

                    }
                }
            }


            //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
            //a vertex of t1 is on an edge of t0
            //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

            for(int e = 0; e < 3; e++){
                for(int j = 0; j < 3; j++){
                    //a t1 vertex and the opposite edge stay on the same edge of t0
                    if(t1VerticesInT0Edges[e][j]){
                        std::pair<int, int> oppositeEdge = getOppositeEdge(j);
                        std::pair<int, int> oppositeEdgeT1 = getOppositeEdge(j);

                        if(t0EdgesCrossT1Edges[e][oppositeEdge.first]){
                            intersection.addIntersectionPoint(VB_ON_EA, j, e);
                            intersection.addIntersectionPoint(EA_CROSS_EB, e, oppositeEdge.first);
                            intersection.addIntersectionEdge(0,1);
                            cout<< "Debug: case 8" << std::endl;
                            return intersection;
                        }

                        //a t1 vertex and the opposite edge touching a vertex of t0 in the same edge
                        if(t0VerticesInT1Edges[oppositeEdge.first][e]){
                            intersection.addIntersectionPoint(VB_ON_EA, j, e);
                            intersection.addIntersectionPoint(VA_ON_EB, e, oppositeEdge.first);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }
                        if(t0VerticesInT1Edges[oppositeEdge.first][(e+1)%3]){
                            intersection.addIntersectionPoint(VB_ON_EA, j, (e+1)%3);
                            intersection.addIntersectionPoint(VA_ON_EB, (e+1)%3, oppositeEdge.first);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }

                        //a t1 vertex stay on an edge of t0 and a vertex of t0 is in the inner triangle of t1
                        if(t0verticesInT1[e]){
                            intersection.addIntersectionPoint(VB_ON_EA, j, e);
                            intersection.addIntersectionPoint(VA_ON_TB, e, -1);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }
                        if(t0verticesInT1[(e+1)%3]){
                            intersection.addIntersectionPoint(VB_ON_EA, j, (e+1)%3);
                            intersection.addIntersectionPoint(VA_ON_TB, (e+1)%3, -1);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }

                        //a t1 vertex stay on an edge of t0 and the opposite edge of t1 is in the inner triangle of t0
                        if(t1EdgesCrossT0InnerTriangle[oppositeEdge.first]){
                            intersection.addIntersectionPoint(VB_ON_EA, j, e);
                            intersection.addIntersectionPoint(EB_CROSS_TA, oppositeEdge.first, -1);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }

                        //a t1 vertex stay on an edge of t0 and the opposite edge of t1 crosses an edge of t0
                        if(t0EdgesCrossT1Edges[(e+1)%3][oppositeEdgeT1.first]){
                            intersection.addIntersectionPoint(VB_ON_EA, j, e);
                            intersection.addIntersectionPoint(EA_CROSS_EB, (e+1)%3, oppositeEdgeT1.first);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }
                        if(t0EdgesCrossT1Edges[(e+2)%3][oppositeEdgeT1.first]){
                            intersection.addIntersectionPoint(VB_ON_EA, j, e);
                            intersection.addIntersectionPoint(EA_CROSS_EB, (e+2)%3, oppositeEdgeT1.first);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }

                        //a t1 vertex stay on an edge of t0 and an edge of t0 crosses t1
                        if(t0EdgesCrossT1InnerTriangle[(e+1)%3]){
                            intersection.addIntersectionPoint(VB_ON_EA, j, e);
                            intersection.addIntersectionPoint(EA_CROSS_TB, (e+1)%3, -1);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }
                        if(t0EdgesCrossT1InnerTriangle[(e+2)%3]){
                            intersection.addIntersectionPoint(VB_ON_EA, j, e);
                            intersection.addIntersectionPoint(EA_CROSS_TB, (e+2)%3, -1);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }

                        //a t1 vertex stay on an edge of t0 and the opposite t0 vertex crosses an edge of t1
                        if(t0VerticesInT1Edges[oppositeEdge.first][(e+2)%3]){
                            intersection.addIntersectionPoint(VB_ON_EA, j, e);
                            intersection.addIntersectionPoint(VA_ON_EB, (e+2)%3, oppositeEdge.first);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }

                        //a t1 vertex stay on an edge of t0 and the opposite t0 vertex is inside t1

                        if(t0verticesInT1[(e+2)%3]){
                            intersection.addIntersectionPoint(VB_ON_EA, j, e);
                            intersection.addIntersectionPoint(VA_ON_TB, (e+2)%3, -1);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }


                    }


                }
            }


            //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
            //a vertex of t1 is inside t0
            //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

            for(int v = 0; v<3; v++){
                if(t1verticesInT0[v]){
                    std::pair<int, int> oppositeEdge = getOppositeEdge(v);
                    for(int i=0; i< 3; i++){
                        //the opposite edge of t1 crosses an edge of t0
                        if(t0EdgesCrossT1Edges[i][oppositeEdge.first]){
                            intersection.addIntersectionPoint(VB_ON_TA, v, -1);
                            intersection.addIntersectionPoint(EA_CROSS_EB, i, oppositeEdge.first);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }

                        //an edge of t0 crosses t1
                        if(t0EdgesCrossT1InnerTriangle[i]){
                            intersection.addIntersectionPoint(VB_ON_TA, v, -1);
                            intersection.addIntersectionPoint(EA_CROSS_TB, i, -1);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }

                        //a vertex of t0 is in the opposite edge of t1
                        if (t0VerticesInT1Edges[oppositeEdge.first][i]){
                            intersection.addIntersectionPoint(VB_ON_TA, v, -1);
                            intersection.addIntersectionPoint(VA_ON_EB, i, oppositeEdge.first);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }

                        //a vertex of t0 is in the inner triangle of t1
                        if(t0verticesInT1[i]){
                            intersection.addIntersectionPoint(VB_ON_TA, v, -1);
                            intersection.addIntersectionPoint(VA_ON_TB, i, -1);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }
                    }


                }
            }

            //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
            //a vertex of t1 is coplanar, but outside t0
            //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
            for(int v = 0;v<3;v++){
                if(t1VerticesCoplanar[v]){
                    std::pair<int, int> oppositeEdget1 = getOppositeEdge(v);

                    //an edge of t0 crosses t1
                    for(int e = 0; e<3; e++){
                        if(t0EdgesCrossT1InnerTriangle[e]){
                            //the opposite edge of t1 touch a vertex of t0
                            for (int i = 0; i<3; i++){
                                if(t0VerticesInT1Edges[oppositeEdget1.first][i]){
                                    intersection.addIntersectionPoint(EA_CROSS_TB, e, -1);
                                    intersection.addIntersectionPoint(VA_ON_EB, i, oppositeEdget1.first);
                                    intersection.addIntersectionEdge(0,1);
                                    return intersection;
                                }
                            }

                            //the opposite edge of t1 crosses an edge of t0
                            for (int i = 0; i<3; i++){
                                if(t0EdgesCrossT1Edges[i][oppositeEdget1.first]){
                                    intersection.addIntersectionPoint(EA_CROSS_TB, e, -1);
                                    intersection.addIntersectionPoint(EA_CROSS_EB, i, oppositeEdget1.first);
                                    intersection.addIntersectionEdge(0,1);
                                    return intersection;
                                }
                            }

                            //the opposite edge of t1 is in the inner triangle of t0
                            if(t1EdgesCrossT0InnerTriangle[oppositeEdget1.first]){
                                intersection.addIntersectionPoint(EA_CROSS_TB, e, -1);
                                intersection.addIntersectionPoint(EB_CROSS_TA, oppositeEdget1.first, -1);
                                intersection.addIntersectionEdge(0,1);
                                return intersection;
                            }

                            //t0 has a vertex inside t1
                            for (int i = 0; i<3; i++){
                                if(t0verticesInT1[i]){
                                    intersection.addIntersectionPoint(EA_CROSS_TB, e, -1);
                                    intersection.addIntersectionPoint(VA_ON_TB, i, -1);
                                    intersection.addIntersectionEdge(0,1);
                                    return intersection;
                                }
                            }

                            //an edge of t0 crosses t1
                            for(int i = 0; i<3; i++){
                                if(i!=e && t0EdgesCrossT1InnerTriangle[i]){
                                    intersection.addIntersectionPoint(EA_CROSS_TB, e, -1);
                                    intersection.addIntersectionPoint(EA_CROSS_TB, i, -1);
                                    intersection.addIntersectionEdge(0,1);
                                    return intersection;
                                }
                            }

                        }
                        else{
                            //and edge of t0 crosses the opposite edge of t1
                            if(t0EdgesCrossT1Edges[e][oppositeEdget1.first]){
                                intersection.addIntersectionPoint(EA_CROSS_EB, e, oppositeEdget1.first);
                                return intersection;
                            }
                        }
                    }

                    //a vertex of t0 stay on an edge of t1
                    for(int i = 0; i<3; i++){
                        for (int j = 0; j<3; j++){
                            if(t0VerticesInT1Edges[j][i]){
                                intersection.addIntersectionPoint(VA_ON_EB, i, j);
                                return intersection;
                            }
                        }
                    }

                    //a vertex of t0 is inside t1
                    for (int i = 0; i<3; i++){
                        if(t0verticesInT1[i]){
                            intersection.addIntersectionPoint(VA_ON_TB, i, -1);
                            return intersection;
                        }
                    }


                }
            }


        }

        //case 3: 2 orient3d == 0 and 1 orient3d with the same sign
        if(count(orient3DVector.begin(),orient3DVector.end(),0)==2){
            //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
            //t0 and t1 have 1 common vertex
            //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
            for(int v0 = 0; v0<3;v0++){
                for(int v1 = 0; v1<3;v1++){
                    if (coincidentPoints[v0][v1]){
                        std::pair<int, int> oppositeEdgeT0 = getOppositeEdge(v0);
                        std::pair<int, int> oppositeEdgeT1 = getOppositeEdge(v1);

                        //an edge of t1 is partially overlapped by an edge of t0, t1 vertex is in the inner segment of t0
                        for(int e = 0; e<3; e++){
                            if(t1VerticesInT0Edges[e][(v1+1)%3]){
                                intersection.addIntersectionPoint(VA_ON_VB, v0, v1);
                                intersection.addIntersectionPoint(VB_ON_EA, (v1+1)%3, e);
                                intersection.addIntersectionEdge(0,1);
                                return intersection;
                            }
                            else if (t1VerticesInT0Edges[e][(v1+2)%3]){
                                intersection.addIntersectionPoint(VA_ON_VB, v0, v1);
                                intersection.addIntersectionPoint(VB_ON_EA, (v1+2)%3, e);
                                intersection.addIntersectionEdge(0,1);
                                return intersection;
                            }
                        }
                        //t0 and t1 have a common edge
                        for(int i = 0; i<3;i++){
                            if(i!=v0 && coincidentPoints[i][(v1+1)%3]){
                                intersection.addIntersectionPoint(VA_ON_VB, v0, v1);
                                intersection.addIntersectionPoint(VA_ON_VB, i, (v1+1)%3);
                                intersection.addIntersectionEdge(0,1);
                                return intersection;
                            }
                            if(i!=v0 && coincidentPoints[i][(v1+2)%3]){
                                intersection.addIntersectionPoint(VA_ON_VB, v0, v1);
                                intersection.addIntersectionPoint(VA_ON_VB, i, (v1+2)%3);
                                intersection.addIntersectionEdge(0,1);
                                return intersection;
                            }
                            //an edge of t1 is partially overlapped by an edge of t0, t0 vertex is in the inner segment of t1
                            if(i!=v0 && t0VerticesInT1Edges[v1][i]){
                                intersection.addIntersectionPoint(VA_ON_VB, v0, v1);
                                intersection.addIntersectionPoint(VA_ON_EB, i, v1);
                                intersection.addIntersectionEdge(0,1);
                                return intersection;
                            }
                            if(i!=v0 && t0VerticesInT1Edges[(v1+2)%3][i]){
                                intersection.addIntersectionPoint(VA_ON_VB, v0, v1);
                                intersection.addIntersectionPoint(VA_ON_EB, i, (v1+2)%3);
                                intersection.addIntersectionEdge(0,1);
                                return intersection;
                            }
                        }

                        //a t1 vertex is in the inner triangle of t0
                        for(int i = 0; i<3; i++){
                            if(i!=v1 && t1verticesInT0[i]){
                                intersection.addIntersectionPoint(VA_ON_VB, v0, v1);
                                intersection.addIntersectionPoint(VB_ON_TA, i, -1);
                                intersection.addIntersectionEdge(0,1);
                                return intersection;
                            }

                            //a vertex of t1 is in the opposite edge of t0
                            if(i!=v1 && t1VerticesInT0Edges[oppositeEdgeT0.first][i]){
                                intersection.addIntersectionPoint(VA_ON_VB, v0, v1);
                                intersection.addIntersectionPoint(VB_ON_EA, i, oppositeEdgeT0.first);
                                intersection.addIntersectionEdge(0,1);
                                return intersection;
                            }

                        }

                        //an edge of t1 crosses an edge of t0
                        if(t0EdgesCrossT1Edges[oppositeEdgeT0.first][v1]){
                            intersection.addIntersectionPoint(VA_ON_VB, v0, v1);
                            intersection.addIntersectionPoint(EA_CROSS_EB, oppositeEdgeT0.first, v1);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }
                        if(t0EdgesCrossT1Edges[oppositeEdgeT0.first][(v1+2)%3]){
                            intersection.addIntersectionPoint(VA_ON_VB, v0, v1);
                            intersection.addIntersectionPoint(EA_CROSS_EB, oppositeEdgeT0.first, (v1+2)%3);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }

                        //t0 and t1 have only a common vertex
                        intersection.addIntersectionPoint(VA_ON_VB, v0, v1);
                        return intersection;



                    }
                }
            }

            //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
            //a vertex of t1 is on an edge of t0
            //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
            for(int e = 0; e<3; e++){
                for(int v1 = 0; v1<3;v1++){
                    if(t1VerticesInT0Edges[e][v1]){
                        //an edge of t1 is partially overlapped by an edge of t0, t1 vertex is in the inner segment of t0
                        if(t1VerticesInT0Edges[e][(v1+1)%3]){
                            intersection.addIntersectionPoint(VB_ON_EA, v1, e);
                            intersection.addIntersectionPoint(VB_ON_EA, (v1+1)%3, e);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }
                        else if(t1VerticesInT0Edges[e][(v1+2)%3]){
                            intersection.addIntersectionPoint(VB_ON_EA, v1, e);
                            intersection.addIntersectionPoint(VB_ON_EA, (v1+2)%3, e);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }

                        //an edge of t1 is partially overlapped by an edge of t0, t0 vertex is in the inner segment of t1
                        for (int i = 0; i<3; i++){

                            if(i!=(e+2)%3 && t0VerticesInT1Edges[v1][i]){
                                intersection.addIntersectionPoint(VB_ON_EA, v1, e);
                                intersection.addIntersectionPoint(VA_ON_EB, i, v1);
                                intersection.addIntersectionEdge(0,1);
                                return intersection;
                            }
                            if(i!=(e+2)%3 &&t0VerticesInT1Edges[(v1+2)%3][i]){
                                intersection.addIntersectionPoint(VB_ON_EA, v1, e);
                                intersection.addIntersectionPoint(VA_ON_EB, i, (v1+2)%3);
                                intersection.addIntersectionEdge(0,1);
                                return intersection;
                            }
                        }

                        for (int i = 0; i<3; i++){
                            for(int j = 0; j<3; j++){
                                //an edge of t1 crosses an edge of t0
                                if(t0EdgesCrossT1Edges[i][j]){
                                    intersection.addIntersectionPoint(VB_ON_EA, v1, e);
                                    intersection.addIntersectionPoint(EA_CROSS_EB, i, j);
                                    intersection.addIntersectionEdge(0,1);
                                    return intersection;
                                }
                                //an edge of t1 crosses a vertex of t0
                                if(t0VerticesInT1Edges[j][i]){
                                    intersection.addIntersectionPoint(VB_ON_EA, v1, e);
                                    intersection.addIntersectionPoint(VA_ON_EB, i, j);
                                    intersection.addIntersectionEdge(0,1);
                                    return intersection;
                                }
                            }
                        }

                        //a vertex of t1 is in an edge of t0
                        for (int i = 0; i<3; i++){
                            for(int j = 0; j<3; j++){
                                if(t1VerticesInT0Edges[j][i] && j!=e && i!=v1){
                                    intersection.addIntersectionPoint(VB_ON_EA, v1, e);
                                    intersection.addIntersectionPoint(VB_ON_EA, i, j);
                                    intersection.addIntersectionEdge(0,1);
                                    return intersection;
                                }

                                //an edge of t1 crosses an edge of t0
                                if(t0EdgesCrossT1Edges[j][i]){
                                    intersection.addIntersectionPoint(VB_ON_EA, v1, e);
                                    intersection.addIntersectionPoint(EA_CROSS_EB, j, i);
                                    intersection.addIntersectionEdge(0,1);
                                    return intersection;
                                }
                            }
                        }
                        //a vertex of t1 is in the inner triangle of t0
                        for (int i = 0; i<3; i++){
                            if(t1verticesInT0[i] && i!=v1){
                                intersection.addIntersectionPoint(VB_ON_EA, v1, e);
                                intersection.addIntersectionPoint(VB_ON_TA, i, -1);
                                intersection.addIntersectionEdge(0,1);
                                return intersection;
                            }
                        }

                        //t1 have only a vertex in t0 edge
                        intersection.addIntersectionPoint(VB_ON_EA, v1, e);
                        return intersection;


                    }
                }
            }

            //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
            //a vertex of t1 is inside t0
            //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
            for(int v1 = 0; v1<3; v1++){
                if(t1verticesInT0[v1]){

                    //another vertex of t1 is inside t0
                    if(t1verticesInT0[(v1+1)%3]){
                        intersection.addIntersectionPoint(VB_ON_TA,v1,-1);
                        intersection.addIntersectionPoint(VB_ON_TA,(v1+1)%3,-1);
                        intersection.addIntersectionEdge(0,1);
                        return intersection;
                    }
                    if(t1verticesInT0[(v1+2)%3]){
                        intersection.addIntersectionPoint(VB_ON_TA,v1,-1);
                        intersection.addIntersectionPoint(VB_ON_TA,(v1+2)%3,-1);
                        intersection.addIntersectionEdge(0,1);
                        return intersection;
                    }

                    //an adjacent edge of t1 crosses an edge of t0
                    for(int e = 0; e<3; e++){
                        if(t0EdgesCrossT1Edges[e][v1]){
                            intersection.addIntersectionPoint(VB_ON_TA, v1, -1);
                            intersection.addIntersectionPoint(EA_CROSS_EB, e, v1);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }
                        if(t0EdgesCrossT1Edges[e][(v1+2)%3]){
                            intersection.addIntersectionPoint(VB_ON_TA, v1, -1);
                            intersection.addIntersectionPoint(EA_CROSS_EB, e, (v1+2)%3);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }
                    }

                }
            }

            //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
            //t0 is partially overlapped by t1 edge
            //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
            for(int v0 = 0; v0 < 3; v0++){
                for(int e = 0; e < 3; e++){
                    if (t0VerticesInT1Edges[e][v0]){
                        //an edge of t0 is partially overlapped by an edge of t1, t0 vertex is in the inner segment of t1
                        if(t0VerticesInT1Edges[e][(v0+1)%3]){
                            intersection.addIntersectionPoint(VA_ON_EB, v0, e);
                            intersection.addIntersectionPoint(VA_ON_EB, (v0+1)%3, e);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }
                        else if(t0VerticesInT1Edges[e][(v0+2)%3]){
                            intersection.addIntersectionPoint(VA_ON_EB, v0, e);
                            intersection.addIntersectionPoint(VA_ON_EB, (v0+2)%3, e);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }

                        //the opposite edge of t0 crosses an edge of t1
                        std::pair<int, int> oppositeEdgeT0 = getOppositeEdge(v0);
                        if(t0EdgesCrossT1Edges[oppositeEdgeT0.first][e]){
                            intersection.addIntersectionPoint(VA_ON_EB, v0, e);
                            intersection.addIntersectionPoint(EA_CROSS_EB, oppositeEdgeT0.first, e);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }

                    }
                }
            }

            //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
            //t1 edge crosses all t0
            //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
            for (int e1 = 0; e1<3; e1++){
                for (int e0 = 0; e0<3; e0++){
                    if(t0EdgesCrossT1Edges[e0][e1]){
                        if (t0EdgesCrossT1Edges[(e0+1)%3][e1]){
                            intersection.addIntersectionPoint(EA_CROSS_EB, e0, e1);
                            intersection.addIntersectionPoint(EA_CROSS_EB, (e0+1)%3, e1);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }
                        if (t0EdgesCrossT1Edges[(e0+2)%3][e1]){
                            intersection.addIntersectionPoint(EA_CROSS_EB, e0, e1);
                            intersection.addIntersectionPoint(EA_CROSS_EB, (e0+2)%3, e1);
                            intersection.addIntersectionEdge(0,1);
                            return intersection;
                        }
                    }
                }
            }

        }
    }


    return intersection;

}

