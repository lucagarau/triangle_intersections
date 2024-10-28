#include "orientTemplated.h"

template int orient3dT<double>(const double * pa, const double * pb, const double * pc, const double * pd);
template int orient3dT<CGAL::Gmpq>(const CGAL::Gmpq * pa, const CGAL::Gmpq * pb, const CGAL::Gmpq * pc, const CGAL::Gmpq * pd);
template int orient3dT<genericPoint>(const genericPoint * pa, const genericPoint * pb, const genericPoint * pc, const genericPoint * pd);

template int orient2dT<double>(const double * pa, const double * pb, const double * pc);
template int orient2dT<CGAL::Gmpq>(const CGAL::Gmpq * pa, const CGAL::Gmpq * pb, const CGAL::Gmpq * pc);
template int orient2dT<genericPoint>(const genericPoint * pa, const genericPoint * pb, const genericPoint * pc);

template <typename T>
int orient3dT( const T * pa, const T * pb, const T * pc, const T * pd) {

    using ElementType = typename std::remove_extent<T>::type;

    if constexpr (std::is_same<ElementType, double>::value) {
        //Double
        double result = cinolib::orient3d(pa, pb, pc, pd);
        if (result < 0) return -1;
        else if (result > 0) return 1;
        else return 0;

    } else if constexpr (std::is_same<ElementType, CGAL::Gmpq>::value) {
        //Rationals Gmpq from CGAL
        CGAL::Gmpq result = cinolib::orient3d(pa, pb, pc, pd);
        if (result < 0) return -1;
        else if (result > 0) return 1;
        else return 0;
    } else {
        return -1 *  genericPoint::orient3D(*pa, *pb, *pc, *pd);
    }
}

template <typename T>
int orient2dT( const T * pa, const T * pb, const T * pc) {

    using ElementType = typename std::remove_extent<T>::type;

    if constexpr (std::is_same<ElementType, double>::value) {
        //Double
        double result = cinolib::orient2d(pa, pb, pc);
        if (result < 0) return -1;
        else if (result > 0) return 1;
        else return 0;
    } else if constexpr (std::is_same<ElementType, CGAL::Gmpq>::value) {
        //Rationals Gmpq from CGAL
        CGAL::Gmpq result = cinolib::orient2d(pa, pb, pc);
        if (result < 0) return -1;
        else if (result > 0) return 1;
        else return 0;
    } else {
        return genericPoint::orient2D(*pa, *pb, *pc);
    }
}