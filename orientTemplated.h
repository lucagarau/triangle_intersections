// orientTemplated.h
#ifndef ORIENT_TEMPLATED_H
#define ORIENT_TEMPLATED_H

#include <cinolib/predicates.h>
#include <CGAL/Gmpq.h>  // Per il supporto CGAL::Gmpq, se usato
#include <type_traits>  // Per std::is_same
#include <implicit_point.h>
#include <cinolib/rationals.h>
#include <cinolib/geometry/vec_mat.h>

template <typename T>
int myorient3D(const T &pa, const T &pb, const T &pc, const T &pd) {

    //DEBUG
    cout << "Type: " << typeid(T).name() << endl;

    if constexpr (std::is_same<T, double>::value) {
        cout << "orient3D Double" << endl;
        return cinolib::orient3d(pa, pb, pc, pd);
    } else if constexpr (std::is_same<T, cinolib::vec3d>::value){
        cout << "orient3D vec3d" << endl;
        return cinolib::orient3d(pa, pb, pc, pd);
    } else if constexpr (std::is_same<T, cinolib::CGAL_Q>::value) {
        cout << "orient3D CGAL_Q" << endl;
        return cinolib::orient3d(pa, pb, pc, pd);
    } else if constexpr (std::is_same<T, CGAL::Gmpq>::value) {
        cout << "orient3D Gmpq" << endl;
        return cinolib::orient3d(pa, pb, pc, pd);
    } else if constexpr (std::is_same<T, genericPoint>::value) {
        cout << "orient3D genericPoint" << endl;
        return orient3d(pa, pb, pc, pd);
    } else {
        return -1;  // Tipo non gestito
    }
}

#endif // ORIENT_TEMPLATED_H
