#include <iostream>
#include <cinolib/predicates.h>
#include "orientTemplated.h"
#include <cinolib/geometry/vec_mat.h>

int main() {
    cout << " Creo il triangolo e il punto" << endl;
    cinolib::vec3d pa(0, 0, 0);
    cinolib::vec3d pb(1, 2, 3);
    cinolib::vec3d pc(9, 3, 1);
    cinolib::vec3d pd(3, 4, 5);

    cout << "Calcolo l'orientazione" << endl;

    //call the orient3D function with the 3 points
    int result = myorient3D(pa, pb, pc, pd);
    std::cout << "The result is: " << result << std::endl;
    return 0;
}

