// Saca primorial de un n
#include <iostream>
#include "misfunciones.hpp"

int primorial(int n) {
    int prim = 1;
    for (int i = 1; i <= n; i++) {
        if (misf::EsPrimo(i) == true) {
             prim *= i;
        }
    }
    return prim;
}

int main() {
    int n;
    int prim = 1;
    
    std::cout << "";
    std::cin >> n;
    
    prim = primorial(n);

    std::cout << n << "# = " << prim << std::endl;

    return 0;
}
