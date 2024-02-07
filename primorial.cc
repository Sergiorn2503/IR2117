// Saca primorial de un n
#include <iostream>
#include "misfunciones.hpp"


bool EsPrimo(int n) {
    if(n < 2) {
        return false;
    } else {
        for(int i = 2; i < n; ++i) {
            if(n % i == 0) {
                return false;
            }
        }
        return true;
    }
}

int primorial(int n) {
    int prim = 1;
    for(int i = 1; i <= n; i++) {
        if(EsPrimo(i) == true) {
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
