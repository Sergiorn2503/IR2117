// Saca primorial de un n
#include <iostream>


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


int main() {
    int n;
    int prim = 1;
    
    std::cout << "";
    std::cin >> n;
    
    if(EsPrimo(n)){
        std::cout << n << "Es primo";
    }else{
        std::cout << n << "No Es primo";
    }

    return 0;
}
