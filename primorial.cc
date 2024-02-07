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
    
    if(n == 0 || n == 1){
        int prim = 1;
        std::cout << n << "# = " << prim << std::endl;
    }

    return 0;
}
