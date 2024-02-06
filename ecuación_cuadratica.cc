#include <iostream>
#include <vector>
#include <cmath>

int discriminante(int a, int b, int c){
	int disc = b*b -4*a*c;
	return disc;
}

int main() {
        int a;
        int b;
        int c;
        std::cout << "Introduce un coeficiente de x²";
        std::cin >> a;
		std::cout << "Introduce un coeficiente de x¹";
        std::cin >> b;
        std::cout << "Introduce un coeficiente de x⁰";
        std::cin >> c;
        
        int disc = discriminante(a,b,c);

		if(disc >= 0){
            int raiz = std::sqrt(disc);
		int sol1 = -b+raiz/2*a;
		int sol2 = -b-raiz/2*a;
		std::cout << "Existen dos soluciones:" << sol1 << sol2;
        }
	
	return 0;
}

