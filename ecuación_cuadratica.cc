#include <iostream>
#include <vector>
#include <cmath>
#include <complex>

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

        if(disc < 0){
		std::complex<float> raiz1(-b / (2*a), std::sqrt(-disc) / (2*a));
        std::complex<float> raiz2(-b / (2*a), -std::sqrt(-disc) / (2*a));
        std::cout << "Las soluciones complejas son: " << raiz1 << "y" << raiz2;

        }else if(disc == 0){
		int sol1 = -b/2*a;
		std::cout << "Existe una solución: " << sol1;

        }else{
            int raiz = std::sqrt(disc);
            int sol1 = -b+raiz/2*a;
            int sol2 = -b-raiz/2*a;
            std::cout << "Existen dos soluciones:" << sol1 << sol2;
        }
	
	return 0;
}

