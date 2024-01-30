#include <iostream>
#include <vector>
#include <math>

int discriminate(int a, int b, int c){
	int disc = b*b -4*a*c;
	return disc;

int main() {
        int a;
        int b;
        int c;
        std::cout << "Mete un coeficiente de x²";
        std::cin >> a;
	std::cout << "Mete un coeficiente de x¹";
        std::cin >> b;
        std::cout << "Mete un coeficiente de x⁰";
        std::cin >> c;
        
        int disc = discriminante(a,b,c);
        
	if(disc < 0){
		std::cout << "No existen soluciones reales";
	}else if(disc == 0){
		int sol1 = -b/2a;
		std::cout << "Existe una solución: " << sol1;
	}else{
		int raiz = std::sqrt(disc);
		int sol1 = -b+raiz/2a;
		int sol2 = -b-raiz/2a;
		std::cout << "Existen dos soluciones:" << sol1 << sol2;
	}
	return 0;

}

