#include <iostream>

int Suma-b(int a, int b) {
        int sum = 0;
        for(int i = a; i <= b; i++){
                sum += i;
        }
}

int main() {
        int a;
        int b;
        std::cout << "Mete un número de inicio";
        std::cin >> a;
	std::cout << "Mete un número de final";
        std::cin >> b;
	
        int suma = Suma_b(a,b);

        while(a >= 1 && b >= a){
                std::cout << "La suma desde " << a << "a" << b << "es" << suma <<            std::endl;
              
		std::cout << "Mete un número de inicio";
		std::cin >> a;
		std::cout << "Mete un número de final";
		std::cin >> b;
        }

        std::cout << "Número no válido";
        return 0;
}

