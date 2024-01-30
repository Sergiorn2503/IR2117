#include <iostream>

int Suma(int num) {
	int sum = 0;
	for(int i = 1; i <= num; i++){
		sum += i;
	}
}

int main() {
	int num;
	std::cout << "Mete un número";
	std::cin >> num;
	
	int suma = Suma(num);
	
	while(num >= 1){
		std::cout << "La suma desde 1 de " << num << "es" << suma << 		std::endl;
		std::cout << "Mete un número";
		std::cin >> num;
		suma = Suma(num);
	}
	
	std::cout << "Número no válido";
	return 0;
}
