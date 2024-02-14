#include <iostream>

int Suma(int num) {
	int sum = 0;
	for(int i = 1; i <= num; i++){
		sum += i;
	}
	return sum;
}

int main() {
	int num;
	std::cout << "Mete un número";
	std::cin >> num;
	
	
	while(num < 1){
			std::cout << "Mete un número";
			std::cin >> num;
		}

	int suma = Suma(num);
	std::cout << "La suma desde 1 de " << num << "es" << suma << std::endl;

	return 0;
}
