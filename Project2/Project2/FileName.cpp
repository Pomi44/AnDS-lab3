#include <iostream>
#include <random>
#include <bitset>

std::string generateRandomBinarySequence(int size) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 1);

    std::string binarySequence;
    binarySequence.reserve(size);

    for (int i = 0; i < size; ++i) {
        binarySequence.push_back('0' + dis(gen));
    }

    return binarySequence;
}

int main() {
    int size;
    std::cout << "������� ������ ������������������: ";
    std::cin >> size;

    std::string randomBinarySequence = generateRandomBinarySequence(size);

    std::cout << "��������� ������������������ �������� �����: " << randomBinarySequence << std::endl;

    return 0;
}