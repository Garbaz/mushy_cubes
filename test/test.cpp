#include <iostream>
#include <vector>

int main() {
    std::vector<int> v;
    int t = 42;
    v.push_back(t);
    std::cout << &v[0] << std::endl;
    std::cout << &v.back() << std::endl;
}
