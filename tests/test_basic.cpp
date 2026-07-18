#include <cassert>
#include <numeric>
#include <vector>

int main() {
    std::vector<int> v{1, 2, 3, 4};
    assert(std::accumulate(v.begin(), v.end(), 0) == 10);
    return 0;
}
