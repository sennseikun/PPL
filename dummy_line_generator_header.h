#define TESTDLLSORT_API __declspec(dllexport)

extern "C" {
	TESTDLLSORT_API void generateDummyLine(double*point, int n);
}
