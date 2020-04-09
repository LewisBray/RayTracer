tests: maths_tests ray_tracing_tests

maths_tests: ../Source/maths.o test_utils.o maths_tests.o
	clang ../Source/maths.o test_utils.o maths_tests.o -o maths_tests -std=c++1z -g -lm -lstdc++

ray_tracing_tests: ../Source/maths.o ../Source/ray_tracing.o test_utils.o ray_tracing_tests.o
	clang ../Source/maths.o ../Source/ray_tracing.o test_utils.o ray_tracing_tests.o -o ray_tracing_tests -g -lm -lstdc++

../Source/maths.o: ../Source/maths.h ../Source/maths.cpp
	clang -c ../Source/maths.cpp -o ../Source/maths.o -std=c++1z -g

../Source/ray_tracing.o: ../Source/ray_tracing.h ../Source/ray_tracing.cpp
	clang -c ../Source/ray_tracing.cpp -o ../Source/ray_tracing.o -std=c++1z -g

test_utils.o: test_utils.h test_utils.cpp
	clang -c test_utils.cpp -std=c++1z -g

maths_tests.o: maths_tests.cpp
	clang -c maths_tests.cpp -std=c++1z -g

ray_tracing_tests.o: ray_tracing_tests.cpp
	clang -c ray_tracing_tests.cpp -std=c++1z -g

clean:
	rm *.o maths_tests ray_tracing_tests
