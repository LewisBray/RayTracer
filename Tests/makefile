tests: maths_tests ray_tracing_tests input_parsing_tests

maths_tests:
	clang maths_tests.cpp -o maths_tests -std=c++1z -g -lm -lstdc++

ray_tracing_tests:
	clang ray_tracing_tests.cpp -o ray_tracing_tests -std=c++1z -g -lm -lstdc++

input_parsing_tests:
	clang input_parsing_tests.cpp -o input_parsing_tests -std=c++1z -g -lm -lstdc++

clean:
	rm maths_tests ray_tracing_tests input_parsing_tests
