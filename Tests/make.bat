@ECHO OFF

clang maths_tests.cpp -o maths_tests.exe -std=c++1z -g --for-linker=/INCREMENTAL:NO
clang ray_tracing_tests.cpp -o ray_tracing_tests.exe -std=c++1z -g --for-linker=/INCREMENTAL:NO
clang input_parsing_tests.cpp -o input_parsing_tests.exe -std=c++1z -g --for-linker=/INCREMENTAL:NO
