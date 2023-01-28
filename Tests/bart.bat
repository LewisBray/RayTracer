@ECHO OFF

clang kd_tree_tests.cpp -o kd_tree_tests.exe -std=c++1z -g --for-linker=/INCREMENTAL:NO
kd_tree_tests.exe
