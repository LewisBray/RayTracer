@ECHO OFF

clang main.cpp -o ray_tracer.exe -std=c++1z -g -Weverything -Wno-c++98-compat --for-linker=/INCREMENTAL:NO
