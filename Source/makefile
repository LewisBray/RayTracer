ray_tracer: maths.o ray_tracing.o input_parsing.o main.o
	clang maths.o ray_tracing.o input_parsing.o main.o \
		-o ray_tracer -std=c++1z -g -lm -lstdc++

maths.o: maths.h maths.cpp
	clang -c maths.cpp -std=c++1z -g

ray_tracing.o: ray_tracing.h ray_tracing.cpp
	clang -c ray_tracing.cpp -std=c++1z -g

input_parsing.o: input_parsing.h input_parsing.cpp
	clang -c input_parsing.cpp -std=c++1z -g

main.o: main.cpp
	clang -c main.cpp -std=c++1z -g

clean:
	rm *.o ray_tracer
