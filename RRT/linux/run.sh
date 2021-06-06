g++ -c Tree.c -llz4 -fPIC
g++ -c Environment.c -llz4 -fPIC
g++ -c somain.c -llz4 -fPIC
g++ -shared -o RRT.so somain.o Tree.o Environment.o -llz4 -fPIC -lc -fvisibility=default
