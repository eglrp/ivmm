CC = clang
CXX = clang++
CXXFLAGS = -std=c++11  -fpic $(shell python-config --includes)
LIBS = -lboost_python -lshp
LIBTEST = -lboost_unit_test_framework -lshp
SOURCE = $(wildcard *.cpp)
OBJES = $(patsubst %.cpp, %.o, $(SOURCE))
DEBUG =
.PHONY:clean all cleanTest

ifeq ($(DEBUG),on)
CXXFLAGS += -g -DDEBUG
else
CXXFLAGS += -O2
endif


all:pyivmm.so test
	

pyivmm.so:$(OBJES)
	$(CXX) $(CXXFLAGS) -shared -o $@ $^  $(LIBS)

test:test.o road.o util.o network.o sample_generator.o ivmm.o evaluatoin.o
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LIBTEST)

modify:road.o util.o network.o modify.o
	$(CXX) $(CXXFLAGS) $^ -o $@ -lshp
modify.o:modify.cxx
	$(CXX) $(CXXFLAGS) -c $^ -o $@

cleanTest:
	rm -f *.shp *.dbf *.shx
clean:
	rm -f *.o *.so *.shp *.dbf *.shx
