# Makefile used to build libraries
include ../Makefile.am

TARGETS=all clean
.PHONY: $(TARGETS)



RSWALK_LIB = -static -L/home/nao/testunsw/testunsw/rswalk2014/librswalk2014.a

INCLUDES=-I/home/nao/testunsw/testunsw/rswalk2014/

CXXFLAGS +=-DDEBUG

all: test_unsw

%.o: %.cpp %.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

test_unsw: test_unsw.o
	g++ $(RSWALK_LIB) $< -o $@


clean:
	rm -f *.$(SHLIBEXT) *.o
