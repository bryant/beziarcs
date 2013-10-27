CXX=/usr/bin/g++
CXXFLAGS=-std=c++11

%-test: %-test.o
	$(CXX) -o $@ $^ -lboost_unit_test_framework
	./$@

%-test.o: %.cpp
	$(CXX) -c -o $@ $(CXXFLAGS) -D UNIT_TEST $^

clean:
	rm -f *.o
