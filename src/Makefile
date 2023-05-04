all: menu test

menu: loader.h loader.cpp attribute_type.h airport.cpp airport.h flight.h flight.cpp node.cpp node.h edge.cpp edge.h multigraph.h multigraph.cpp main.cpp
	g++ -Wall -w -g -o project.exe loader.h loader.cpp attribute_type.h airport.cpp airport.h flight.h flight.cpp node.cpp node.h edge.cpp edge.h multigraph.h multigraph.cpp main.cpp

test: attribute_type.h loader.h loader.cpp airport.cpp airport.h flight.h flight.cpp node.cpp node.h edge.cpp edge.h multigraph.h multigraph.cpp test.cpp
	g++ -Wall -w -g -o tester.exe loader.h loader.cpp attribute_type.h airport.cpp airport.h flight.h flight.cpp node.cpp node.h edge.cpp edge.h multigraph.h multigraph.cpp test.cpp