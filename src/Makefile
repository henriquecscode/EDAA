all: attribute_type.h airport.cpp airport.h flight.h flight.cpp node.cpp node.h edge.cpp edge.h multigraph.h multigraph.cpp main.cpp test.cpp
	menu
	test

menu: attribute_type.h airport.cpp airport.h flight.h flight.cpp node.cpp node.h edge.cpp edge.h multigraph.h multigraph.cpp main.cpp
	g++ -Wall -w -g -o project.exe attribute_type.h airport.cpp airport.h flight.h flight.cpp node.cpp node.h edge.cpp edge.h multigraph.h multigraph.cpp main.cpp

test: attribute_type.h airport.cpp airport.h flight.h flight.cpp node.cpp node.h edge.cpp edge.h multigraph.h multigraph.cpp test.cpp
	g++ -Wall -w -g -o tester.exe attribute_type.h airport.cpp airport.h flight.h flight.cpp node.cpp node.h edge.cpp edge.h multigraph.h multigraph.cpp test.cpp