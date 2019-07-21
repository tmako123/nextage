#include <zmq.hpp>
#include <string>
#include <iostream>
#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>

#define sleep(n)    Sleep(n)
#endif

int main() {
	zmq::context_t context(1);
	zmq::socket_t socket(context, ZMQ_REQ);
	socket.bind("tcp://*:5555");

	while (true) {
		std::cout << "input words" << std::endl;
		char str[256];
		scanf("%s", &str);

		std::string st(str);
		if (st == std::string("q")) break;
 
		zmq::message_t request(st.data(), st.size());
		socket.send(request);

		zmq::message_t reply;
		socket.recv(reply);
	}

	return 0;
}