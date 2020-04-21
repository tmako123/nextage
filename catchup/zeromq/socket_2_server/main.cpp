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
		std::cout << "push enter" << std::endl;
		char str = getchar();
		std::string st(&str);
		if (st == std::string("q")) break;
 
		zmq::message_t request(st.data(), st.size());
		socket.send(request);

		zmq::message_t reply;
		socket.recv(reply);
		std::string rpl(std::string(static_cast<char*>(reply.data()), reply.size()));
		std::cout << "Rev : " << rpl << std::endl;
	}

	return 0;
}