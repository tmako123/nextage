#include <zmq.hpp>
#include <string>
#include <iostream>
#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>

#define sleep(n)    Sleep(n)
#endif
#include <thread>

void tcp_server() {
	zmq::context_t context(1);
	zmq::socket_t socket(context, ZMQ_REP);
	socket.bind("tcp://*:5555");

	static int count = 0;
	while (true) {
		zmq::message_t request;

		socket.recv(request);
		std::string rpl(std::string(static_cast<char*>(request.data()), request.size()));
		std::cout << "Received Hello : " << count << " : " << rpl << std::endl;
		count++;
		sleep(1);

		zmq::message_t reply(8);
		memcpy(reply.data(), "Accepted", 8);
		socket.send(reply);
	}
}

int main() {
	std::thread thr0(tcp_server);

	std::cout << "wayt for key" << std::endl;
	getchar();

	std::cout << "fin.." << std::endl;

	return 0;
}