#include <zmq.hpp>
#include <string>
#include <iostream>
#include <thread>

#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>

#define sleep(n)    Sleep(n)
#endif

#include<sstream>
#include<fstream>

class posePublisher {
public:
	posePublisher():m_count(0){
		m_curData.resize(8, 0.f);
	}

	void poseSelector() {
		//load csv
		std::ifstream ifs("estPos.csv");
		std::string str;
		std::string line;
		while (getline(ifs, line)) {
			std::vector<float> strvec = split(line, ',');
			m_data.push_back(strvec);
		}

		while (true) {
			if (m_data.size() > ++m_count) {
			}
			else {
				m_count = 0;
			}
			auto& curData = m_data[m_count];

			for (int i = 0; i < 8; i++) {
				m_curData[i] = curData[i];
			}
			m_curData[0] = m_count;

			sleep(30);
		}

	}

	std::vector<float> split(std::string& input, char delimiter)
	{
		std::istringstream stream(input);
		std::string field;
		std::vector<float> result;
		while (getline(stream, field, delimiter)) {
			result.push_back(std::atof(field.c_str()));
		}
		return result;
	}

	void tcp_client() {
		zmq::context_t context(1);
		zmq::socket_t socket(context, ZMQ_REP);

		socket.connect("tcp://localhost:5555");

		while (true) {

			zmq::message_t request;

			socket.recv(request);
			std::string rpl(std::string(static_cast<char*>(request.data()), request.size()));
			sleep(1);

			std::string st;
			for (int i = 0; i < 8; i++) {
				st = st + std::to_string(m_curData[i]) + std::string(",");
			}
			std::cout << "Publish : " << st << std::endl;

			zmq::message_t reply(st.size());
			memcpy(reply.data(), st.data(), st.size());
			socket.send(reply);
		}
	}
	std::vector<float> m_curData;
	std::vector<std::vector<float>> m_data;
	int m_count;
};

int main()
{
	posePublisher pc;
	std::thread thr0(&posePublisher::poseSelector, &pc);
	std::thread thr1(&posePublisher::tcp_client, &pc);

	std::cout << "wayt for key" << std::endl;
	getchar();

	std::cout << "fin.." << std::endl;

	return 0;
}