#pragma once

#include <string>
#include <vector>

using Tstring = std::string;
using Tchar = char;

std::vector<std::string> getSerialList();

class Serial {
public:
	struct SerialConfig {
		unsigned int BaudRate;
		unsigned int ByteSize;
		unsigned int Parity;
		unsigned int StopBits;
	};

private:
	SerialConfig serialConfig;

	std::string port;

	bool opened;
	void* handle;
	void setConfig(const SerialConfig&);
	void setBufferSize(unsigned long read, unsigned long write);

public:
	Serial();
	Serial(const Serial&) = delete;
	~Serial();

	bool Open(const std::string port, const SerialConfig& config);
	void Close();
	bool IsOpened();

	std::vector<unsigned char> Read();

	int Write(const std::vector<unsigned char>& data);

	void Clear();
	void ClearWrite();
	void ClearRead();

};
