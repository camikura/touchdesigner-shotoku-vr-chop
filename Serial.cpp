#include "Serial.hpp"

#include <Windows.h>
#include <SetupAPI.h>
#pragma comment(lib, "setupapi.lib")

#define PATH "\\\\.\\"
#define PORTS "Ports"
#define PORTNAME "PortName"

void Serial::setBufferSize(unsigned long read, unsigned long write){
	SetupComm(handle, read, write);
}

Serial::Serial() {
	serialConfig = Serial::SerialConfig{ CBR_38400, 8, ODDPARITY, ONESTOPBIT };
	opened = false;
	handle = nullptr;
}

Serial::~Serial(){
	Close();
}

bool Serial::Open(const std::string port, const SerialConfig& config) {
	Tstring path = PATH + port;
	handle = CreateFile(
		path.c_str(),
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		NULL);
	if (handle == INVALID_HANDLE_VALUE) {
		opened = false;
		return false;
	}

	setConfig(config);
	setBufferSize(1024, 1024);

	opened = true;
	return true;
}

void Serial::Close(){
	if (opened) {
		CloseHandle(handle);
	}
	opened = false;
}

bool Serial::IsOpened() {
	return opened;
}

void Serial::setConfig(const SerialConfig& config){
	DCB dcb;
	GetCommState(handle, &dcb);

	dcb.BaudRate = config.BaudRate;
	dcb.ByteSize = (BYTE)config.ByteSize;
	dcb.Parity = config.Parity;
	dcb.fParity = (config.Parity != NOPARITY);
	dcb.StopBits = config.StopBits;
	SetCommState(handle, &dcb);
}

int available(void* handle) {
	unsigned long error;
	COMSTAT stat;
	ClearCommError(handle, &error, &stat);
	return stat.cbInQue;
}

std::vector<unsigned char> Serial::Read(){
	std::vector<unsigned char> vals;

	unsigned long readSize;
	int read_size = available(handle);
	if (read_size == 0) {
		read_size = 1;
	}
	unsigned char* data = new unsigned char[read_size];
	bool status = ReadFile(handle, data, read_size, &readSize, NULL);
	if (!status)
		return vals;

	for (int i = 0; i < read_size; i++) {
		vals.push_back(data[i]);
	}
	delete[] data;
	return vals;
}

void Serial::Clear(){
	PurgeComm(handle, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
}

void Serial::ClearWrite(){
	PurgeComm(handle, PURGE_TXABORT | PURGE_TXCLEAR);
}

void Serial::ClearRead(){
	PurgeComm(handle, PURGE_RXABORT | PURGE_RXCLEAR);
}

int Serial::Write(const std::vector<unsigned char>& data){
	unsigned long writtenSize;
	int size = data.size();
	char* buff = new char[size];
	for (int i = 0; i < size; i++) {
		buff[i] = data[i];
	}

	WriteFile(handle, buff, size, &writtenSize, NULL);
	return writtenSize;
}

std::vector<std::string> getSerialList() {
	std::vector<std::string> list;
	HDEVINFO hinfo = NULL;
	SP_DEVINFO_DATA info_data = { 0 };
	info_data.cbSize = sizeof(SP_DEVINFO_DATA);

	GUID guid;
	unsigned long guid_size = 0;
	if (SetupDiClassGuidsFromName(PORTS, &guid, 1, &guid_size) == FALSE)
		return list;

	hinfo = SetupDiGetClassDevs(&guid, 0, 0, DIGCF_PRESENT | DIGCF_PROFILE);
	if (hinfo == INVALID_HANDLE_VALUE)
		return list;

	Tchar buff[MAX_PATH];
	Tstring name;
	Tstring fullname;
	unsigned int index = 0;
	while (SetupDiEnumDeviceInfo(hinfo, index, &info_data)) {
		unsigned long type;
		unsigned long size;

		if (SetupDiGetDeviceRegistryProperty(
			hinfo, &info_data, SPDRP_DEVICEDESC, &type, (PBYTE)buff, MAX_PATH, &size)
			== TRUE) {
			fullname = buff;
		}

		HKEY hkey = SetupDiOpenDevRegKey(hinfo, &info_data, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
		if (hkey) {
			RegQueryValueEx(hkey, PORTNAME, 0, &type, (LPBYTE)buff, &size);
			RegCloseKey(hkey);
			name = buff;
		}
		list.push_back(name);
		index++;
	}
	SetupDiDestroyDeviceInfoList(hinfo);
	return list;

}
