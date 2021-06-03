/* Shared Use License: This file is owned by Derivative Inc. (Derivative)
* and can only be used, and/or modified for use, in conjunction with
* Derivative's TouchDesigner software, and only if you are a licensee who has
* accepted Derivative's TouchDesigner license or assignment agreement
* (which also govern the use of this file). You may share or redistribute
* a modified version of this file provided the following conditions are met:
*
* 1. The shared file or redistribution must retain the information set out
* above and this list of conditions.
* 2. Derivative's name (Derivative Inc.) or its trademarks may not be used
* to endorse or promote products derived from this file without specific
* prior written permission from Derivative.
*/

#include "CPlusPlus_Common.h"
#include "CHOP_CPlusPlusBase.h"

#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <thread>
#include <chrono>
#include <algorithm>
#include <numeric>

#include "Serial.hpp"

using namespace std;

class ShotokuVRCHOP : public CHOP_CPlusPlusBase
{
public:
	std::string portname = "";
	int cameraid = 0;

	std::vector<std::string> chanNames{ "tx", "ty", "tz", "rx", "ry", "rz", "zoom", "focus", "fps", "fpsavg" };
	std::vector<double> chanValues{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

	std::vector<double> transform{ 0.0, 0.0, 0.0 };
	std::vector<double> rotate{ 0.0, 0.0, 0.0 };

	double zoom_max = 0.0;
	double zoom_min = 0.0;
	double focus_max = 0.0;
	double focus_min = 0.0;

	int last_sec = 0;
	int fps_counter = 0;
	time_t now = NULL;

	std::vector<double> fpsHistory{};

	std::thread recv_thread;
	bool running;

	Serial serial;

	ShotokuVRCHOP(const OP_NodeInfo* info)
	{
		this->running = false;
	}

	virtual ~ShotokuVRCHOP()
	{
		this->stop();
		this->close();
	}

	bool open()
	{
		// search device
		bool found = false;
		auto list = getSerialList();
		for (const auto p : list) {
			if (this->portname == p)
				found = true;
		}

		// device not found
		if (!found) {
			return false;
		}

		Serial::SerialConfig serialConfig = { CBR_38400, 8, ODDPARITY, ONESTOPBIT };

		// open error
		if (!this->serial.Open(this->portname, serialConfig)) {
			return false;
		}

		cout << "open success" << endl;

		return true;
	}

	void start()
	{
		std::cout << "thread start" << std::endl;

		assert(this->serial);
		recv_thread = std::thread([this]() {
			this->loop();
		});
	}

	void loop()
	{
		this->running = true;

		unsigned char data[29] = { 0 };
		int pos = 0;

		while (this->running)
		{
			auto v = this->serial.Read();
			for (auto c : v) {
				c &= 0xff;

				if (pos <= 0 && c == 0xd1) {
					data[0] = c;
					unsigned char data[29] = { 0 };
					pos = 1;
					continue;
				}

				if (pos > 0 && pos < 29) {
					data[pos++] = c;
				}

				if (pos == 29) {
					if (isValidData(data)) {
						this->handleData(data);
						unsigned char data[29] = { 0 };
						pos = 0;
					}
					else {
						pos = -1;
					}
				}
			}
		}
	}

	bool isValidData(unsigned char data[29])
	{
		if (data[1] != this->cameraid) {
			std::cout << "Camera id Error: is the Camera ID really " << this->cameraid << "?" << std::endl;
			return false;
		}
		if (!this->checkSum(data)) {
			std::cout << "Check Sum Error" << std::endl;
			return false;
		}
		return true;
	}

	bool checkSum(unsigned char data[29])
	{
		int s = 0;
		for (int i  = 0; i < 28; i++) {
			s += int(data[i]);
		}
		return data[28] == (unsigned char)(0x40 - (s & 0xff));
	}

	void handleData(unsigned char data[29])
	{
		this->measureFps();
		this->readTransformation(data);
		this->readRotation(data);
		this->readLenzData(data);
	}

	void readRotation(unsigned char data[29]) {
		auto rx = this->hexToInt(data[5], data[6], data[7]) / 32768.0 + this->rotate[0];
		auto ry = this->hexToInt(data[2], data[3], data[4]) / 32768.0 + this->rotate[1];
		auto rz = this->hexToInt(data[8], data[9], data[10]) / 32768.0 + this->rotate[2];

		this->chanValues[3] = rx;
		this->chanValues[4] = ry;
		this->chanValues[5] = rz;
	}

	void readTransformation(unsigned char data[29]) {
		auto x = this->hexToInt(data[11], data[12], data[13]) / 64.0 * 0.001 + this->transform[0];
		auto y = this->hexToInt(data[17], data[18], data[19]) / 64.0 * 0.001 + this->transform[1];
		auto z = this->hexToInt(data[14], data[15], data[16]) / 64.0 * 0.001 + this->transform[2];

		this->chanValues[0] = x;
		this->chanValues[1] = y;
		this->chanValues[2] = z;
	}

	void readLenzData(unsigned char data[29]) {
		auto lz = (double)this->hexToInt(data[20], data[21], data[22]) - (double)0x80000;
		auto lf = (double)this->hexToInt(data[23], data[24], data[25]) - (double)0x80000;

		// zoom
		if (this->zoom_max == 0.0 || this->zoom_max < lz)
			this->zoom_max = lz;
		if (this->zoom_min == 0.0 || this->zoom_min > lz)
			this->zoom_min = lz;

		double zoom = 0.0;
		if (this->zoom_max > 0.0 && this->zoom_min > 0.0 && this->zoom_max > this->zoom_min)
			zoom = (this->zoom_max - lz) / (this->zoom_max - this->zoom_min);

		// focus
		if (this->focus_max == 0.0 || this->focus_max < lf)
			this->focus_max = lf;
		if (this->focus_min == 0.0 || this->focus_min > lf)
			this->focus_min = lf;

		double focus = 0.0;
		if (this->focus_max > 0.0 && this->focus_min > 0.0 && this->focus_max > this->focus_min)
			focus = (this->focus_max - lf) / (this->focus_max - this->focus_min);

		this->chanValues[6] = zoom;
		this->chanValues[7] = focus;
	}

	void measureFps() {
		this->now = time(NULL);
		struct tm* pnow = localtime(&now);
		int sec = pnow->tm_sec;
		if (sec != this->last_sec) {
			this->last_sec = sec;
			this->chanValues[8] = (double)this->fps_counter;

			if (this->fpsHistory.size() > 10) {
				this->fpsHistory.erase(this->fpsHistory.begin());
			}
			this->fpsHistory.push_back((double)this->fps_counter);

			if (this->fpsHistory.size() > 0) {
				double sumHistory = std::accumulate(this->fpsHistory.begin(), this->fpsHistory.end(), 0);
				auto avgHistory = sumHistory / this->fpsHistory.size();
				this->chanValues[9] = avgHistory;
			}

			this->fps_counter = 0;
		}
		this->fps_counter++;
	}

	int hexToInt(unsigned char d1, unsigned char d2, unsigned char d3)
	{
		int v = (
			_lrotl(d1, 24) & 0xff000000 |
			_lrotl(d2, 16) & 0x00ff0000 |
			_lrotl(d3, 8) & 0x0000ff00);
		return v / 0x100;
	}

	void stop()
	{
		this->running = false;
		if (recv_thread.joinable())
			recv_thread.join();
	}

	void close()
	{
		this->serial.Close();
	}

	void getGeneralInfo(CHOP_GeneralInfo* ginfo, const OP_Inputs* inputs, void* reserved1)
	{
		ginfo->cookEveryFrameIfAsked = true;
		ginfo->timeslice = false;
		ginfo->inputMatchIndex = 0;
	}

	bool getOutputInfo(CHOP_OutputInfo* info, const OP_Inputs* inputs, void* reserved1)
	{
		info->numSamples = 1;
		info->numChannels = this->chanNames.size();
		return true;
	}

	void getChannelName(int32_t index, OP_String *name, const OP_Inputs* inputs, void* reserved1)
	{
		name->setString(this->chanNames.at(index).c_str());
	}

	void execute(CHOP_Output* output, const OP_Inputs* inputs, void* reserved)
	{
		inputs->getParDouble3("T", this->transform[0], this->transform[1], this->transform[2]);
		inputs->getParDouble3("R", this->rotate[0], this->rotate[1], this->rotate[2]);

		std::string name = inputs->getParString("Portname");
		std::transform(name.cbegin(), name.cend(), name.begin(), toupper);

		this->cameraid = inputs->getParInt("Cameraid");

		if (this->portname != name) {
			this->stop();
			this->close();
		}
		this->portname = name;

		if (!this->portname.size())
			return;

		if (!this->running) {
			if (!this->open())
				return;
			this->start();
		}

		for (int i = 0; i < this->chanNames.size(); i++) {
			for (int j = 0; j < output->numSamples; j++) {
				output->channels[i][j] = this->chanValues.at(i);
			}
		}
	}

	void setupParameters(OP_ParameterManager* manager, void *reserved1)
	{
		{
			OP_StringParameter sp;
			sp.name = "Portname";
			sp.label = "Port Name";
			OP_ParAppendResult res = manager->appendString(sp);
			assert(res == OP_ParAppendResult::Success);
		}
		{
			OP_NumericParameter np;
			np.name = "Cameraid";
			np.label = "Camera ID";
			np.defaultValues[0] = 1.0;
			np.minSliders[0] = 1.0;
			np.maxSliders[0] = 100.0;
			OP_ParAppendResult res = manager->appendInt(np);
			assert(res == OP_ParAppendResult::Success);
		}
		{
			OP_NumericParameter np;
			np.name = "T";
			np.label = "Transform";
			OP_ParAppendResult res = manager->appendXYZ(np);
			assert(res == OP_ParAppendResult::Success);
		}
		{
			OP_NumericParameter np;
			np.name = "R";
			np.label = "Rotate";
			OP_ParAppendResult res = manager->appendXYZ(np);
			assert(res == OP_ParAppendResult::Success);
		}
		{
			OP_NumericParameter np;
			np.name = "Zoomreset";
			np.label = "Zoom Reset";
			OP_ParAppendResult res = manager->appendPulse(np);
			assert(res == OP_ParAppendResult::Success);
		}
		{
			OP_NumericParameter np;
			np.name = "Focusreset";
			np.label = "Focus Reset";
			OP_ParAppendResult res = manager->appendPulse(np);
			assert(res == OP_ParAppendResult::Success);
		}
	}

	void pulsePressed(const char* name, void* reserved1)
	{
		if (!strcmp(name, "Zoomreset")) {
			this->zoom_max = 0;
			this->zoom_min = 0;
		}
		if (!strcmp(name, "Focusreset")) {
			this->focus_max = 0;
			this->focus_min = 0;
		}
	}

};

extern "C"
{
	DLLEXPORT void FillCHOPPluginInfo(CHOP_PluginInfo* info)
	{
		info->apiVersion = CHOPCPlusPlusAPIVersion;
		info->customOPInfo.opType->setString("Shotokuvr");
		info->customOPInfo.opLabel->setString("Shotoku VR");
		info->customOPInfo.authorName->setString("Akira Kamikura");
		info->customOPInfo.authorEmail->setString("akira.kamikura@gmail.com");
	}

	DLLEXPORT CHOP_CPlusPlusBase* CreateCHOPInstance(const OP_NodeInfo* info)
	{
		return new ShotokuVRCHOP(info);
	}

	DLLEXPORT void DestroyCHOPInstance(CHOP_CPlusPlusBase* instance)
	{
		delete (ShotokuVRCHOP*)instance;
	}

};
