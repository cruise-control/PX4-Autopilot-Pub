/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file TattuCan.cpp
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 *
 * Driver for the Tattu 12S 1600mAh Smart Battery connected over CAN.
 *
 * This driver simply decodes the CAN frames based on the specification
 * as provided in the Tattu datasheet DOC 001 REV D, which is highly
 * specific to the 12S 1600mAh battery. Other models of Tattu batteries
 * will NOT work with this driver in its current form.
 *
 */

#include "TattuCan.hpp"
#include "stm32_can.h"
#include <systemlib/mavlink_log.h>
#include <net/if.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <nuttx/can.h>
#include <netpacket/can.h>

// #include <stdio.h>
// #include <stdlib.h>
// #include <stdint.h>
// #include <inttypes.h>
// #include <unistd.h>
// #include <string.h>
// #include <signal.h>
// #include <ctype.h>
// #include <libgen.h>
// #include <time.h>
// #include <errno.h>
// #include "../../../platforms/nuttx/NuttX/nuttx/include/nuttx/can.h"

// #include <sys/time.h>
// #include <sys/types.h>
// #include <sys/socket.h>
// #include <sys/ioctl.h>
// #include <sys/uio.h>
// #include <net/if.h>



// extern orb_advert_t mavlink_log_pub;

orb_advert_t _mavlink_log_pub = nullptr;

TattuCan::TattuCan() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan)
{
}

TattuCan::~TattuCan()
{

	if (_fd >= 0) {
		::close(_fd);
	}


	if (_sk >= 0) {
		::close(_sk);
	}

	_initialized = false;
}

int TattuCan::init_socket()
{
	const char *const can_iface_name = "can1";

	struct ifreq ifr;
	struct sockaddr_can addr;

	PX4_INFO("tattu can bus\n");
	if ((_sk = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		PX4_ERR("Error opening CAN socket\n");
		return -1;
	}

	strncpy(ifr.ifr_name, can_iface_name, IFNAMSIZ - 1);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';
	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);

	if (!ifr.ifr_ifindex) {
		mavlink_log_info(&_mavlink_log_pub, "Failed to find %s device", ifr.ifr_name);
		return -1;
	}

	mavlink_log_info(&_mavlink_log_pub, "CAN Index found %d", ifr.ifr_ifindex);

	memset(&addr, 0, sizeof(struct sockaddr));
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	const int on = 1;
	/* RX Timestamping */

	// if (setsockopt(_sk, SOL_SOCKET, SO_TIMESTAMP, &on, sizeof(on)) < 0) {
	// 	PX4_INFO("SO_TIMESTAMP is disabled %d", get_errno());
	// 	// return -1;
	// }

	if (can_fd) {
		if (setsockopt(_sk, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof(on)) < 0) {
			PX4_ERR("no CAN FD support %d", get_errno());
			return -1;
		}
	}

	if (bind(_sk, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		PX4_ERR("bind error number %d", get_errno());
		return -1;
	}

	// Bring up the interface as it will be down initially (and won't auto-come up)
	ifr.ifr_flags |= IFF_UP;
	ioctl(_sk, SIOCSIFFLAGS, &ifr);

	{
		struct can_filter rfilter[1];
		rfilter[0].can_id   = 0x1091;	// Data Type ID
		rfilter[0].can_mask = 0xFFFF;

		if (setsockopt(_sk, SOL_CAN_RAW, CAN_RAW_FILTER, rfilter, sizeof(rfilter)) < 0) {
			PX4_ERR("CAN Filtering Error %d", get_errno());
		}
	}

	// Set the receive buffer size - doesn't work
	// {
	// 	int rcvbuf_size = CONFIG_NET_RECV_BUFSIZE;
	// 	if (setsockopt(_sk, SOL_SOCKET, SO_RCVBUF,
	// 			&rcvbuf_size, sizeof(rcvbuf_size)) < 0) {
	// 		PX4_ERR("setsockopt SO_RCVBUF %d", get_errno());
	// 		return 1;
	// 	}
	// }


	/* CAN interface ready to be used */

	PX4_INFO("CAN socket open\n");



	// Setup RX msg
	_recv_iov.iov_base = &_recv_frame;

	if (can_fd) {
		_recv_iov.iov_len = sizeof(struct canfd_frame);

	} else {
		_recv_iov.iov_len = sizeof(struct can_frame);
	}

	memset(_recv_control, 0x00, sizeof(_recv_control));

	_recv_msg.msg_iov = &_recv_iov;
	_recv_msg.msg_iovlen = 1;
	_recv_msg.msg_control = &_recv_control;
	_recv_msg.msg_controllen = sizeof(_recv_control);
	_recv_cmsg = CMSG_FIRSTHDR(&_recv_msg);

	return 0;

}
#define USE_SOCK_CAN 1

void TattuCan::Run()
{
	static hrt_abstime timer;
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	if (!_initialized) {
		timer = hrt_absolute_time();

#if USE_SOCK_CAN

		init_socket();
#else
		_fd = ::open("/dev/can0", O_RDWR);

		if (_fd < 0) {
			PX4_INFO("FAILED TO OPEN /dev/can0");
			return;
		}
#endif


		mavlink_log_info(&_mavlink_log_pub, "[TUC] Initialized");

		_initialized = true;
	}

	uint8_t data[64] {};
	CanFrame received_frame{};
	received_frame.payload = &data;

	Tattu12SBatteryMessage tattu_message = {};

	if (_test_mode) {
		static uint16_t count = 0;
		count++;
		battery_status_s battery_status = {};
		battery_status.timestamp = hrt_absolute_time();
		battery_status.connected = true;
		battery_status.cell_count = 12;
		battery_status.id = 111;
		battery_status.cycle_count = count;
		battery_status.voltage_cell_v[0] = -1.0;
		battery_status.voltage_v = -1.1;
		battery_status.current_a = 5.0;
		battery_status.full_charge_capacity_wh = 100.0;
		battery_status.remaining_capacity_wh = battery_status.full_charge_capacity_wh * .95f;
		battery_status.temperature = 30;

		_battery_status_pub.publish(battery_status);
		return;
	}

	while (receive(&received_frame) > 0) {
		if(hrt_elapsed_time(&timer) > 1*1E6)
		{
			// PX4_INFO("Got %d bytes from can, ID of %lX",received_frame.payload_size, received_frame.extended_can_id);
			timer = hrt_absolute_time();
		}
		// Find the start of a transferr
		if ((received_frame.payload_size == 8) && ((uint8_t *)received_frame.payload)[7] == TAIL_BYTE_START_OF_TRANSFER) {
		} else {
			continue;
		}

		// We have the start of a transfer
		size_t offset = 5;
		memcpy(&tattu_message, &(((uint8_t *)received_frame.payload)[2]), offset);
		int payloads = 6;
		while (receive(&received_frame) > 0) {

			size_t payload_size = received_frame.payload_size - 1;
			// TODO: add check to prevent buffer overflow from a corrupt 'payload_size' value
			// TODO: AND look for TAIL_BYTE_START_OF_TRANSFER to indicate end of transfer. Untested...
			memcpy(((char *)&tattu_message) + offset, received_frame.payload, payload_size);
			offset += payload_size;
			payloads--;
			if(payloads <= 0){
				break;
			}
		}

		PX4_INFO("Finished getting message data");

		battery_status_s battery_status = {};
		battery_status.timestamp = hrt_absolute_time();
		battery_status.connected = true;
		battery_status.cell_count = 12;

		// sprintf(battery_status.serial_number, "%d", tattu_message.manufacturer);
		battery_status.id = static_cast<uint8_t>(tattu_message.sku);

		battery_status.cycle_count = tattu_message.cycle_life;
		battery_status.state_of_health = static_cast<uint16_t>(tattu_message.health_status);

		battery_status.voltage_v = static_cast<float>(tattu_message.voltage) / 1000.0f;
		battery_status.voltage_filtered_v = static_cast<float>(tattu_message.voltage) / 1000.0f;
		battery_status.current_a = static_cast<float>(tattu_message.current) / 1000.0f;
		battery_status.current_filtered_a = static_cast<float>(tattu_message.current) / 1000.0f;
		battery_status.remaining = static_cast<float>(tattu_message.remaining_percent) / 100.0f;
		battery_status.temperature = static_cast<float>(tattu_message.temperature);
		battery_status.capacity = tattu_message.standard_capacity;
		battery_status.voltage_cell_v[0] = static_cast<float>(tattu_message.cell_1_voltage) / 1000.0f;
		battery_status.voltage_cell_v[1] = static_cast<float>(tattu_message.cell_2_voltage) / 1000.0f;
		battery_status.voltage_cell_v[2] = static_cast<float>(tattu_message.cell_3_voltage) / 1000.0f;
		battery_status.voltage_cell_v[3] = static_cast<float>(tattu_message.cell_4_voltage) / 1000.0f;
		battery_status.voltage_cell_v[4] = static_cast<float>(tattu_message.cell_5_voltage) / 1000.0f;
		battery_status.voltage_cell_v[5] = static_cast<float>(tattu_message.cell_6_voltage) / 1000.0f;
		battery_status.voltage_cell_v[6] = static_cast<float>(tattu_message.cell_7_voltage) / 1000.0f;
		battery_status.voltage_cell_v[7] = static_cast<float>(tattu_message.cell_8_voltage) / 1000.0f;
		battery_status.voltage_cell_v[8] = static_cast<float>(tattu_message.cell_9_voltage) / 1000.0f;
		battery_status.voltage_cell_v[9] = static_cast<float>(tattu_message.cell_10_voltage / 1000.0f);
		battery_status.voltage_cell_v[10] = static_cast<float>(tattu_message.cell_11_voltage) / 1000.0f;
		battery_status.voltage_cell_v[11] = static_cast<float>(tattu_message.cell_12_voltage) / 1000.0f;

		_battery_status_pub.publish(battery_status);
		mavlink_log_info(&_mavlink_log_pub, "[TUC] Published message");
	}
}


int16_t TattuCan::can_read(CanFrame *received_frame)
{
	if ((_sk < 0) || (received_frame == nullptr)) {
		PX4_INFO("sk < 0");
		return -1;
	}

	#ifdef POLL_READ

	          FD_ZERO(&rdfs);
          FD_SET(s, &rdfs);  /* CAN Socket */
          FD_SET(fd, &rdfs); /* UART */

          if ((ret = select(s + 1, &rdfs, NULL, NULL, NULL)) <= 0)
            {
              continue;
            }

          if (FD_ISSET(s, &rdfs))
            {

	    }

	#else

	#if 0
	uint8_t buf[64+8];

	// struct sockaddr_in _receiver_outaddr;
	static socklen_t addrlen = (socklen_t)0;
	int32_t result = recvfrom(_sk, buf, 72, MSG_DONTWAIT, NULL, &addrlen);
	for( int i=0;i<result;i++){
		printf("%X ",buf[i]);
	}
	printf("\n");
	return -1;
	#elif 1
		// fd_set rdfs;
		// FD_ZERO(&rdfs);
		// FD_SET(_sk, &rdfs);  /* CAN Socket */
		// struct timeval timeout{0};
		// // timeout.tv_sec = 1;
		printf(".");
		fflush(stdout);
		// if ((select(_sk+1, &rdfs, NULL, NULL, &timeout)) <= 0)
		// {
		// 	printf("x");
		// 	return -1;
		// }
		// printf("-");
		// fflush(stdout);
		// if (!FD_ISSET(_sk, &rdfs))
		// {
		// 	printf("+");
		// 	PX4_INFO("Nothing to read");
		// 	return -1;
		// }
		// printf("!");
		// fflush(stdout);
		// Data is now available
		// read with the filter
		int32_t result = recvmsg(_sk, &_recv_msg, MSG_WAIT_SEC);

	#else

	// In the current implementation, you MUST use MSG_WAITALL to get the filter to work,
	// otherwise, the codepath is to get the next message in the queue (or none) and all filtering
	// gets bypassed. This may or may not happen using select.
	// This seems inconsistent and less than ideal for any application, especially as stopping it will
	// not exit cleanly...
	// MSG_WAITALL is giving unexpected results
	int32_t result = recvmsg(_sk, &_recv_msg, /*MSG_WAITALL*/MSG_DONTWAIT);

	#endif

	if (result < 0) {
		printf("x");
		// PX4_INFO("Read result %ld :: %d", result, get_errno());
		return -1;
	}

	/* Copy CAN frame to CanardFrame */
	printf("!");
	if (can_fd) {
		struct canfd_frame *recv_frame = (struct canfd_frame *)&_recv_frame;
		received_frame->extended_can_id = recv_frame->can_id & CAN_EFF_MASK;
		received_frame->payload_size = recv_frame->len;
		memcpy((void *)received_frame->payload,recv_frame->data,recv_frame->len);
		// received_frame->payload = &recv_frame->data;

		PX4_INFO("Read result %ld ID %lX bytes %d", result, recv_frame->can_id & CAN_EFF_MASK, recv_frame->len);
		printf("%X\n",recv_frame->data[0]);
		// for( int i=0;i<recv_frame->len;i++){
		// 	printf("%X ",recv_frame->data[i]);
		// }
		// printf("\n");

	} else {
		struct can_frame *recv_frame = (struct can_frame *)&_recv_frame;
		received_frame->extended_can_id = recv_frame->can_id & CAN_EFF_MASK;
		received_frame->payload_size = recv_frame->can_dlc;
		memcpy((void *)received_frame->payload,recv_frame->data,recv_frame->can_dlc);
		// received_frame->payload = &recv_frame->data; //FIXME either copy or clearly state the pointer reference
	}

	return result;
	#endif
}


int16_t TattuCan::receive(CanFrame *received_frame)
{
	#if USE_SOCK_CAN
	return can_read(received_frame);
	#else
	if ((_fd < 0) || (received_frame == nullptr)) {
		PX4_INFO("fd < 0");
		return -1;
	}

	struct pollfd fds {};

	fds.fd = _fd;

	fds.events = POLLIN;

	// Jake: this doesn't block even in blocking mode, dunno if we need it
	::poll(&fds, 1, 0);

	// Only execute this part if can0 is changed.
	if (fds.revents & POLLIN) {

		// Try to read.
		struct can_msg_s receive_msg;
		const ssize_t nbytes = ::read(fds.fd, &receive_msg, sizeof(receive_msg));

		if (nbytes < 0 || (size_t)nbytes < CAN_MSGLEN(0) || (size_t)nbytes > sizeof(receive_msg)) {
			PX4_INFO("error");
			return -1;

		} else {
			received_frame->extended_can_id = receive_msg.cm_hdr.ch_id;
			received_frame->payload_size = receive_msg.cm_hdr.ch_dlc;
			memcpy((void *)received_frame->payload, receive_msg.cm_data, receive_msg.cm_hdr.ch_dlc);
			return nbytes;
		}
	}

	return 0;
	#endif
}

int TattuCan::start()
{
	// There is a race condition at boot that sometimes causes opening of
	// /dev/can0 to fail. We will delay 0.5s to be safe.
	uint32_t delay_us = 500000;
	ScheduleOnInterval(1000000 / SAMPLE_RATE, delay_us);
	return PX4_OK;
}

void TattuCan::set_test_mode(bool mode)
{
	_test_mode = mode;
}

bool TattuCan::get_test_mode()
{
	return _test_mode;
}

int TattuCan::task_spawn(int argc, char *argv[])
{
	TattuCan *instance = new TattuCan();

	if (!instance) {
		PX4_ERR("driver allocation failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	instance->start();
	return 0;
}

int TattuCan::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for reading data from the Tattu 12S 16000mAh smart battery.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tattu_can", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Generates and publishes dummy data");

	return 0;
}

int TattuCan::print_status()
{
	PX4_INFO("Initialized %s", _initialized?"Yes":"No");
	PX4_INFO("Test Mode %s", _test_mode?"On":"Off");
	return PX4_OK;
}

int TattuCan::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	if(argc <= 0){
		PX4_INFO("no command");
		return PX4_ERROR;
	}

	if (strcmp(argv[0], "test") == 0) {
		bool mode = get_instance()->get_test_mode();
		mode = !mode;
		mavlink_log_info(&_mavlink_log_pub, "Test Mode changed to %s",mode? "On":"Off");
		get_instance()->set_test_mode(mode);
		return PX4_OK;
	}

	return print_usage("Unrecognized command.");
}

extern "C" __EXPORT int tattu_can_main(int argc, char *argv[])
{
	return TattuCan::main(argc, argv);
}
