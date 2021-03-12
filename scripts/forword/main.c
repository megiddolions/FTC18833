#include <winsock2.h>
#include <Windows.h>
#include <stdio.h>
#include <stdbool.h>

typedef struct {
	SOCKET src;
	SOCKET dst;
} FORWORD_ARG;

bool connected(SOCKET sock) {
	fd_set sock_set = { 0 };
	struct timeval timeout = {
		0,
		0
	};
	char buf;
	sock_set.fd_count = 1;
	sock_set.fd_array[0] = sock;
	if (select(1, &sock_set, NULL, NULL, &timeout)) {
		return recv(sock, &buf, sizeof(buf), MSG_PEEK) == 1;
	}
	return true;
}

bool can_read(SOCKET sock) {
	fd_set sock_set = { 0 };
	struct timeval timeout = {
		0,
		0
	};
	sock_set.fd_count = 1;
	sock_set.fd_array[0] = sock;
	int ans = select(1, &sock_set, NULL, NULL, &timeout);
	if (ans != 0) {
		char buf;
		return recv(sock, &buf, 1, MSG_PEEK) == 1;
	}
	return 0;
}


void WINAPI forword(LPVOID param) {
	FORWORD_ARG* arg = (FORWORD_ARG*)param;
	char buffer[1 << 10] = {0};
	while (connected(arg->src) && connected(arg->dst)) {
		if (can_read(arg->src)) {
			int size = recv(arg->src, buffer, sizeof(buffer), 0);
			if (size >= sizeof(buffer))
				break;
			
			buffer[size] = '\0';
			printf("%d -> %d: %d %s\n", arg->src, arg->dst, size, buffer);
			send(arg->dst, buffer, size, 0);
		}
	}
}

int main(int argc, char** argv) {
	WSADATA wsa_data = { 0 };
	if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) 
		return -1;
	
	short src_port, dst_port;
	if (argc != 3)
		return 1;

	src_port = atoi(argv[1]);
	dst_port = atoi(argv[2]);

	SOCKET listen_socket = socket(AF_INET,  SOCK_STREAM, IPPROTO_TCP);
	if (listen_socket == INVALID_SOCKET)
		return 2;

	struct sockaddr_in src_address = { 0 };
	src_address.sin_port = htons(src_port);
	src_address.sin_family = AF_INET;
	src_address.sin_addr.s_addr = INADDR_ANY;

	struct sockaddr_in dst_address = { 0 };
	dst_address.sin_port = htons(dst_port);
	dst_address.sin_family = AF_INET;
	dst_address.sin_addr.s_addr = inet_addr("127.0.0.1");

	if(bind(listen_socket, (struct sockaddr*)&src_address, sizeof(src_address)) == SOCKET_ERROR)
		return 3;

	listen(listen_socket, SOMAXCONN);

	while (TRUE) {
		SOCKET src_socket = accept(listen_socket, NULL, NULL);
		if (src_socket == INVALID_SOCKET)
			return 4;
		SOCKET dst_socket = socket(AF_INET,  SOCK_STREAM, IPPROTO_TCP);

		if(connect(dst_socket, (struct sockaddr*)&dst_address, sizeof(dst_address))) {
			printf("%d", WSAGetLastError());
			return 5;
		}

		FORWORD_ARG args[2] = {
			{
				src_socket,
				dst_socket
			},
			{
				dst_socket,
				src_socket
			}
		};

		HANDLE threads[2] = {
			CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)forword, (void*)args, 0, NULL),
			CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)forword, (void*)(args+1), 0, NULL)
		};
		WaitForMultipleObjects(sizeof(threads)/sizeof(HANDLE), threads, TRUE, INFINITE);

		closesocket(src_socket);
		closesocket(dst_socket);
		puts("close sockets");
	}

	return 0;
}
