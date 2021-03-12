import socket


if __name__ == '__main__':
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.connect(('localhost', 5038))
        server.send(b"100")

