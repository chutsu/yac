#include "net.hpp"

namespace yac {

/*****************************************************************************
 *                                NETWORKING
 *****************************************************************************/

int ip_port_info(const int sockfd, char *ip, int *port) {
  struct sockaddr_storage addr;
  socklen_t len = sizeof addr;
  if (getpeername(sockfd, (struct sockaddr *)&addr, &len) != 0) {
    return -1;
  }

  // Deal with both IPv4 and IPv6:
  char ipstr[INET6_ADDRSTRLEN];

  if (addr.ss_family == AF_INET) {
    // IPV4
    struct sockaddr_in *s = (struct sockaddr_in *)&addr;
    *port = ntohs(s->sin_port);
    inet_ntop(AF_INET, &s->sin_addr, ipstr, sizeof(ipstr));
  } else {
    // IPV6
    struct sockaddr_in6 *s = (struct sockaddr_in6 *)&addr;
    *port = ntohs(s->sin6_port);
    inet_ntop(AF_INET6, &s->sin6_addr, ipstr, sizeof(ipstr));
  }
  strcpy(ip, ipstr);

  return 0;
}

int ip_port_info(const int sockfd, std::string &ip, int &port) {
  char ipstr[INET6_ADDRSTRLEN];
  const int retval = ip_port_info(sockfd, ipstr, &port);
  ip = std::string{ipstr};
  return retval;
}

tcp_server_t::tcp_server_t(int port_) : port{port_} {}

tcp_client_t::tcp_client_t(const std::string &server_ip_, int server_port_)
    : server_ip{server_ip_}, server_port{server_port_} {}

int tcp_server_config(tcp_server_t &server) {
  // Create socket
  server.sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (server.sockfd == -1) {
    LOG_ERROR("Socket creation failed...");
    return -1;
  }

  // Socket options
  const int en = 1;
  const size_t int_sz = sizeof(int);
  if (setsockopt(server.sockfd, SOL_SOCKET, SO_REUSEADDR, &en, int_sz) < 0) {
    LOG_ERROR("setsockopt(SO_REUSEADDR) failed");
  }
  if (setsockopt(server.sockfd, SOL_SOCKET, SO_REUSEPORT, &en, int_sz) < 0) {
    LOG_ERROR("setsockopt(SO_REUSEPORT) failed");
  }

  // Assign IP, PORT
  struct sockaddr_in sockaddr;
  bzero(&sockaddr, sizeof(sockaddr));
  sockaddr.sin_family = AF_INET;
  sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  sockaddr.sin_port = htons(server.port);

  // Bind newly created socket to given IP
  int retval =
      bind(server.sockfd, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
  if (retval != 0) {
    LOG_ERROR("Socket bind failed: %s", strerror(errno));
    return -1;
  }

  return 0;
}

int tcp_server_loop(tcp_server_t &server) {
  // Server is ready to listen
  if ((listen(server.sockfd, 5)) != 0) {
    LOG_ERROR("Listen failed...");
    return -1;
  }

  // Accept the data packet from client and verification
  std::map<int, pthread_t *> threads;
  int thread_id = 0;

  // DEBUG("Server ready!");
  while (true) {
    // Accept incomming connections
    struct sockaddr_in sockaddr;
    socklen_t len = sizeof(sockaddr);
    int connfd = accept(server.sockfd, (struct sockaddr *)&sockaddr, &len);
    if (connfd < 0) {
      LOG_ERROR("Server acccept failed!");
      return -1;
    } else {
      server.conns.push_back(connfd);
    }

    // Fork off a thread to handle the connection
    pthread_t thread;
    pthread_create(&thread, nullptr, server.conn_thread, (void *)&server);
    threads.insert({thread_id, &thread});
    thread_id++;
  }
  // DEBUG("Server shutting down ...");

  return 0;
}

int tcp_client_config(tcp_client_t &client) {
  // Create socket
  client.sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (client.sockfd == -1) {
    LOG_ERROR("Socket creation failed!");
    return -1;
  }

  // Assign IP, PORT
  struct sockaddr_in server;
  size_t server_size = sizeof(server);
  bzero(&server, server_size);
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = inet_addr(client.server_ip.c_str());
  server.sin_port = htons(client.server_port);

  // Connect to server
  if (connect(client.sockfd, (struct sockaddr *)&server, server_size) != 0) {
    LOG_ERROR("Failed to connect to server!");
    return -1;
  }
  // DEBUG("Connected to the server!");

  return 0;
}

int tcp_client_loop(tcp_client_t &client) {
  while (true) {
    if (client.loop_cb) {
      int retval = client.loop_cb(client);
      switch (retval) {
        case -1:
          return -1;
        case 1:
          break;
      }
    }
  }

  return 0;
}

} // namespace yac
