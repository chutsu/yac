#include "../munit.hpp"
#include "util/net.hpp"

namespace yac {

int test_tcp_server() {
  tcp_server_t server;
  MU_CHECK(server.port == 8080);
  MU_CHECK(server.sockfd == -1);
  MU_CHECK(server.conns.size() == 0);
  MU_CHECK(server.conn_thread == nullptr);
  return 0;
}

int test_tcp_client() {
  tcp_client_t client;
  MU_CHECK(client.server_ip == "127.0.0.1");
  MU_CHECK(client.server_port == 8080);
  MU_CHECK(client.sockfd == -1);
  MU_CHECK(client.loop_cb == nullptr);
  return 0;
}

int test_tcp_server_config() {
  tcp_server_t server;

  MU_CHECK(tcp_server_config(server) == 0);
  MU_CHECK(server.sockfd != -1);
  return 0;
}

int test_tcp_client_config() {
  tcp_client_t client;

  MU_CHECK(tcp_client_config(client) == 0);
  MU_CHECK(client.sockfd != -1);
  return 0;
}

static void *server_conn_thread(void *arg) {
  UNUSED(arg);
  tcp_server_t *server = (tcp_server_t *) arg;
  const std::string msg_data = "hello world\n";

  // while (1) {
  for (int i = 0; i < 10; i++) {
    for (const auto conn : server->conns) {
      if (write(conn, msg_data.c_str(), msg_data.length()) == -1) {
        LOG_INFO("Opps!\n");
      }
    }
    sleep(1);
  }

  return nullptr;
}

static int client_loop_cb(tcp_client_t &client) {
  UNUSED(client);

  // Read byte
  uint8_t data = 0;
  if (read(client.sockfd, &data, 1) == 1) {
    // printf("%c\n", data);
  }

  return 0;
}

static int client_loop_die_cb(tcp_client_t &client) {
  UNUSED(client);
  return -1;
}

static void *server_thread(void *arg) {
  UNUSED(arg);

  tcp_server_t server;
  server.conn_thread = server_conn_thread;
  tcp_server_config(server);
  tcp_server_loop(server);

  return nullptr;
}

static void *client_thread(void *arg) {
  UNUSED(arg);

  tcp_client_t client;
  client.loop_cb = client_loop_cb;
  tcp_client_config(client);
  tcp_client_loop(client);

  return nullptr;
}

static void *client_bad_thread(void *arg) {
  UNUSED(arg);

  tcp_client_t client;
  client.loop_cb = client_loop_die_cb;
  tcp_client_config(client);
  tcp_client_loop(client);

  return nullptr;
}

int test_tcp_server_client_loop() {
  // Start server
  LOG_INFO("Starting server");
  pthread_t server_tid;
  pthread_create(&server_tid, nullptr, server_thread, nullptr);
  sleep(1);

  // Start client
  LOG_INFO("Starting client 1");
  pthread_t client_tid;
  pthread_create(&client_tid, nullptr, client_thread, nullptr);
  sleep(1);

  // Start client
  LOG_INFO("Starting client 2");
  pthread_t client2_tid;
  pthread_create(&client2_tid, nullptr, client_bad_thread, nullptr);
  sleep(1);

  // Block until both are done
  pthread_join(server_tid, nullptr);
  pthread_join(client_tid, nullptr);
  pthread_join(client2_tid, nullptr);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_tcp_server);
  MU_ADD_TEST(test_tcp_client);
  // MU_ADD_TEST(test_tcp_server_config);
  // MU_ADD_TEST(test_tcp_client_config);
  // MU_ADD_TEST(test_tcp_server_client_loop);
}

} // namespace yac

MU_RUN_TESTS(yac::test_suite);
