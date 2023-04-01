#!/bin/bash
g++ tls-alex-client-ncurses.cpp make_tls_client.cpp tls_client_lib.cpp tls_pthread.cpp tls_common_lib.cpp -pthread -lssl -lcrypto -lncurses -o $1
