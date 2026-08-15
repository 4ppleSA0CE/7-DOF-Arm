#pragma once
//
// ws_server.hpp — minimal, self-contained RFC6455 WebSocket server for
// localhost dev use (plain ws://, no TLS).
//
// Designed to be pumped by a single-threaded sim loop:
//   wsx::WsServer srv(8080);
//   for (;;) {
//     srv.poll(0, [](const std::string& msg){ handle_command(msg); });
//     srv.broadcast(state_json);
//   }
//
// Zero third-party dependencies: C++17 stdlib + POSIX/BSD sockets only.
// SHA-1 and base64 are vendored below (handshake use only — NOT crypto-grade).
// Non-blocking, poll()-based, no threads. The owner drives it.
//
// Compiles on macOS (Apple clang / homebrew llvm) and Linux.
//

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cctype>
#include <string>
#include <vector>
#include <array>
#include <functional>
#include <stdexcept>

#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

namespace wsx {

namespace detail {

// ---------------------------------------------------------------------------
// SHA-1 (FIPS 180-1). For the WebSocket handshake only.
// ---------------------------------------------------------------------------
inline uint32_t sha1_rol(uint32_t x, int s) {
    return (x << s) | (x >> (32 - s));
}

inline std::array<uint8_t, 20> sha1(const uint8_t* data, size_t len) {
    uint32_t h0 = 0x67452301u, h1 = 0xEFCDAB89u, h2 = 0x98BADCFEu,
             h3 = 0x10325476u, h4 = 0xC3D2E1F0u;

    // Pre-processing: pad message to a multiple of 64 bytes.
    std::vector<uint8_t> msg(data, data + len);
    const uint64_t ml = static_cast<uint64_t>(len) * 8u;  // length in bits
    msg.push_back(0x80);
    while (msg.size() % 64 != 56) msg.push_back(0x00);
    for (int i = 7; i >= 0; --i)
        msg.push_back(static_cast<uint8_t>((ml >> (i * 8)) & 0xFF));

    for (size_t chunk = 0; chunk < msg.size(); chunk += 64) {
        uint32_t w[80];
        for (int i = 0; i < 16; ++i) {
            w[i] = (static_cast<uint32_t>(msg[chunk + i * 4 + 0]) << 24) |
                   (static_cast<uint32_t>(msg[chunk + i * 4 + 1]) << 16) |
                   (static_cast<uint32_t>(msg[chunk + i * 4 + 2]) << 8)  |
                   (static_cast<uint32_t>(msg[chunk + i * 4 + 3]));
        }
        for (int i = 16; i < 80; ++i)
            w[i] = sha1_rol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);

        uint32_t a = h0, b = h1, c = h2, d = h3, e = h4;
        for (int i = 0; i < 80; ++i) {
            uint32_t f, k;
            if (i < 20)      { f = (b & c) | ((~b) & d);           k = 0x5A827999u; }
            else if (i < 40) { f = b ^ c ^ d;                      k = 0x6ED9EBA1u; }
            else if (i < 60) { f = (b & c) | (b & d) | (c & d);    k = 0x8F1BBCDCu; }
            else             { f = b ^ c ^ d;                      k = 0xCA62C1D6u; }
            uint32_t tmp = sha1_rol(a, 5) + f + e + k + w[i];
            e = d; d = c; c = sha1_rol(b, 30); b = a; a = tmp;
        }
        h0 += a; h1 += b; h2 += c; h3 += d; h4 += e;
    }

    std::array<uint8_t, 20> out{};
    const uint32_t hs[5] = {h0, h1, h2, h3, h4};
    for (int i = 0; i < 5; ++i) {
        out[i * 4 + 0] = static_cast<uint8_t>((hs[i] >> 24) & 0xFF);
        out[i * 4 + 1] = static_cast<uint8_t>((hs[i] >> 16) & 0xFF);
        out[i * 4 + 2] = static_cast<uint8_t>((hs[i] >> 8) & 0xFF);
        out[i * 4 + 3] = static_cast<uint8_t>((hs[i]) & 0xFF);
    }
    return out;
}

// ---------------------------------------------------------------------------
// base64 encode (standard alphabet, with '=' padding).
// ---------------------------------------------------------------------------
inline std::string base64_encode(const uint8_t* data, size_t len) {
    static const char tbl[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve(((len + 2) / 3) * 4);
    size_t i = 0;
    for (; i + 3 <= len; i += 3) {
        uint32_t n = (static_cast<uint32_t>(data[i]) << 16) |
                     (static_cast<uint32_t>(data[i + 1]) << 8) |
                     (static_cast<uint32_t>(data[i + 2]));
        out.push_back(tbl[(n >> 18) & 63]);
        out.push_back(tbl[(n >> 12) & 63]);
        out.push_back(tbl[(n >> 6) & 63]);
        out.push_back(tbl[n & 63]);
    }
    if (len - i == 1) {
        uint32_t n = static_cast<uint32_t>(data[i]) << 16;
        out.push_back(tbl[(n >> 18) & 63]);
        out.push_back(tbl[(n >> 12) & 63]);
        out.push_back('=');
        out.push_back('=');
    } else if (len - i == 2) {
        uint32_t n = (static_cast<uint32_t>(data[i]) << 16) |
                     (static_cast<uint32_t>(data[i + 1]) << 8);
        out.push_back(tbl[(n >> 18) & 63]);
        out.push_back(tbl[(n >> 12) & 63]);
        out.push_back(tbl[(n >> 6) & 63]);
        out.push_back('=');
    }
    return out;
}

// ---------------------------------------------------------------------------
// Small string helpers for HTTP header parsing.
// ---------------------------------------------------------------------------
inline std::string trim(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

inline std::string to_lower(std::string s) {
    for (auto& ch : s) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    return s;
}

// Returns the (trimmed) value of the first header whose name matches
// name_lower (case-insensitive), or "" if not found.
inline std::string extract_header(const std::string& headers,
                                  const std::string& name_lower) {
    size_t start = 0;
    while (start < headers.size()) {
        size_t end = headers.find("\r\n", start);
        if (end == std::string::npos) end = headers.size();
        std::string line = headers.substr(start, end - start);
        start = (end == headers.size()) ? end : end + 2;
        size_t colon = line.find(':');
        if (colon == std::string::npos) continue;
        if (to_lower(trim(line.substr(0, colon))) == name_lower)
            return trim(line.substr(colon + 1));
    }
    return std::string();
}

}  // namespace detail

// ---------------------------------------------------------------------------
// Free function: compute Sec-WebSocket-Accept from Sec-WebSocket-Key.
//   accept = base64( SHA1( key + GUID ) )
// Exposed so the self-test can validate it directly.
// ---------------------------------------------------------------------------
inline std::string handshake_accept(const std::string& key) {
    static const char kGuid[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    std::string s = key + kGuid;
    auto digest = detail::sha1(reinterpret_cast<const uint8_t*>(s.data()), s.size());
    return detail::base64_encode(digest.data(), digest.size());
}

// ---------------------------------------------------------------------------
// WsServer
// ---------------------------------------------------------------------------
class WsServer {
 public:
    using MessageCb = std::function<void(const std::string&)>;

    // Creates + binds + listens. SO_REUSEADDR, non-blocking listen socket.
    // Throws std::runtime_error on failure.
    explicit WsServer(uint16_t port) {
        listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (listen_fd_ < 0)
            throw std::runtime_error("WsServer: socket() failed");

        int yes = 1;
        ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(port);

        if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            ::close(listen_fd_);
            listen_fd_ = -1;
            throw std::runtime_error("WsServer: bind() failed");
        }
        if (::listen(listen_fd_, 16) < 0) {
            ::close(listen_fd_);
            listen_fd_ = -1;
            throw std::runtime_error("WsServer: listen() failed");
        }
        set_nonblocking(listen_fd_);
    }

    ~WsServer() {
        for (auto& c : clients_)
            if (c.fd >= 0) ::close(c.fd);
        clients_.clear();
        if (listen_fd_ >= 0) ::close(listen_fd_);
    }

    // Non-copyable (owns file descriptors).
    WsServer(const WsServer&) = delete;
    WsServer& operator=(const WsServer&) = delete;

    // Pump once: accept pending connections, complete handshakes, read frames.
    // For every complete inbound TEXT message, invoke on_message(payload).
    // timeout_ms is passed to poll(); returns promptly if there is no activity.
    void poll(int timeout_ms, const MessageCb& on_message) {
        std::vector<pollfd> pfds;
        pfds.reserve(clients_.size() + 1);
        pfds.push_back(pollfd{listen_fd_, POLLIN, 0});

        std::vector<int> client_fds;
        client_fds.reserve(clients_.size());
        for (auto& c : clients_) {
            pfds.push_back(pollfd{c.fd, POLLIN, 0});
            client_fds.push_back(c.fd);
        }

        int n = ::poll(pfds.data(), static_cast<nfds_t>(pfds.size()), timeout_ms);
        if (n <= 0) return;  // 0 = timeout, <0 = error (incl. EINTR): try again next tick

        // Accept first so the listen socket never starves.
        if (pfds[0].revents & POLLIN) accept_all();

        // Process readable clients. We captured fds before accept_all() appended
        // any new clients, so indices stay valid; we resolve each fd to its
        // current client struct (which may have been dropped meanwhile).
        std::vector<int> to_close;
        for (size_t i = 0; i < client_fds.size(); ++i) {
            short re = pfds[i + 1].revents;
            if (re == 0) continue;
            int fd = client_fds[i];
            Client* c = find_client(fd);
            if (!c) continue;

            if (re & POLLIN) {
                bool close_it = false;
                read_and_process(*c, on_message, close_it);
                if (close_it) to_close.push_back(fd);
            } else if (re & (POLLERR | POLLHUP | POLLNVAL)) {
                to_close.push_back(fd);
            }
        }
        for (int fd : to_close) close_client(fd);
    }

    // Send one unmasked TEXT frame to every handshake-completed client.
    // Clients whose write fails hard are dropped.
    void broadcast(const std::string& text) {
        std::string frame = encode_text_frame(text);
        for (auto it = clients_.begin(); it != clients_.end();) {
            if (!it->handshake_done) { ++it; continue; }
            if (!send_all_raw(it->fd, frame.data(), frame.size())) {
                ::close(it->fd);
                it = clients_.erase(it);
            } else {
                ++it;
            }
        }
    }

    // Number of clients that have completed the WebSocket handshake.
    size_t client_count() const {
        size_t n = 0;
        for (const auto& c : clients_) if (c.handshake_done) ++n;
        return n;
    }

 private:
    struct Client {
        int fd = -1;
        bool handshake_done = false;
        std::string inbuf;  // raw inbound bytes, consumed as frames complete
    };

    int listen_fd_ = -1;
    std::vector<Client> clients_;

#ifdef MSG_NOSIGNAL
    static constexpr int kSendFlags = MSG_NOSIGNAL;
#else
    static constexpr int kSendFlags = 0;
#endif

    static void set_nonblocking(int fd) {
        int flags = ::fcntl(fd, F_GETFL, 0);
        if (flags >= 0) ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }

    static void set_nosigpipe(int fd) {
#ifdef SO_NOSIGPIPE  // macOS / BSD: suppress SIGPIPE per-socket
        int yes = 1;
        ::setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &yes, sizeof(yes));
#else
        (void)fd;  // Linux uses MSG_NOSIGNAL on each send instead
#endif
    }

    Client* find_client(int fd) {
        for (auto& c : clients_) if (c.fd == fd) return &c;
        return nullptr;
    }

    void close_client(int fd) {
        for (auto it = clients_.begin(); it != clients_.end(); ++it) {
            if (it->fd == fd) {
                ::close(it->fd);
                clients_.erase(it);
                return;
            }
        }
    }

    void accept_all() {
        for (;;) {
            sockaddr_in addr{};
            socklen_t alen = sizeof(addr);
            int fd = ::accept(listen_fd_, reinterpret_cast<sockaddr*>(&addr), &alen);
            if (fd < 0) {
                if (errno == EINTR) continue;
                break;  // EAGAIN/EWOULDBLOCK: drained, or a transient error
            }
            set_nonblocking(fd);
            set_nosigpipe(fd);
            Client c;
            c.fd = fd;
            clients_.push_back(std::move(c));
        }
    }

    // Drain everything readable on this client, then parse buffered frames.
    void read_and_process(Client& c, const MessageCb& on_message, bool& close_it) {
        char buf[4096];
        for (;;) {
            ssize_t n = ::recv(c.fd, buf, sizeof(buf), 0);
            if (n > 0) {
                c.inbuf.append(buf, static_cast<size_t>(n));
                continue;
            }
            if (n == 0) { close_it = true; break; }  // peer closed
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            if (errno == EINTR) continue;
            close_it = true;  // ECONNRESET et al.
            break;
        }
        // Never let a malformed client throw out of poll().
        try {
            process_buffer(c, on_message, close_it);
        } catch (...) {
            close_it = true;
        }
    }

    // Complete the handshake if pending, then dispatch any whole frames.
    void process_buffer(Client& c, const MessageCb& on_message, bool& close_it) {
        if (!c.handshake_done) {
            size_t pos = c.inbuf.find("\r\n\r\n");
            if (pos == std::string::npos) {
                if (c.inbuf.size() > 16384) close_it = true;  // runaway / non-HTTP
                return;
            }
            std::string headers = c.inbuf.substr(0, pos);
            c.inbuf.erase(0, pos + 4);

            std::string key = detail::extract_header(headers, "sec-websocket-key");
            if (key.empty()) { close_it = true; return; }

            std::string accept = handshake_accept(key);
            std::string resp =
                "HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\n"
                "Connection: Upgrade\r\n"
                "Sec-WebSocket-Accept: " + accept + "\r\n\r\n";
            if (!send_all_raw(c.fd, resp.data(), resp.size())) { close_it = true; return; }
            c.handshake_done = true;
            // Fall through: a client may pipeline a frame right after the upgrade.
        }

        while (c.inbuf.size() >= 2) {
            const uint8_t* p = reinterpret_cast<const uint8_t*>(c.inbuf.data());
            const uint8_t b1 = p[1];
            const uint8_t opcode = static_cast<uint8_t>(p[0] & 0x0F);
            const bool masked = (b1 & 0x80) != 0;
            uint64_t len = static_cast<uint64_t>(b1 & 0x7F);
            size_t need = 2;

            if (len == 126) {
                if (c.inbuf.size() < 4) break;
                len = (static_cast<uint64_t>(p[2]) << 8) | p[3];
                need = 4;
            } else if (len == 127) {
                if (c.inbuf.size() < 10) break;
                len = 0;
                for (int i = 0; i < 8; ++i)
                    len = (len << 8) | static_cast<uint64_t>(p[2 + i]);
                need = 10;
            }

            if (len > (1ull << 26)) { close_it = true; return; }  // 64 MiB sanity cap

            uint8_t mask[4] = {0, 0, 0, 0};
            if (masked) {
                if (c.inbuf.size() < need + 4) break;
                for (int i = 0; i < 4; ++i) mask[i] = p[need + i];
                need += 4;
            }

            if (c.inbuf.size() < need + static_cast<size_t>(len)) break;  // partial

            std::string payload = c.inbuf.substr(need, static_cast<size_t>(len));
            if (masked)
                for (size_t i = 0; i < payload.size(); ++i)
                    payload[i] = static_cast<char>(
                        static_cast<uint8_t>(payload[i]) ^ mask[i & 3]);
            c.inbuf.erase(0, need + static_cast<size_t>(len));

            if (opcode == 0x8) { close_it = true; return; }    // close
            else if (opcode == 0x1) { if (on_message) on_message(payload); }  // text
            // 0x0 continuation, 0x2 binary, 0x9 ping, 0xA pong: ignored gracefully
        }
    }

    // Build an unmasked server->client TEXT frame: 0x81 + length + payload.
    static std::string encode_text_frame(const std::string& payload) {
        std::string out;
        const size_t len = payload.size();
        out.push_back(static_cast<char>(0x81));  // FIN + opcode TEXT
        if (len < 126) {
            out.push_back(static_cast<char>(len));
        } else if (len <= 0xFFFF) {
            out.push_back(static_cast<char>(126));
            out.push_back(static_cast<char>((len >> 8) & 0xFF));
            out.push_back(static_cast<char>(len & 0xFF));
        } else {
            out.push_back(static_cast<char>(127));
            for (int i = 7; i >= 0; --i)
                out.push_back(static_cast<char>((static_cast<uint64_t>(len) >> (i * 8)) & 0xFF));
        }
        out.append(payload);
        return out;
    }

    // Best-effort full write on a non-blocking socket. Returns false on hard error.
    static bool send_all_raw(int fd, const char* data, size_t len) {
        size_t sent = 0;
        while (sent < len) {
            ssize_t n = ::send(fd, data + sent, len - sent, kSendFlags);
            if (n > 0) { sent += static_cast<size_t>(n); continue; }
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                pollfd pfd{fd, POLLOUT, 0};
                if (::poll(&pfd, 1, 1000) <= 0) return false;  // stalled
                continue;
            }
            if (n < 0 && errno == EINTR) continue;
            return false;  // EPIPE / ECONNRESET / etc.
        }
        return true;
    }
};

}  // namespace wsx

// ---------------------------------------------------------------------------
// Self-test:  clang++ -std=c++17 -DWS_SERVER_TEST ...
// ---------------------------------------------------------------------------
#ifdef WS_SERVER_TEST
#include <cassert>
#include <iostream>

int main() {
    // Canonical RFC6455 §1.3 handshake example.
    const std::string key = "dGhlIHNhbXBsZSBub25jZQ==";
    const std::string expected = "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=";
    const std::string got = wsx::handshake_accept(key);

    std::cout << "Sec-WebSocket-Key:    " << key << "\n";
    std::cout << "computed accept:      " << got << "\n";
    std::cout << "expected accept:      " << expected << "\n";
    assert(got == expected && "handshake_accept must match RFC6455 canonical value");

    std::cout << "ws_server self-test OK" << std::endl;
    return 0;
}
#endif
