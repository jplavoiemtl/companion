#pragma once
#include <WiFiClientSecure.h>

// The existing shared client, not a second TLS allocation. Core 3.3.11 and
// rollback 3.1.3 leave this protected flag set after failed TCP or startTLS.
class MediaSecureClient : public WiFiClientSecure {
 public:
  void clearPlainStart() { _stillinPlainStart = false; }
};

// Main-task scope only. No plaintext socket or sticky flag may escape to the
// still-image borrower. Native connect/startTLS already stop on failure.
class MediaPlainStartGuard {
 public:
  explicit MediaPlainStartGuard(MediaSecureClient& client) : client_(client) {
    client_.setPlainStart();
  }
  void tcpComplete(bool ok) { tcpOpen_ = ok; }
  void tlsStarted() { tlsAttempted_ = true; }
  ~MediaPlainStartGuard() {
    // Covers an early exit between successful TCP and the native handshake.
    // Do not stop twice on native failure paths, or close successful TLS.
    if (tcpOpen_ && !tlsAttempted_) client_.stop();
    client_.clearPlainStart();
  }
  MediaPlainStartGuard(const MediaPlainStartGuard&) = delete;
  MediaPlainStartGuard& operator=(const MediaPlainStartGuard&) = delete;
 private:
  MediaSecureClient& client_;
  bool tcpOpen_ = false;
  bool tlsAttempted_ = false;
};
