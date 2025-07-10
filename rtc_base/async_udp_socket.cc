/*
 *  Copyright 2004 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "rtc_base/async_udp_socket.h"

#include <cstddef>
#include <memory>
#include <optional>

#include "api/sequence_checker.h"
#include "api/units/time_delta.h"
#include "api/units/timestamp.h"
#include "rtc_base/async_packet_socket.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/network/received_packet.h"
#include "rtc_base/network/sent_packet.h"
#include "rtc_base/socket.h"
#include "rtc_base/socket_address.h"
#include "rtc_base/socket_factory.h"
#include "rtc_base/time_utils.h"

namespace webrtc {

AsyncUDPSocket* AsyncUDPSocket::Create(Socket* socket,
                                       const SocketAddress& bind_address) {
  std::unique_ptr<Socket> owned_socket(socket);
  if (socket->Bind(bind_address) < 0) {
    RTC_LOG(LS_ERROR) << "Bind() failed with error " << socket->GetError();
    return nullptr;
  }
  return new AsyncUDPSocket(owned_socket.release());
}

AsyncUDPSocket* AsyncUDPSocket::Create(SocketFactory* factory,
                                       const SocketAddress& bind_address) {
  Socket* socket = factory->CreateSocket(bind_address.family(), SOCK_DGRAM);
  if (!socket)
    return nullptr;
  return Create(socket, bind_address);
}

AsyncUDPSocket::AsyncUDPSocket(Socket* socket) : socket_(socket) {
  sequence_checker_.Detach();
  // The socket should start out readable but not writable.
  socket_->SignalReadEvent.connect(this, &AsyncUDPSocket::OnReadEvent);
  socket_->SignalWriteEvent.connect(this, &AsyncUDPSocket::OnWriteEvent);
  
  // Log initial ECN socket capabilities
  RTC_LOG(LS_INFO) << "SOCKET INIT: Created AsyncUDPSocket for address=" 
                   << socket_->GetLocalAddress().ToString();
  LogEcnSocketOptions();
  
  // Try to enable ECN reception by default
  int recv_ecn_result = socket_->SetOption(Socket::OPT_RECV_ECN, 1);
  RTC_LOG(LS_INFO) << "SOCKET INIT: Attempted to enable ECN reception, result=" << recv_ecn_result;
  if (recv_ecn_result == 0) {
    LogEcnSocketOptions();
  }
}

SocketAddress AsyncUDPSocket::GetLocalAddress() const {
  return socket_->GetLocalAddress();
}

SocketAddress AsyncUDPSocket::GetRemoteAddress() const {
  return socket_->GetRemoteAddress();
}

int AsyncUDPSocket::Send(const void* pv,
                         size_t cb,
                         const AsyncSocketPacketOptions& options) {
  SentPacketInfo sent_packet(options.packet_id, TimeMillis(),
                             options.info_signaled_after_sent);
  CopySocketInformationToPacketInfo(cb, *this, &sent_packet.info);
  int ret = socket_->Send(pv, cb);
  SignalSentPacket(this, sent_packet);
  return ret;
}

int AsyncUDPSocket::SendTo(const void* pv,
                           size_t cb,
                           const SocketAddress& addr,
                           const AsyncSocketPacketOptions& options) {
  SentPacketInfo sent_packet(options.packet_id, TimeMillis(),
                             options.info_signaled_after_sent);
  CopySocketInformationToPacketInfo(cb, *this, &sent_packet.info);
                            
  // Enhanced ECN logging for socket-level debugging
  // RTC_LOG(LS_INFO) << "SOCKET SEND: Packet size=" << cb << " bytes"
  //                  << " to=" << addr.ToString()
  //                  << " ECN requested=" << (options.ecn_1 ? "ECT(1)" : "Not ECT")
  //                  << " Socket ECN option currently set=" << (has_set_ect1_options_ ? "ECT(1)" : "Not ECT");

  if (has_set_ect1_options_ != options.ecn_1) {
    // It is unclear what is most efficient, setting options on every sent
    // packet or when changed. Potentially, can separate send sockets be used?
    // This is the easier implementation.
    int set_result = socket_->SetOption(Socket::Option::OPT_SEND_ECN,
                                       options.ecn_1 ? 1 : 0);
    if (set_result == 0) {
      has_set_ect1_options_ = options.ecn_1;
      RTC_LOG(LS_INFO) << "SOCKET SEND: Successfully set ECN socket option to " 
                       << (options.ecn_1 ? "ECT(1)" : "Not ECT");
    } else {
      RTC_LOG(LS_ERROR) << "SOCKET SEND: FAILED to set ECN socket option! Error=" << set_result
                        << " Requested=" << (options.ecn_1 ? "ECT(1)" : "Not ECT");
    }
  }
  
  int ret = socket_->SendTo(pv, cb, addr);
  
  // Log the result of the send operation
  if (ret == static_cast<int>(cb)) {
    RTC_LOG(LS_INFO) << "SOCKET SEND: SUCCESS - Sent " << ret << " bytes with ECN=" 
                     << (has_set_ect1_options_ ? "ECT(1)" : "Not ECT");
  } else {
    RTC_LOG(LS_ERROR) << "SOCKET SEND: FAILED - Attempted " << cb << " bytes, sent " << ret 
                      << " bytes, error=" << socket_->GetError();
  }
  SignalSentPacket(this, sent_packet);
  return ret;
}

int AsyncUDPSocket::Close() {
  return socket_->Close();
}

AsyncUDPSocket::State AsyncUDPSocket::GetState() const {
  return STATE_BOUND;
}

int AsyncUDPSocket::GetOption(Socket::Option opt, int* value) {
  return socket_->GetOption(opt, value);
}

int AsyncUDPSocket::SetOption(Socket::Option opt, int value) {
  int result = socket_->SetOption(opt, value);
  
  // Log ECN-related socket option changes
  if (opt == Socket::OPT_SEND_ECN || opt == Socket::OPT_RECV_ECN) {
    // RTC_LOG(LS_INFO) << "SOCKET OPTION: Set " 
    //                  << (opt == Socket::OPT_SEND_ECN ? "SEND_ECN" : "RECV_ECN")
    //                  << " to " << value << " result=" << result;
    LogEcnSocketOptions();
  }
  
  return result;
}

int AsyncUDPSocket::GetError() const {
  return socket_->GetError();
}

void AsyncUDPSocket::SetError(int error) {
  return socket_->SetError(error);
}

// Add helper function to verify ECN socket options
void AsyncUDPSocket::LogEcnSocketOptions() {
  int send_ecn_value = -1;
  int recv_ecn_value = -1;
  
  int send_result = socket_->GetOption(Socket::OPT_SEND_ECN, &send_ecn_value);
  int recv_result = socket_->GetOption(Socket::OPT_RECV_ECN, &recv_ecn_value);
  
  RTC_LOG(LS_INFO) << "SOCKET ECN OPTIONS: "
                   << "SEND_ECN=" << (send_result == 0 ? std::to_string(send_ecn_value) : "ERROR")
                   << " RECV_ECN=" << (recv_result == 0 ? std::to_string(recv_ecn_value) : "ERROR")
                   << " Internal ECT1 flag=" << (has_set_ect1_options_ ? "true" : "false");
}

void AsyncUDPSocket::OnReadEvent(Socket* socket) {
  RTC_DCHECK(socket_.get() == socket);
  RTC_DCHECK_RUN_ON(&sequence_checker_);

  Socket::ReceiveBuffer receive_buffer(buffer_);
  int len = socket_->RecvFrom(receive_buffer);
  if (len < 0) {
    // An error here typically means we got an ICMP error in response to our
    // send datagram, indicating the remote address was unreachable.
    // When doing ICE, this kind of thing will often happen.
    // TODO: Do something better like forwarding the error to the user.
    SocketAddress local_addr = socket_->GetLocalAddress();
    RTC_LOG(LS_INFO) << "AsyncUDPSocket[" << local_addr.ToSensitiveString()
                     << "] receive failed with error " << socket_->GetError();
    return;
  }
  if (len == 0) {
    // Spurios wakeup.
    return;
  }

  if (!receive_buffer.arrival_time) {
    // Timestamp from socket is not available.
    receive_buffer.arrival_time = Timestamp::Micros(TimeMicros());
  } else {
    if (!socket_time_offset_) {
      // Estimate timestamp offset from first packet arrival time.
      socket_time_offset_ =
          Timestamp::Micros(TimeMicros()) - *receive_buffer.arrival_time;
    }
    *receive_buffer.arrival_time += *socket_time_offset_;
  }
  NotifyPacketReceived(
      ReceivedIpPacket(receive_buffer.payload, receive_buffer.source_address,
                       receive_buffer.arrival_time, receive_buffer.ecn));
}

void AsyncUDPSocket::OnWriteEvent(Socket* socket) {
  SignalReadyToSend(this);
}

}  // namespace webrtc
