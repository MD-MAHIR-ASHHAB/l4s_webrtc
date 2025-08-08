/*
 * Example of how the 10.0.x.x IP filtering works in BasicPortAllocator
 * 
 * This patch automatically filters network interfaces to only use 10.0.x.x addresses.
 * No code changes are required in your application.
 */

#include "p2p/client/basic_port_allocator.h"
#include "rtc_base/ip_address.h"
#include "rtc_base/logging.h"

namespace webrtc {

void ExampleIPFiltering() {
  // The IP filtering happens automatically in BasicPortAllocatorSession::GetNetworks()
  // 
  // When WebRTC enumerates network interfaces, it will:
  // 1. Find all available network interfaces
  // 2. Check each interface's IP address
  // 3. Only keep interfaces with IPs starting with "10.0."
  // 4. Skip all other interfaces (192.168.x.x, public IPs, etc.)
  //
  // Example log output:
  // [INFO] Using interface: eth0 with IP: 10.0.1.100
  // [INFO] Skipping interface: wlan0 with IP: 192.168.1.50
  // [INFO] Applied 10.0.x.x filtering, networks count: 1
  //
  // To customize the IP prefix, modify this line in basic_port_allocator.cc:
  //   if (absl::StartsWith(ip_str, "10.0.")) {
  //
  // For example, to use 192.168.x.x instead:
  //   if (absl::StartsWith(ip_str, "192.168.")) {
  
  RTC_LOG(LS_INFO) << "ICE candidate generation will automatically use only 10.0.x.x addresses";
}

} // namespace webrtc
