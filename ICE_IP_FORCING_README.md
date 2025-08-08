# ICE Candidate IP Filtering Implementation

## Overview
This patch adds the ability to force WebRTC ICE candidates to use only 10.0.x.x IP addresses by filtering out other network interfaces during candidate generation.

## Changes Made

### 1. BasicPortAllocator Implementation (`p2p/client/basic_port_allocator.cc`)
- Added `absl/strings/match.h` include for `StartsWith` function
- Modified `BasicPortAllocatorSession::GetNetworks()` to filter networks
- Only networks with IP addresses starting with "10.0." are kept
- Added comprehensive logging for network filtering process

### 2. Network Filtering Logic
The filtering works by:
1. **Iterate through all networks**: Check each discovered network interface
2. **Check IP prefix**: Use `absl::StartsWith(ip_str, "10.0.")` to match 10.0.x.x addresses
3. **Skip non-matching**: Log and skip interfaces that don't match the prefix
4. **Fallback**: If no 10.0.x.x networks found, use all networks to avoid connection failure

### 3. Filtering Strategy
- **Prefix-based**: Simple string prefix matching on IP address
- **Inclusive**: Keeps all 10.0.x.x networks (10.0.0.x, 10.0.1.x, etc.)
- **Logged**: Each interface decision is logged for debugging
- **Safe**: Falls back to all networks if no matches found

## Usage

The filtering is automatically applied - no code changes needed in your application.

Simply compile with this patch and WebRTC will automatically:
- Use only 10.0.x.x network interfaces for ICE candidates
- Skip all other interfaces (192.168.x.x, public IPs, etc.)
- Log which interfaces are used/skipped

## Code Example

The core filtering logic in `GetNetworks()`:

```cpp
// Filter networks to only allow 10.0.x.x addresses
std::vector<const Network*> filtered_networks;
for (const Network* network : networks) {
  std::string ip_str = network->GetBestIP().ToString();
  if (absl::StartsWith(ip_str, "10.0.")) {
    filtered_networks.push_back(network);
    RTC_LOG(LS_INFO) << "Using interface: " << network->name() << " with IP: " << ip_str;
  } else {
    RTC_LOG(LS_INFO) << "Skipping interface: " << network->name() << " with IP: " << ip_str;
  }
}
```

## Benefits
- **Simple**: Straightforward prefix-based filtering
- **Deterministic**: Always uses the same IP range for ICE candidates
- **Robust**: Falls back gracefully if no 10.0.x.x networks are found
- **Logged**: Easy to debug which interfaces are being used/skipped
- **Automatic**: No application code changes required

## Testing
1. Compile WebRTC with this patch
2. Run your WebRTC application
3. Check logs for "Using interface" and "Skipping interface" messages
4. Verify that only 10.0.x.x IP addresses appear in ICE candidates

## Customization
To use a different IP prefix, modify the string in this line:
```cpp
if (absl::StartsWith(ip_str, "10.0.")) {
```

For example, to use 192.168.x.x:
```cpp
if (absl::StartsWith(ip_str, "192.168.")) {
```

## Files Modified
- `p2p/client/basic_port_allocator.cc` - Added network filtering logic and include
