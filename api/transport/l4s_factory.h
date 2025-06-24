#ifndef API_TRANSPORT_L4S_FACTORY_H_
#define API_TRANSPORT_L4S_FACTORY_H_

#include <memory>

#include "api/transport/network_control.h"
#include "modules/congestion_controller/l4s/l4s_network_controller.h"

namespace webrtc {

struct L4SFactoryConfig {
  // Whether to use ECT(1) marking for packets
  bool use_ect1_marking = true;
  // Whether to fallback to GCC if L4S isn't supported
  bool fallback_to_gcc = true;
};

// Factory that creates an L4S-based network controller.
class L4SFactory : public NetworkControllerFactoryInterface {
 public:
  explicit L4SFactory(L4SFactoryConfig config = L4SFactoryConfig());
  ~L4SFactory() override;
  
  std::unique_ptr<NetworkControllerInterface> Create(
      NetworkControllerConfig config) override;
      
  TimeDelta GetProcessInterval() const override;

 private:
  const L4SFactoryConfig config_;
};

}  // namespace webrtc

#endif  // API_TRANSPORT_L4S_FACTORY_H_