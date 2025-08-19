#include "api/transport/l4s_factory.h"

namespace webrtc {

L4SFactory::L4SFactory(L4SFactoryConfig config) : config_(config) {}

L4SFactory::~L4SFactory() = default;

std::unique_ptr<NetworkControllerInterface> L4SFactory::Create(
    NetworkControllerConfig config) {
  L4SConfig l4s_config;
  l4s_config.use_ect1_marking = config_.use_ect1_marking;
  l4s_config.fallback_to_gcc = config_.fallback_to_gcc;
  
  return std::make_unique<L4SNetworkController>(config, l4s_config);
}

TimeDelta L4SFactory::GetProcessInterval() const {
  // Use the same interval as GoogCC
  return TimeDelta::Millis(25);
}

}  // namespace webrtc