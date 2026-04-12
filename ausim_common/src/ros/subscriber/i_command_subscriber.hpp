#pragma once

namespace quadrotor {

class ICommandSubscriber {
 public:
  virtual ~ICommandSubscriber() = default;
};

}  // namespace quadrotor
