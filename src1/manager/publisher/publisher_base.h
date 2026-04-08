#pragma once

#include <memory>
#include <string>
#include "aima/sim/manager/aimrt/aimrt_helper.h"

#include "./publisher_cfg.h"

namespace aimrt::module::publish {

class BasePublisher {
 protected:
  aimrt::CoreRef core_;
  PublisherConfig config_;
  std::atomic<bool> ready_{false};

 private:
  std::atomic<int> print_interval_{0};

 public:
  BasePublisher();
  virtual ~BasePublisher() = default;

  void SetCore(aimrt::CoreRef core);
  void SetConfig(const PublisherConfig& config);
  std::string GetName() const;
  std::string GetTopic() const;
  double GetFrequency() const;
  bool IsEnable() const;
  bool IsPrintable();
  void PrintCount();

  virtual bool Init() = 0;
  virtual void Publish() = 0;
};

using BasePublisherPtr = std::shared_ptr<aimrt::module::publish::BasePublisher>;

class PublisherFactory {
 public:
  using CreatePublisherFunction = std::function<BasePublisherPtr()>;

  static PublisherFactory& Instance();
  void Register(const std::string& name, CreatePublisherFunction func);
  BasePublisherPtr Create(const std::string& name);

 private:
  PublisherFactory() = default;
  ~PublisherFactory() = default;
};

#define REGISTER_PUBLISHER_IMPL(publisher_name, publisher_class)                                                                                    \
  static inline bool _g_##publisher_name##_publisher_registered = []() {                                                                            \
    static_assert(std::is_base_of_v<aimrt::module::publish::BasePublisher, publisher_class>, "publisher_class must be derived from BasePublisher"); \
    aimrt::module::publish::PublisherFactory::Instance().Register(#publisher_name,                                                                  \
                                                                  []() -> BasePublisherPtr { return std::make_shared<publisher_class>(); });        \
    return true;                                                                                                                                    \
  }();

#define REGISTER_PUBLISHER_TEMPLATE_IMPL(sub_name, pub_class) \
  template class pub_class;                                   \
  REGISTER_PUBLISHER_IMPL(sub_name, pub_class)

#define REGISTER_PUBLISHER_TEMPLATE_INSTANTIATION(pub_class) template class pub_class;

}  // namespace aimrt::module::publish
