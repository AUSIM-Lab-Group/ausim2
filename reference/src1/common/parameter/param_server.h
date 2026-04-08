#pragma once

#include "./param_manager.h"

#include <iostream>
#include <thread>

#include <boost/asio.hpp>
#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/beast.hpp>
#include <boost/beast/http.hpp>

namespace param {

namespace asio = boost::asio;
namespace http = boost::beast::http;

class ParamServerManager {
  using TcpSocket = asio::ip::tcp::socket;
  using TcpAcceptor = asio::ip::tcp::acceptor;
  using IoContext = asio::io_context;
  using ExecutorType = IoContext::executor_type;
  using AsioExecutor = asio::executor_work_guard<ExecutorType>;
  using Request = http::request<http::string_body>;
  using Response = http::response<http::string_body>;

 private:
  uint16_t port_;
  bool running_ = false;
  IoContext ioContext_;
  TcpAcceptor acceptor_;
  AsioExecutor executor_;

 private:
  ParamServerManager() : acceptor_(ioContext_), executor_(asio::make_work_guard(ioContext_)) {}
  ~ParamServerManager() { Shutdown(); }

  void Init(uint16_t port) {
    port_ = port;
    acceptor_.open(asio::ip::tcp::v4());
    acceptor_.set_option(asio::ip::tcp::acceptor::reuse_address(true));
    acceptor_.bind(asio::ip::tcp::endpoint(asio::ip::tcp::v4(), port_));
    acceptor_.listen(asio::socket_base::max_listen_connections);
    StartAccept();
  }

  void StartAccept() {
    acceptor_.async_accept([this](boost::system::error_code ec, TcpSocket socket) {
      if (!ec) {
        HandleRequest(std::move(socket));
      } else {
        std::cerr << "Error accepting connection: " << ec.message() << std::endl;
      }
      StartAccept();  // 继续接受新的连接
    });
  }

  void HandleRequest(TcpSocket socket) {
    boost::beast::flat_buffer buffer;
    Request req;
    Response rsp;

    boost::system::error_code ec;
    http::read(socket, buffer, req, ec);
    if (ec) {
      std::cerr << "Error reading request: " << ec.message() << std::endl;
      return;
    }

    auto param_map = param::ParamManger::Instance().GetParamMap();

    rsp = Response(http::status::ok, req.version());
    rsp.set(http::field::server, BOOST_BEAST_VERSION_STRING);
    rsp.set(http::field::content_type, "text/html");
    rsp.keep_alive(req.keep_alive());

    // 获取请求的路径
    auto target = req.target();
    if (target != "/parameters") {
      rsp = Response(http::status::bad_request, req.version());
      rsp.set(http::field::server, BOOST_BEAST_VERSION_STRING);
      rsp.set(http::field::content_type, "text/html");
      rsp.keep_alive(req.keep_alive());
      rsp.body() = "bad request";
      http::write(socket, rsp);
      return;
    }

    // 获取请求的方法 GET/POST
    auto method = req.method();
    if (method == http::verb::get) {
      // for 循环将 param_map 的key与序列化后的YAML::Node内容填充到body
      YAML::Node protocol_yaml;
      for (auto& it : param_map) {
        YAML::Node node;
        node["__param_server_file_path__"] = it.first;
        node["__param_server_yaml_content__"] = it.second;
        protocol_yaml.push_back(node);
      }

      rsp.body() = YAML::Dump(protocol_yaml);
    } else if (method == http::verb::post) {
      try {
        std::string body_str = std::string(req.body().begin(), req.body().end());
        YAML::Node body_yaml = YAML::Load(body_str);

        // 遍历body_yaml，将内容更新到param_map
        for (auto item : body_yaml) {
          std::string file_path = item["__param_server_file_path__"].as<std::string>();
          YAML::Node yaml_content = item["__param_server_yaml_content__"];
          param::ParamManger::Instance().UpdateParam(yaml_content, file_path);
          std::cout << "update param: " << file_path << std::endl;
        }
        rsp.body() = "ok";
      } catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
        rsp.body() = "bad request";
      }
    } else {
      rsp = Response(http::status::bad_request, req.version());
      rsp.set(http::field::server, BOOST_BEAST_VERSION_STRING);
      rsp.set(http::field::content_type, "text/html");
      rsp.keep_alive(req.keep_alive());
      rsp.body() = "bad request";
    }

    http::write(socket, rsp);
  }

 public:
  static ParamServerManager& Instance() {
    static ParamServerManager instance;
    return instance;
  }

  void Start(uint16_t port = 21080) {
    if (running_) return;
    port_ = port;
    Init(port);
    std::thread([this]() { ioContext_.run(); }).detach();
    running_ = true;
  }

  void Shutdown() {
    if (!running_) return;
    ioContext_.stop();
    running_ = false;
  }
};

}  // namespace param
