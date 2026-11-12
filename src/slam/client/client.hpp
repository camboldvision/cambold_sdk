/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "model/scene.hpp"
#include "svio/transport.hpp"
#include <atomic>
#include <deque>
#include <functional>
#include <memory>
#include <thread>

namespace cambold::detail
{
class Client
{
  public:
    Client() = default;
    ~Client()
    {
        stop();
    }
    using TransportFactory = std::function<std::unique_ptr<svio::Transport>()>;
    void connect_transport(TransportFactory factory, const std::string &label);
    void stop();
    void command(const std::string &name, svio::Json args = svio::Json::object());
    void refresh_maps();
    void subscribe(bool images, bool features);
    svio::Json poll();
    Scene &scene()
    {
        return scene_;
    }

  private:
    void live(std::string label, TransportFactory factory);
    void enqueue(std::uint16_t type, svio::Json body);
    Scene scene_;
    std::atomic<bool> stopping_{false};
    std::thread thread_;
    std::mutex commands_mutex_;
    std::deque<std::pair<std::uint16_t, svio::Json>> commands_;
};
} // namespace cambold::detail
