/* SPDX-License-Identifier: Apache-2.0 */
#include "client.hpp"
#include "svio/transport.hpp"
#include <algorithm>
#include <map>
#include <stdexcept>

namespace cambold::detail
{
namespace
{
std::string token()
{
    return svio::id_string(svio::random_id()) + svio::id_string(svio::random_id());
}
svio::Json subscriptions(bool images, bool dense, double budget)
{
    svio::Json streams = svio::Json::array();
    for (auto id : svio::active_streams)
        streams.push_back({{"name", svio::stream_name(id)},
                           {"enabled", id == 1 || id == 3 || (id == 9 ? dense : id == 7 && images)},
                           {"max_hz", id == 1   ? 60
                                      : id == 2 ? 200
                                      : id == 3 ? 1
                                      : id == 9 ? 1
                                                : svio::preview_max_hz},
                           {"max_points", id == 9   ? svio::dense_point_limit
                                          : id == 3 ? svio::keyframe_limit
                                                    : 128}});
    return {{"tx_budget_bytes_per_sec", static_cast<unsigned>(budget)}, {"streams", streams}};
}
} // namespace
void Client::stop()
{
    stopping_ = true;
    if (thread_.joinable())
        thread_.join();
    {
        std::lock_guard<std::mutex> lock(commands_mutex_);
        commands_.clear();
    }
    scene_.connection("DISCONNECTED");
}
void Client::connect_transport(TransportFactory factory, const std::string &label)
{
    stop();
    stopping_ = false;
    scene_.reset();
    thread_ = std::thread(&Client::live, this, label, std::move(factory));
}
void Client::enqueue(std::uint16_t type, svio::Json body)
{
    std::lock_guard<std::mutex> lock(commands_mutex_);
    if (commands_.size() >= 64)
        throw std::runtime_error("Command queue full");
    commands_.emplace_back(type, std::move(body));
}
void Client::command(const std::string &name, svio::Json args)
{
    if (!args.is_object())
        throw std::runtime_error("Command args must be an object");
    enqueue(svio::Command,
            {{"command", name}, {"command_token", token()}, {"args", std::move(args)}});
}
void Client::refresh_maps()
{
    const auto id = token();
    scene_.catalog_requested(id);
    command("MAP_LIST", {{"catalog_token", id}, {"offset", 0}});
}
void Client::subscribe(bool images, bool dense)
{
    enqueue(svio::Subscribe, subscriptions(images, dense, 2097152));
}
svio::Json Client::poll()
{
    return scene_.poll();
}
void Client::live(std::string port, TransportFactory factory)
{
    struct Pending
    {
        svio::Frame frame;
        std::uint64_t deadline;
        unsigned retries = 0;
    };
    unsigned backoff = 0;
    while (!stopping_)
    {
        try
        {
            const std::string &selected = port;
            scene_.connection("OPENING", selected);
            auto transport = factory();
            svio::Decoder decoder;
            svio::Transmitter tx;
            std::deque<svio::Frame> outbound;
            std::map<std::uint32_t, Pending> pending;
            struct OperationWatch
            {
                std::uint64_t next, deadline;
            };
            std::map<std::string, OperationWatch> operations;
            std::uint32_t request = 1;
            std::uint64_t session = 0, last_receive = svio::monotonic_ns(), last_ping = 0,
                          last_sync = 0, last_status = 0, rx = 0, tx_bytes = 0;
            bool calibration_ready = false, maps_supported = false;
            std::string camera_revision, catalog_revision;
            const auto nonce = token();
            scene_.connection("HANDSHAKING", selected);
            auto send = [&](std::uint16_t type, svio::Json j)
            {
                if (request == 0)
                    throw std::runtime_error("Request ID exhausted; reconnect required");
                if (session)
                    j["session_id"] = svio::id_string(session);
                auto f = svio::json_frame(type, j, request++);
                f.timestamp_ns = 0;
                outbound.push_back(f);
                pending.emplace(f.request_id, Pending{f, svio::monotonic_ns() + 1000000000ull, 0});
            };
            send(svio::Hello, {{"client", "cambold_slam_viewer"},
                               {"client_nonce", nonce},
                               {"protocol", {{"major", 1}, {"min_minor", 0}, {"max_minor", 0}}},
                               {"max_payload_bytes", 4096}});
            while (!stopping_)
            {
                auto now = svio::monotonic_ns();
                std::uint8_t bytes[4096];
                auto count = transport->read(bytes, sizeof(bytes), 5);
                rx += count;
                for (const auto &frame : decoder.feed(bytes, count))
                {
                    bool valid = false;
                    svio::Json json;
                    try
                    {
                        if (frame.flags == svio::json_flag)
                        {
                            json = svio::parse_json(frame);
                            if (frame.type == svio::HelloReply)
                            {
                                if (json.at("client_nonce") != nonce)
                                    continue;
                                valid = true;
                            }
                            else
                                valid = session && svio::json_u64(json, "session_id") == session;
                        }
                        else
                        {
                            svio::Reader reader(frame.payload);
                            valid = session && reader.u64() == session;
                        }
                    }
                    catch (const std::exception &e)
                    {
                        scene_.log(e.what());
                        continue;
                    }
                    if (!valid)
                        continue;
                    last_receive = now;
                    if (frame.type == svio::HelloReply)
                    {
                        const auto received = svio::json_u64(json, "session_id");
                        if (!session)
                        {
                            if (!json.contains("capabilities") ||
                                std::find(json["capabilities"].begin(), json["capabilities"].end(),
                                          "incremental_map_v1") == json["capabilities"].end())
                                throw std::runtime_error(
                                    "CamBold SLAM incremental map protocol required; update the "
                                    "board and viewer together");
                            session = received;
                            backoff = 0;
                            pending.clear();
                            scene_.accept(frame);
                            scene_.connection("READY", selected);
                            calibration_ready = json.value("calibration_ready", true);
                            for (const char *section :
                                 {"device", "clock", "frames", "cameras", "streams", "config"})
                                send(svio::Descriptor, {{"section", section}});
                            maps_supported =
                                std::find(json["capabilities"].begin(), json["capabilities"].end(),
                                          "map_management") != json["capabilities"].end();
                            if (maps_supported)
                            {
                                send(svio::Descriptor, {{"section", "map_options"}});
                                catalog_revision =
                                    json.value("map_catalog_revision", std::string());
                                refresh_maps();
                            }
                            send(svio::Status, svio::Json::object());
                            send(svio::Subscribe, subscriptions(false, false, 65536));
                        }
                        else if (session != received)
                            throw std::runtime_error("Unexpected session change");
                    }
                    else
                    {
                        if (frame.type == svio::TimeSyncReply)
                        {
                            svio::Reader reader(frame.payload);
                            reader.u64();
                            auto t1 = reader.u64(), t2 = reader.u64(), t3 = reader.u64();
                            reader.done();
                            auto t4 = svio::monotonic_ns();
                            double rtt = (double(t4) - double(t1)) - (double(t3) - double(t2));
                            if (rtt >= 0 && rtt < 1e9)
                                scene_.round_trip(rtt);
                        }
                        else
                            scene_.accept(frame);
                    }
                    if (frame.type == svio::StatusReply || frame.type == svio::StatusEvent ||
                        frame.type == svio::EpochEvent)
                    {
                        const auto maps_revision =
                            json.value("map_catalog_revision", std::string());
                        if (maps_supported && maps_revision != catalog_revision)
                        {
                            catalog_revision = maps_revision;
                            refresh_maps();
                        }
                        const auto revision = json.value("camera_revision", std::string());
                        if (revision != camera_revision)
                        {
                            camera_revision = revision;
                            for (const char *section : {"cameras", "config"})
                                send(svio::Descriptor, {{"section", section}});
                        }
                        const bool ready = json.value("calibration_ready", true);
                        if (ready && !calibration_ready)
                            for (const char *section : {"cameras", "config"})
                                send(svio::Descriptor, {{"section", section}});
                        calibration_ready = ready;
                    }
                    if (frame.request_id)
                        pending.erase(frame.request_id);
                    if (frame.type == svio::CommandReply &&
                        json.value("status", std::string()) == "accepted")
                    {
                        auto id = json.at("operation_id").get<std::string>();
                        operations.emplace(id, OperationWatch{now, now + 60000000000ull});
                    }
                    if ((frame.type == svio::OperationReply ||
                         frame.type == svio::OperationEvent) &&
                        json.contains("operation_id") &&
                        json.value("status", std::string()) != "accepted")
                        operations.erase(json.at("operation_id").get<std::string>());
                    if ((frame.type == svio::OperationReply ||
                         frame.type == svio::OperationEvent) &&
                        json.contains("map_catalog"))
                    {
                        const auto &page = json["map_catalog"];
                        const auto id = page.value("catalog_token", std::string());
                        // Retries can repeat an event; the scene tracks the expected offset.
                        if (page.contains("next_offset") && !page["next_offset"].is_null() &&
                            scene_.catalog_next(id, page["next_offset"].get<std::size_t>()))
                            command("MAP_LIST",
                                    {{"catalog_token", id}, {"offset", page["next_offset"]}});
                    }
                }
                if (now - last_receive > 4000000000ull)
                    throw std::runtime_error("Device heartbeat/handshake timeout");
                if (session)
                {
                    if (auto feedback = scene_.map_feedback())
                    {
                        (*feedback)["session_id"] = svio::id_string(session);
                        outbound.push_back(svio::json_frame(svio::MapAck, *feedback));
                    }
                    if (now - last_ping >= 1000000000ull)
                    {
                        send(svio::Ping, svio::Json::object());
                        last_ping = now;
                    }
                    if (now - last_status >= 1000000000ull ||
                        (scene_.needs_status() && now - last_status >= 200000000ull))
                    {
                        send(svio::Status, svio::Json::object());
                        last_status = now;
                    }
                    if (now - last_sync >= 2000000000ull)
                    {
                        if (request == 0)
                            throw std::runtime_error("Request ID exhausted; reconnect required");
                        svio::Writer w;
                        w.u64(session);
                        w.u64(now);
                        svio::Frame f;
                        f.type = svio::TimeSync;
                        f.request_id = request++;
                        f.payload = std::move(w.bytes);
                        outbound.push_back(std::move(f));
                        last_sync = now;
                    }
                    for (auto it = operations.begin(); it != operations.end();)
                    {
                        if (now >= it->second.deadline)
                        {
                            scene_.log("Operation completion unknown after 60 seconds: " +
                                       it->first);
                            it = operations.erase(it);
                        }
                        else
                        {
                            if (now >= it->second.next)
                            {
                                send(svio::Operation, {{"operation_id", it->first}});
                                it->second.next = now + 1000000000ull;
                            }
                            ++it;
                        }
                    }
                    std::deque<std::pair<std::uint16_t, svio::Json>> commands;
                    {
                        std::lock_guard<std::mutex> lock(commands_mutex_);
                        commands.swap(commands_);
                    }
                    for (auto &command : commands)
                        send(command.first, std::move(command.second));
                }
                for (auto it = pending.begin(); it != pending.end();)
                {
                    if (now >= it->second.deadline)
                    {
                        if (it->second.retries++ < 2)
                        {
                            outbound.push_back(it->second.frame);
                            it->second.deadline = now + 1000000000ull;
                            ++it;
                        }
                        else
                        {
                            scene_.log(
                                "Request timed out: " + std::to_string(it->first) +
                                ". State-changing commands are not replayed after reconnect.");
                            it = pending.erase(it);
                        }
                    }
                    else
                        ++it;
                }
                if (outbound.size() > 128 || pending.size() > 128)
                    throw std::runtime_error("Request queue limit");
                for (int i = 0; i < 8; ++i)
                {
                    if (tx.empty())
                    {
                        if (outbound.empty())
                            break;
                        tx.begin(std::move(outbound.front()));
                        outbound.pop_front();
                    }
                    auto n = tx.flush(*transport, 5);
                    tx_bytes += n;
                    if (!n || !tx.empty())
                        break;
                }
                scene_.counters(rx, tx_bytes, decoder.crc_errors + decoder.header_errors);
            }
            transport->close();
        }
        catch (const std::exception &e)
        {
            {
                std::lock_guard<std::mutex> lock(commands_mutex_);
                commands_.clear();
            }
            if (!stopping_)
            {
                scene_.connection("RECONNECTING", e.what());
                scene_.log(e.what());
            }
        }
        if (stopping_)
            break;
        auto delay = std::min(5000u, 500u << std::min(backoff++, 4u));
        for (unsigned waited = 0; waited < delay && !stopping_; waited += 20)
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
} // namespace cambold::detail
