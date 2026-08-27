#pragma once
// JSON-on-String helpers — every v2 JSON topic goes through these two.
// Schemas: docs/v2_interfaces.md. Malformed input -> nullopt (drop + count at
// the call site), never throw on the message path.
#include <optional>
#include <string>

#include <nlohmann/json.hpp>
#include <std_msgs/msg/string.hpp>

namespace av
{

    using json = nlohmann::json;

    inline std::optional<json> parse_msg(const std_msgs::msg::String &msg)
    {
        json j = json::parse(msg.data, nullptr, /*allow_exceptions=*/false);
        if (j.is_discarded())
            return std::nullopt;
        return j;
    }

    inline std_msgs::msg::String to_msg(const json &j)
    {
        std_msgs::msg::String m;
        m.data = j.dump();
        return m;
    }

    // Tolerant field access (consumers must tolerate missing/extra fields).
    template <typename T>
    inline T get_or(const json &j, const char *key, T def)
    {
        if (!j.is_object() || !j.contains(key))
            return def;
        try
        {
            if (j.at(key).is_null())
                return def;
            return j.at(key).get<T>();
        }
        catch (...)
        {
            return def;
        }
    }

} // namespace av