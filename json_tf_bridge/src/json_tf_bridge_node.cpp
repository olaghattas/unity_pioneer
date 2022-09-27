#include <functional>
#include <memory>

#include <nlohmann/json.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/msg/string.hpp"


namespace nlohmann {

///////////////////////////////////////////////////////////////////////////////
// std::variant
///////////////////////////////////////////////////////////////////////////////
// Try to set the value of type T into the variant data if it fails, do nothing
    template<typename T, typename... Ts>
    void variant_from_json(const nlohmann::json &j, std::variant<Ts...> &data) {
        try {
            data = j.get<T>();
        } catch (...) {
        }
    }

    template<typename... Ts>
    struct adl_serializer<std::variant<Ts...>> {
        static void to_json(nlohmann::json &j, const std::variant<Ts...> &data) {
            // Will call j = v automatically for the right type
            std::visit([&j](const auto &v) { j = v; }, data);
        }

        static void from_json(const nlohmann::json &j, std::variant<Ts...> &data) {
            // Call variant_from_json for all types, only one will succeed
            (variant_from_json<Ts>(j, data), ...);
        }
    };

///////////////////////////////////////////////////////////////////////////////
// std::optional
///////////////////////////////////////////////////////////////////////////////
    template<class T>
    void optional_to_json(nlohmann::json &j, const char *name, const std::optional<T> &value) {
        if (value)
            j[name] = *value;
    }

    template<class T>
    void optional_from_json(const nlohmann::json &j, const char *name, std::optional<T> &value) {
        const auto it = j.find(name);
        if (it != j.end())
            value = it->get<T>();
        else
            value = std::nullopt;
    }

///////////////////////////////////////////////////////////////////////////////
// all together
///////////////////////////////////////////////////////////////////////////////
    template<typename>
    constexpr bool is_optional = false;
    template<typename T>
    constexpr bool is_optional<std::optional<T>> = true;

    template<typename T>
    void extended_to_json(const char *key, nlohmann::json &j, const T &value) {
        if constexpr (is_optional<T>)
            nlohmann::optional_to_json(j, key, value);
        else
            j[key] = value;
    }

    template<typename T>
    void extended_from_json(const char *key, const nlohmann::json &j, T &value) {
        if constexpr (is_optional<T>)
            nlohmann::optional_from_json(j, key, value);
        else
            j.at(key).get_to(value);
    }

}

#define EXTEND_JSON_TO(v1) extended_to_json(#v1, nlohmann_json_j, nlohmann_json_t.v1);
#define EXTEND_JSON_FROM(v1) extended_from_json(#v1, nlohmann_json_j, nlohmann_json_t.v1);

#define NLOHMANN_JSONIFY_ALL_THINGS(Type, ...)                                          \
  inline void to_json(nlohmann::json &nlohmann_json_j, const Type &nlohmann_json_t) {   \
      NLOHMANN_JSON_EXPAND(NLOHMANN_JSON_PASTE(EXTEND_JSON_TO, __VA_ARGS__))            \
  }                                                                                     \
  inline void from_json(const nlohmann::json &nlohmann_json_j, Type &nlohmann_json_t) { \
      NLOHMANN_JSON_EXPAND(NLOHMANN_JSON_PASTE(EXTEND_JSON_FROM, __VA_ARGS__))          \
  }



// for convenience
using json = nlohmann::json;

struct TransformData {
    double posX;
    double posY;
    double posZ;
    double quatW;
    double quatX;
    double quatY;
    double quatZ;
    int sec;
    unsigned int nanosec;
    std::string frameID;
};

NLOHMANN_JSONIFY_ALL_THINGS(TransformData, posX, posY, posZ, quatW, quatX, quatY, quatZ, sec, nanosec, frameID)

class FramePublisher : public rclcpp::Node {
public:
    FramePublisher()
            : Node("json_tf_bridge") {

        // Initialize the transform broadcaster
        tf_broadcaster_ =
                std::make_unique<tf2_ros::TransformBroadcaster>(*this);


        subscription_ = this->create_subscription<std_msgs::msg::String>(
                "unity_tf_bridge", 10,
                std::bind(&FramePublisher::callback, this, std::placeholders::_1));

        clock_ = rclcpp::Clock(rcl_clock_type_e::RCL_ROS_TIME);
    }

private:
    void callback(const std::shared_ptr<std_msgs::msg::String> msg) {
        auto json_msg = json::parse(msg->data);
        auto transform_data_vec = json_msg.get<std::vector<TransformData>>();

        for (auto transform_data: transform_data_vec) {

            geometry_msgs::msg::TransformStamped t;

            // Read message content and assign it to
            // corresponding tf variables
//            t.header.stamp = clock_.now();
            t.header.stamp.sec = transform_data.sec;
            t.header.stamp.nanosec = transform_data.nanosec;
            t.header.frame_id = "odom";
            t.child_frame_id = transform_data.frameID;

            // Turtle only exists in 2D, thus we get x and y translation
            // coordinates from the message and set the z coordinate to 0
            t.transform.translation.y = -transform_data.posX;
            t.transform.translation.z = transform_data.posY;
            t.transform.translation.x = transform_data.posZ;

            // For the same reason, turtle can only rotate around one axis
            // and this why we set rotation in x and y to 0 and obtain
            // rotation in z axis from the message
            tf2::Quaternion q;
            t.transform.rotation.w = -transform_data.quatW;
            t.transform.rotation.y = -transform_data.quatX;
            t.transform.rotation.z = transform_data.quatY;
            t.transform.rotation.x = transform_data.quatZ;

            // Send the transformation
            tf_broadcaster_->sendTransform(t);
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Clock clock_;
};

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FramePublisher>());
    rclcpp::shutdown();

    return 0;
}
