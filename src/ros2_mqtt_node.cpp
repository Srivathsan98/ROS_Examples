// ros2_mqtt_node.cpp
#include <chrono>
#include <iostream>
#include <random>
#include <string>
#include <thread>
#include <ctime>
#include <mqtt/async_client.h>

const std::string SERVER_ADDRESS{"tcp://localhost:1883"}; // Replace broker-ip with broker IP or hostname
const std::string CLIENT_ID{"ros2_node"};
const std::string TOPIC_DATA{"sensor/data"};
const std::string TOPIC_REQUEST{"sensor/request"};
const std::string TOPIC_RESPONSE{"sensor/response"};

mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);

void publish_random_data() {
    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(1, 100);

    while (true) {
        int random_value = distribution(generator);
        std::string message = "RandomValue: " + std::to_string(random_value);

        mqtt::message_ptr pubmsg = mqtt::make_message(TOPIC_DATA, message);
        client.publish(pubmsg);

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void on_message_arrived(mqtt::const_message_ptr msg) {
    std::string request = msg->to_string();
    std::string response;

    if (request == "time") {
        std::time_t now = std::time(nullptr);
        response = std::ctime(&now);
        response.pop_back(); // Remove newline
    } else if (request == "storage") {
        // Fake storage value for example
        response = "Storage: 55% used";
    } else if (request == "temperature") {
        // Fake temperature
        response = "Temperature: 23.5 C";
    } else {
        response = "Unknown request";
    }

    mqtt::message_ptr pubmsg = mqtt::make_message(TOPIC_RESPONSE, response);
    client.publish(pubmsg);
}

// Define a callback class inheriting from mqtt::callback
class Callback : public virtual mqtt::callback {
public:
    void connection_lost(const std::string& cause) override {
        std::cout << "Connection lost: " << cause << std::endl;
    }

    void message_arrived(mqtt::const_message_ptr msg) override {
        if (msg->get_topic() == TOPIC_REQUEST) {
            on_message_arrived(msg);
        }
    }

    void delivery_complete(mqtt::delivery_token_ptr token) override {
        // Optional: handle delivery complete event
    }
};


int main() {
    mqtt::connect_options connOpts;
    try {
        client.connect(connOpts)->wait();
        std::cout << "Connected to MQTT broker." << std::endl;

        Callback cb;
        client.set_callback(cb);

        client.subscribe(TOPIC_REQUEST, 1)->wait();

        // Start publishing random data in separate thread
        std::thread pub_thread(publish_random_data);

        pub_thread.join();

        client.disconnect()->wait();
    } catch (const mqtt::exception& exc) {
        std::cerr << "MQTT Error: " << exc.what() << std::endl;
        return 1;
    }
    return 0;
}
