#include <foxglove/channel.hpp>
#include <foxglove/context.hpp>
#include <foxglove/error.hpp>
#include <foxglove/foxglove.hpp>
#include <foxglove/mcap.hpp>
#include <foxglove/schemas.hpp>
#include <foxglove/server.hpp>
#include "Configuration.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <functional>
#include <iostream>
#include <thread>
#include "Quadcopter.h"
#include "Util.h"
#include "Kinematics.h"
#include "InverseKinematics.h"
#include "MotionController.h"

using namespace std::chrono_literals;

bool replace(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}

TakeoffController takeoffController = TakeoffController(1.5,1,1);
MotorVelocities initialVels = MotorVelocities(0,0,0,0);
QCState initialState = QCState(
    Pose3d(Vector3d(0,0,0), Rotation3d(0,0,0)),
    Pose3d(Vector3d(0,0,0), Rotation3d(0,0,0)), initialVels, 0);


QCState currentState = initialState;

void initSimulation() {
    currentState = initialState;
    takeoffController.setTargetHeight(3,0);
}


void runSimulation(int numIters, double dt) {
    std::cout << "Loop time: " << dt << std::endl;
    //test takeoff controller:
    // std::cout << "\n\ncurrent state velocity: ";
    // currentState.getVelocity().print();
    currentState.print();
    Vector3d accel = takeoffController.getTargetAcceleration(currentState, currentState.getPose()) + Vector3d(0.01,0,0);
    std::cout << "controller req accel: ";
    accel.print();
    auto newVels = optimizeMotorVelocities(
        currentState,
        calculateTargetState(
            currentState,
            accel,
            0
        ),
        1
    );
    currentState.setMotorVelocities(newVels.motorVelocities);


    if(numIters > 0){
        currentState = currentState.predict(dt);
    }
}

int main(){
    std::cout << "hi" << std::endl;

    static std::function<void()> sigint_handler;

    std::signal(SIGINT, [](int) {
        if (sigint_handler) {
        sigint_handler();
        }
    });


    //connect to server
    foxglove::WebSocketServerOptions options;
    auto serverResult = foxglove::WebSocketServer::create(std::move(options));
    if (!serverResult.has_value()) {
        std::cerr << foxglove::strerror(serverResult.error()) << '\n';
        return 1;
    }

    auto server = std::move(serverResult.value());

    // Create a schema for a JSON channel for logging {size: number}
    foxglove::Schema schema;
    schema.encoding = "jsonschema";
    std::string schema_data = R"({
            "type": "object",
            "properties": {
            "size": { "type": "number" }
            }
        })";
    schema.data = reinterpret_cast<const std::byte*>(schema_data.data());
    schema.data_len = schema_data.size();
    auto channel_result = foxglove::RawChannel::create("/size", "json", std::move(schema));
    if (!channel_result.has_value()) {
        std::cerr << "Failed to create channel: " << foxglove::strerror(channel_result.error()) << '\n';
        return 1;
    }
    auto size_channel = std::move(channel_result.value());

    // Create a SceneUpdateChannel for logging changes to a 3d scene
    auto scene_channel_result = foxglove::schemas::SceneUpdateChannel::create("/scene");
    if (!scene_channel_result.has_value()) {
        std::cerr << "Failed to create scene channel: "
                << foxglove::strerror(scene_channel_result.error()) << '\n';
        return 1;
    }
    auto scene_channel = std::move(scene_channel_result.value());


    std::atomic_bool done = false;
    sigint_handler = [&] {
        done = true;
    };

    int numIters = 0;
    double last = 0;
    initSimulation();

    while (!done) {
        if(numIters > 500) {
            numIters = 0;
            initSimulation();
        }

        auto now = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();

        runSimulation(numIters, now-last);

        last = now;
        auto rotation = currentState.getPose().rotation;


        foxglove::schemas::CubePrimitive cube;
        cube.size = foxglove::schemas::Vector3{QUADCOPTER_ROTOR_DISTANCE, QUADCOPTER_ROTOR_DISTANCE, 0.05};
        cube.color = foxglove::schemas::Color{1, 1, 1, 1};
        cube.pose = foxglove::schemas::Pose{foxglove::schemas::Vector3{-1*currentState.getPose().getX(), -1*currentState.getPose().getY(), -1*currentState.getPose().getZ()}, foxglove::schemas::Quaternion{rotation.x,rotation.y,rotation.z,rotation.w}};

        foxglove::schemas::SceneEntity entity;
        entity.id = "box";
        entity.cubes.push_back(cube);

        foxglove::schemas::SceneUpdate scene_update;
        scene_update.entities.push_back(entity);

        scene_channel.log(scene_update);

        numIters++;
        std::this_thread::sleep_for(33ms);
    }

    return 0;
}
