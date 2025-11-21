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
#include <cstdlib> // For rand() and srand()
#include <ctime>   // For time()
#include <vector>
#include <iostream>
#include <thread>
#include "Physics.h"
#include "Util.h"
#include "Kinematics.h"
#include "InverseKinematics.h"
#include "MotionController.h"
#include "LQR.h"
#include "Quadcopter.h"

using namespace std::chrono_literals;

bool replace(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}

std::vector<Vector2D> waypoints1 = {
    Vector2D{0,0},
    Vector2D{1,0},
    Vector2D{1,1},
    Vector2D{0,1},
    Vector2D{-1,1},
    Vector2D{-1,0},
    Vector2D{-1,-1},
    Vector2D{0,-1},
    Vector2D{1,-1},
    Vector2D{1,0},
    Vector2D{0,0}
};
std::vector<Vector2D> waypoints = waypoints1;
std::vector<Vector2D> waypoints2 = {
    Vector2D{0,0},
    Vector2D{1,3},
    Vector2D{-1,1},
    Vector2D{0,0}
};
Path path = Path(waypoints,
    1, //vel
    2, //accel
    5, //jerk
    200
);
Path path2 = Path(waypoints2,
    1, //vel
    2, //accel
    5, //jerk
    200
);

MotorVelocities initialVels = MotorVelocities(0,0,0,0);
State initialState = State(
    Pose3D(Vector3D(0,0,0), Quaternion(0,0,0)),
    Vector3D(0,0,0),
    Vector3D(0,0,0),
    initialVels,
    0
);

Quadcopter quadcopter = Quadcopter(initialState, 1, 1, 0.3);

int runtime = 2500;
int noise = 50;
double ramp = 1000;
int step = 0;

void initSimulation() {
    srand(time(0));
    quadcopter.setHeight(2);
    waypoints = waypoints1;
    step = 0;
}

int numIters;
void runSimulation(double dt) {
    std::cout << "\n\n\nLoop time: " << dt << " Iter: " << numIters << std::endl;

    quadcopter.getState().print();

    if(step == 0 && !quadcopter.busy()) {
        quadcopter.beginPath(path);
        step++;
    }
    if(step == 1 && !quadcopter.busy()) {
        quadcopter.setHeight(3);
        step++;
    }
    if(step == 2 && !quadcopter.busy()) {
        quadcopter.beginPath(path2);
        waypoints = waypoints2; //just to update visualization
        step++;
    }
    if(step == 3 && !quadcopter.busy()) {
        quadcopter.setHeight(0);
        step++;
    }
    if(step == 4 && !quadcopter.busy()) {
        numIters = 10000; //force reset
    }
    if(numIters == 300) {
        // quadcopter.beginManualControl([]() {return Vector3D(1,0,0);});
        // quadcopter.setHeight(3);
    }
    if(numIters == 450) {
        // quadcopter.setHeight(2);
        // quadcopter.beginManualControl([]() {return Vector3D(0,0,0);});
    }

    if(numIters > 150) {
        // req = pathController.getTarget(currentState);
        // req = velocityController.getTarget(currentState, Vector3D(1,0,0));
    }

    if(numIters > 0){
        // currentState = currentState.predict(dt);
        quadcopter.updateSimulation();
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

    for(double t = 0.0; t <= path.getTotalTime(); t += 0.1) {
        PathPoint pt = path.sample(t);
        std::cout << "t=" << t << ": Pos(" << pt.pos.x << ", " << pt.pos.y << "), Vel(" << pt.vel.x << ", " << pt.vel.y << "), Acc(" << pt.acc.x << ", " << pt.acc.y << ")\n";
    }


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

    numIters = 0;
    double last = 0;
    initSimulation();

    while (!done) {
        if(numIters > runtime) {
            numIters = 0;
            initSimulation();
        }

        auto now = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();

        runSimulation(now-last);

        last = now;
        auto qcrotation = quadcopter.getState().pose.rotation;
        auto rotation = Quaternion(
            qcrotation.getYaw() + M_PI_4,
            qcrotation.getPitch(),
            qcrotation.getRoll()
        );


        foxglove::schemas::CubePrimitive cube;
        cube.size = foxglove::schemas::Vector3{QUADCOPTER_ROTOR_DISTANCE, QUADCOPTER_ROTOR_DISTANCE, 0.05};
        cube.color = foxglove::schemas::Color{1, 1, 1, 1};
        auto currentState = quadcopter.getState();
        cube.pose = foxglove::schemas::Pose{foxglove::schemas::Vector3{1*currentState.pose.getX(), 1*currentState.pose.getY(), 1*currentState.pose.getZ()}, foxglove::schemas::Quaternion{rotation.x,rotation.y,rotation.z,rotation.w}};

        foxglove::schemas::SceneEntity entity;
        entity.id = "box";
        entity.cubes.push_back(cube);

        for(int i = 0; i < waypoints.size(); i++) {
            foxglove::schemas::CubePrimitive waypt;
            waypt.size = foxglove::schemas::Vector3{0.05, 0.05, 0.05};
            waypt.color = foxglove::schemas::Color{2, 255, 1, 255};
            waypt.pose = foxglove::schemas::Pose{foxglove::schemas::Vector3{waypoints[i].x, waypoints[i].y, currentState.pose.getZ()}, foxglove::schemas::Quaternion{rotation.x,rotation.y,rotation.z,rotation.w}};
            entity.cubes.push_back(waypt);
        }

        foxglove::schemas::SceneUpdate scene_update;
        scene_update.entities.push_back(entity);

        scene_channel.log(scene_update);
        // if(numIters == 1) std::this_thread::sleep_for(std::chrono::seconds(5));
        numIters++;
        while(std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now().time_since_epoch()).count() - last < LOOP_TIME) {}
    }

    return 0;
}
