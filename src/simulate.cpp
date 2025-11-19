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
#include "Quadcopter.h"
#include "Util.h"
#include "Kinematics.h"
#include "InverseKinematics.h"
#include "MotionController.h"
#include "LQR.h"

using namespace std::chrono_literals;

bool replace(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}

TakeoffController takeoffController = TakeoffController(1.5,2,0.3);

PathController pathController = PathController(Vector2D(),Vector2D(),0);

std::vector<Vector2D> waypoints = {
    Vector2D{0,0},
    Vector2D{1,0},
    Vector2D{2,2},
    Vector2D{0,1}
};
Path testPath = Path(waypoints,
    1, //vel
    1, //accel
    1, //jerk
    20
);

MotorVelocities initialVels = MotorVelocities(0,0,0,0);
State initialState = State(
    Pose3D(Vector3D(0,0,0), Quaternion(0,0,0)),
    Vector3D(0,0,0),
    Vector3D(0,0,0),
    initialVels,
    0
);


State currentState = initialState;

int runtime = 1000;
int noise = 100;
double ramp = 2000;

void initSimulation() {
    currentState = initialState;
    takeoffController.setTargetHeight(3,0);
    testPath = Path(waypoints,
        1, //vel
        0.5, //accel
        0.1, //jerk
        200
    );
    srand(time(0));
}


void runSimulation(int numIters, double dt) {
    std::cout << "\n\n\nLoop time: " << dt << " Iter: " << numIters << std::endl;

    currentState.print();

    if(numIters == 150) {
        pathController.beginPath(testPath, 3);
    }

    QCRequest req = takeoffController.getTarget(currentState, currentState.getPose());
    if(numIters > 150) {
        req = pathController.getTarget(currentState);
        std::cout << "controller req position: ";
        req.position.print();
    }

    auto currentStateNoVel = State(
        currentState.getPose(),
        Vector3D(),
        Vector3D(),
        currentState.getMotorVelocities(),
        currentState.getTime()
    );

    auto motorvels = applyMixer(lqrControlStep(
        getStateVector(currentState),
        getStateVector(req.position, req.velocity)
    ));

    
    double left = motorvels.getLeft();
    double front = motorvels.getFront();
    double right = motorvels.getRight();
    double rear = motorvels.getRear();
    if(true) {
        double maxAllowedDeltaV = ramp * dt;
        double leftdv = left - currentState.getMotorVelocities().getLeft();
        double frontdv = front - currentState.getMotorVelocities().getFront();
        double rightdv = right - currentState.getMotorVelocities().getRight();
        double reardv = rear - currentState.getMotorVelocities().getRear();
        double maxAbsDeltaV = std::max(std::max(std::abs(leftdv), std::abs(frontdv)), std::max(std::abs(rightdv), std::abs(reardv)));
        std::cout << maxAbsDeltaV << "\n";
        if(maxAbsDeltaV > maxAllowedDeltaV) {
            double scale = maxAllowedDeltaV / maxAbsDeltaV;
            left = currentState.getMotorVelocities().getLeft() + leftdv * scale;
            front = currentState.getMotorVelocities().getFront() + frontdv * scale;
            right = currentState.getMotorVelocities().getRight() + rightdv * scale;
            rear = currentState.getMotorVelocities().getRear() + reardv * scale;
        }
    }
    motorvels = MotorVelocities(left + rand() % noise -(noise/2), front+ rand() % noise -(noise/2), right+ rand() % noise -(noise/2), rear+ rand() % noise -(noise/2));
    std::cout << "Motor Velocities Left:" << motorvels.getLeft() << " Front:" << motorvels.getFront()
              << " Right:" << motorvels.getRight() << " Rear:" << motorvels.getRear() << std::endl;
    currentState.setMotorVelocities(motorvels);


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

    for(double t = 0.0; t <= testPath.getTotalTime(); t += 0.1) {
        PathPoint pt = testPath.sample(t);
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

    int numIters = 0;
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

        runSimulation(numIters, now-last);

        last = now;
        auto rotation = currentState.getPose().rotation;


        foxglove::schemas::CubePrimitive cube;
        cube.size = foxglove::schemas::Vector3{QUADCOPTER_ROTOR_DISTANCE, QUADCOPTER_ROTOR_DISTANCE, 0.05};
        cube.color = foxglove::schemas::Color{1, 1, 1, 1};
        cube.pose = foxglove::schemas::Pose{foxglove::schemas::Vector3{1*currentState.getPose().getX(), 1*currentState.getPose().getY(), 1*currentState.getPose().getZ()}, foxglove::schemas::Quaternion{rotation.x,rotation.y,rotation.z,rotation.w}};

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
