from flask import Flask
from flask import abort
from flask import request
from flask import session
import rclpy
from rclpy.node import Node
from static_centerline_optimizer.srv import LoadMap
from static_centerline_optimizer.srv import PlanPath
from static_centerline_optimizer.srv import PlanRoute

rclpy.init()
node = Node("app")

app = Flask(__name__)
app.secret_key = "hogehoge"


def on_load_map(self, request, response):
    response.sum = request.a + request.b
    self.get_logger().info("Incoming request\na: %d b: %d" % (request.a, request.b))

    return response


@app.route("/load_map", methods=["POST"])
def load_map_post():
    data = request.get_json()
    session["map_id"] = 1

    # create client
    cli = node.create_client(LoadMap, "load_map")
    while not cli.wait_for_service(timeout_sec=1.0):
        print("Load map service not available, waiting again...")

    # request map loading
    req = LoadMap.Request()
    req.map = data["map"]
    cli.call_async(req)

    error = False

    if error:
        abort(500, "error_message")

    return {"mapId": "1"}


@app.route("/plan_route", methods=["POST"])
def plan_route_post():
    data = request.get_json()

    # create client
    cli = node.create_client(PlanRoute, "plan_route")
    while not cli.wait_for_service(timeout_sec=1.0):
        print("Plan route service not available, waiting again...")

    # request route planning
    req = PlanRoute.Request()
    req.map_id = 1
    req.start_lane_id = data["start_lane_id"]
    req.end_lane_id = data["end_lane_id"]
    cli.call_async(req)

    error = False

    if error:
        abort(500, "error_message")

    return {"po": "po"}


@app.route("/plan_path", methods=["POST"])
def plan_path_post():
    data = request.get_json()

    # create client
    cli = node.create_client(PlanPath, "plan_path")
    while not cli.wait_for_service(timeout_sec=1.0):
        print("Plan path service not available, waiting again...")

    # request path planning
    req = PlanPath.Request()
    req.start_lane_id = data["start_lane_id"]
    cli.call_async(req)

    return {"po": "po"}


if __name__ == "__main__":
    app.debug = True
    app.secret_key = "anyrandomstring"
    app.run(host="localhost")
