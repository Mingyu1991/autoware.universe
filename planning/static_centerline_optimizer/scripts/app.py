from flask import Flask
from flask import abort
from flask import jsonify
from flask import render_template
from flask import request
from flask import session

app = Flask(__name__)
app.secret_key = "hogehoge"

@app.route("/load_map", methods=["POST"])
def load_map_post():
    data = request.get_json()
    map = data["map"]
    session["map_id"] = 1
    print(session["map_id"])

    # load map(map)
    ros2 service call /load_map static_centerline_optimizer/srv/LoadMap map:\ \'\'\
    error = False

    if error:
        abort(500, "error_message")

    return {"mapId": "1"}


@app.route("/plan_route", methods=["POST"])
def plan_route_post():
    data = request.get_json()

    print(session["map_id"])
    # map_id = data['mapId']
    # start_lanelet_id = data['startLaneletId']
    # end_lanelet_id = data['endLaneletId']
    # plan route(start_lanelet_id, end_lanelet_id)

    error = False

    if error:
        abort(500, "error_message")

    return render_template("index.html")


@app.route("/plan_path", methods=["POST"])
def plan_path_post():
    data = request.get_json()

    map_id = data["mapId"]
    vehicle_model = data["vehicleModel"]
    route = data["route"]

    # plan path(vehicle_model, route)

    return render_template("index.html")


if __name__ == "__main__":
    app.debug = True
    app.secret_key = "anyrandomstring"
    app.run(host="localhost")
