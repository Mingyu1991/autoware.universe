#!/bin/bash

echo "lanelet2_map.osm" | xargs -I {} curl -s -H "Content-type: application/json" -X POST -d '{"map": "{}"}' localhost:5000/load_map
