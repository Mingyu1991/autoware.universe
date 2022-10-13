#!/bin/bash

curl -H "Content-type: application/json" -X POST -d '{"map": "hoge"}' localhost:5000/load_map
