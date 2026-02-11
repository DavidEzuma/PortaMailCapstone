#!/bin/sh
set -e

BASE_URL=${BASE_URL:-http://127.0.0.1:5050}

curl -s -X POST "$BASE_URL/api/mode" \
  -H "Content-Type: application/json" \
  -d '{"mode":"ARRIVED"}' >/dev/null

curl -s -X POST "$BASE_URL/api/mode" \
  -H "Content-Type: application/json" \
  -d '{"mode":"DOCK_IDLE"}' >/dev/null

curl -s "$BASE_URL/api/state" >/dev/null
curl -s "$BASE_URL/api/events" >/dev/null

echo "smoke_test: ok"
