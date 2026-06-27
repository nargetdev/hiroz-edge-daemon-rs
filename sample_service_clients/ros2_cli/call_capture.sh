#!/usr/bin/env bash
set -euo pipefail

service_name="${SERVICE_NAME:-/pgwaam/id2_rpi4/Canon_EOS_6D/capture}"
request_id="${REQUEST_ID:-cli-client-shot}"
shutterspeed="${SHUTTERSPEED:-30}"
iso="${ISO:-7}"
aperture="${APERTURE:-9}"
imageformat="${IMAGEFORMAT:-24}"
service_type="pgwaam_msgs/srv/CaptureDslrImage"

request="{shutterspeed: ${shutterspeed}, iso: ${iso}, aperture: ${aperture}, imageformat: ${imageformat}, request_id: '${request_id}'}"

echo "ROS2_CLI_WAIT service=${service_name} type=${service_type}"
found_service=false
for _ in $(seq 1 20); do
    if ros2 service list -t | grep -F "${service_name} [${service_type}]" >/dev/null; then
        found_service=true
        break
    fi
    sleep 1
done

if [[ "${found_service}" != "true" ]]; then
    echo "ROS2_CLI_SERVICE_MISSING service=${service_name} type=${service_type}" >&2
    exit 2
fi

echo "ROS2_CLI_CALL service=${service_name} request=${request}"
ros2 service call "${service_name}" "${service_type}" "${request}"
