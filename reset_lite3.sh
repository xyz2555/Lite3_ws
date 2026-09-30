#!/bin/bash

source /opt/ros/jazzy/setup.bash
source ~/lite3_ws/install/setup.bash

WORLD_NAME="flat"
ROBOT_NAME="lite3"
CM="/controller_manager"

SPAWN_X="-0.50"
SPAWN_Y="0.0"
SPAWN_Z="0.345"

CONTROLLERS="lite3_effort_controller lite3_position_controller joint_state_broadcaster"

STAND_CMD='{
data: [
  -0.02073, -0.67214, 1.32366,
   0.01497, -0.67765, 1.33907,
  -0.02465, -0.64953, 1.32289,
   0.01714, -0.65116, 1.33529
]
}'

PUB_PID=""
trap '[ -n "$PUB_PID" ] && kill $PUB_PID 2>/dev/null' EXIT

echo "=========================================="
echo "        Lite3 Simulation Reset"
echo "=========================================="

# [1] Matikan experiment node (-f karena nama > 15 karakter)
echo "[1/7] Mematikan experiment node..."
for n in three_leg_stability_test single_step_stair_controller \
         single_leg_lift_test gravity_only_controller; do
    pkill -f "$n" || true
done
sleep 1

# [2] Deactivate + unload satu per satu
echo "[2/7] Deactivate & unload controller..."
for c in $CONTROLLERS; do
    ros2 control switch_controllers --deactivate "$c" >/dev/null 2>&1 || true
done
for c in $CONTROLLERS; do
    ros2 control unload_controller "$c" >/dev/null 2>&1 || true
done

# [3] Hapus robot (perlu type: MODEL) dan cek hasilnya
echo "[3/7] Menghapus ${ROBOT_NAME} dari Gazebo..."
RESULT=$(gz service -s /world/${WORLD_NAME}/remove \
    --reqtype gz.msgs.Entity \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req "name: '${ROBOT_NAME}' type: MODEL" 2>&1)
echo "$RESULT"
if ! echo "$RESULT" | grep -q "true"; then
    echo "GAGAL menghapus robot. Cek nama world/model. Reset dibatalkan."
    exit 1
fi

# Tunggu controller_manager lama benar-benar hilang dari ROS graph
echo "Menunggu controller_manager lama hilang..."
for i in $(seq 1 15); do
    ros2 node list 2>/dev/null | grep -qx "$CM" || break
    sleep 1
done
sleep 1

# [4] Spawn robot baru
echo "[4/7] Spawn ${ROBOT_NAME} baru..."
ros2 run ros_gz_sim create \
    -name ${ROBOT_NAME} \
    -topic robot_description \
    -x ${SPAWN_X} -y ${SPAWN_Y} -z ${SPAWN_Z} || {
    echo "GAGAL spawn robot."; exit 1;
}

# [5] Mulai publish standing pose lebih awal (diterima begitu controller aktif)
echo "[5/7] Mulai publish standing pose..."
ros2 topic pub -r 20 \
    /lite3_position_controller/commands \
    std_msgs/msg/Float64MultiArray \
    "$STAND_CMD" \
    > /tmp/lite3_stand_reset.log 2>&1 &
PUB_PID=$!

# [6] Spawn controller (spawner menunggu controller_manager sampai timeout)
echo "[6/7] Spawn controller..."
ros2 run controller_manager spawner joint_state_broadcaster \
    --controller-manager $CM --controller-manager-timeout 30 || exit 1

ros2 run controller_manager spawner lite3_position_controller \
    --controller-manager $CM --controller-manager-timeout 30 || exit 1

ros2 run controller_manager spawner lite3_effort_controller \
    --controller-manager $CM --controller-manager-timeout 30 --inactive || exit 1

# [7] Beri waktu robot mencapai pose berdiri, lalu hentikan publisher
echo "[7/7] Menstabilkan standing pose..."
sleep 3
kill $PUB_PID 2>/dev/null || true
PUB_PID=""

echo ""
echo "=========================================="
echo "          RESET SELESAI"
echo "=========================================="
echo "Jalankan ulang node eksperimen dengan parameter yang sama"
echo "seperti di launch file (atau pakai --params-file)."
