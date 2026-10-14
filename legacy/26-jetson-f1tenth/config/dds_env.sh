#!/usr/bin/env bash
# ============================================================
#  dds_env.sh — DDS(미들웨어) 환경변수 설정. race.sh / rviz.sh 가 source 한다.
#
#  하는 일:
#    1) race_script.yaml 의 network: 섹션을 읽는다
#    2) 주행망 대역에서 이 기기의 IP 를 찾는다
#    3) config/cyclonedds.xml.in 템플릿을 채워 /tmp 에 실제 XML 을 만든다
#    4) RMW_IMPLEMENTATION / CYCLONEDDS_URI 를 export 한다
#
#  ★ 젯슨과 노트북의 RMW 가 다르면 통신이 0 이 된다. 그래서 대역을 못 찾아도
#    RMW 를 되돌리지 않고, 인터페이스 고정만 포기한 채 CycloneDDS 를 유지한다.
#    (오늘처럼 인터넷용 WiFi 에 붙어 있어도 양쪽이 계속 통신되게)
# ============================================================

# 이 파일의 위치 = repo/config
_DDS_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
_DDS_YAML="${RACE_SCRIPT:-$_DDS_DIR/../race_script.yaml}"
_DDS_TPL="$_DDS_DIR/cyclonedds.xml.in"

# 차량별 overlay. apply_vehicle.py 가 vehicle_car{1,2}.yaml 에서 만든다.
# ★ network.peers 는 차마다 다르므로(젯슨 IP) overlay 가 race_script.yaml 을 이긴다.
#   이 파일은 race.sh 가 apply_vehicle.py 를 돌린 뒤에 source 되므로 항상 최신이다.
#   없으면(생성 전이거나 --car 미설정) race_script.yaml 값을 그대로 쓴다.
_DDS_OVL="$_DDS_DIR/.vehicle_overlay.yaml"

# ★ 노트북에서 "어느 차에 붙을지" 고르는 경로.
#   젯슨은 .car 로 정해지지만(→ overlay), 노트북엔 .car 도 overlay 도 없다.
#   rviz.sh 가 인자(./rviz.sh 77)를 풀어 RACE_CAR=car2 로 넘겨준다.
#   이게 있으면 overlay 보다 **먼저** 그 차의 vehicle 파일을 본다.
if [ -n "${RACE_CAR:-}" ] && [ -f "$_DDS_DIR/vehicle_${RACE_CAR}.yaml" ]; then
    _DDS_VEH="$_DDS_DIR/vehicle_${RACE_CAR}.yaml"
else
    _DDS_VEH=""
fi

_dds_yaml_get () {   # $1=키경로(점 구분)  $2=기본값
    # conda base 의 python3 엔 PyYAML 이 없는 경우가 많다 → 시스템 python 우선.
    local _py=python3
    [ -x /usr/bin/python3 ] && _py=/usr/bin/python3
    "$_py" - "$_DDS_YAML" "$1" "$2" "$_DDS_OVL" "$_DDS_VEH" <<'PY' 2>/dev/null || echo "$2"
import sys, yaml
path, key, default = sys.argv[1], sys.argv[2], sys.argv[3]
ovl = sys.argv[4] if len(sys.argv) > 4 else ''
veh = sys.argv[5] if len(sys.argv) > 5 else ''

def dig(src, key):
    d = yaml.safe_load(open(src))
    for k in key.split('.'):
        d = d[k]
    return d

val = None
# ① RACE_CAR 로 고른 차량 파일 (노트북에서 ./rviz.sh 77)
# ② 젯슨이 apply_vehicle.py 로 펼친 overlay
# ③ 둘 다 없으면 race_script.yaml 공용값
for src in (veh, ovl):
    if val is None and src:
        try:
            val = dig(src, key)
        except Exception:
            val = None
if val is None:
    try:
        val = dig(path, key)
    except Exception:
        val = default

if isinstance(val, list):
    print('\n'.join(str(x) for x in val))
else:
    print(val)
PY
}

setup_dds () {
    [ -f "$_DDS_YAML" ] || return 0

    # ★ ROS_DOMAIN_ID — 두 차를 같은 공유기에서 동시에 돌릴 때의 격리 수단.
    #   peers 가 안 겹쳐서 "우연히" 안 섞이는 것과 달리, 도메인이 다르면
    #   멀티캐스트 그룹·포트 대역 자체가 달라 /scan·/map·/tf 가 절대 안 섞이고
    #   노드 이름 충돌도 없다. dds: fastdds 로 되돌려도 이건 유효하다.
    #   ※ 젯슨↔노트북이 **같은 값**이어야 한다. 한쪽만 다르면 통신이 0 이다.
    local dom; dom=$(_dds_yaml_get 'network.domain_id' '')
    if [ -n "$dom" ] && [ "$dom" != "None" ]; then
        export ROS_DOMAIN_ID="$dom"
        echo "[dds] ROS_DOMAIN_ID=$ROS_DOMAIN_ID${RACE_CAR:+  (${RACE_CAR})}"
    fi

    local dds; dds=$(_dds_yaml_get 'network.dds' 'fastdds')
    case "$dds" in
        cyclone|cyclonedds) ;;
        *) return 0 ;;   # 기본 FastDDS — 아무것도 안 건드림
    esac

    # Humble(젯슨) / Jazzy(노트북) 모두 지원. ROS_DISTRO 우선, 없으면 둘 다 탐색.
    local _cyclo_lib=""
    for _d in "${ROS_DISTRO:-}" humble jazzy; do
        [ -n "$_d" ] || continue
        if [ -e "/opt/ros/${_d}/lib/librmw_cyclonedds_cpp.so" ]; then
            _cyclo_lib="/opt/ros/${_d}/lib/librmw_cyclonedds_cpp.so"
            break
        fi
    done
    if [ -z "$_cyclo_lib" ]; then
        local _pkg="ros-${ROS_DISTRO:-jazzy}-rmw-cyclonedds-cpp"
        echo "[dds] ⚠ CycloneDDS 미설치 — FastDDS 로 진행합니다 (젯슨과 RMW 불일치 → 통신 0)."
        echo "      sudo apt install ${_pkg}"
        echo "      (젯슨은 ros-humble-rmw-cyclonedds-cpp)"
        return 0
    fi

    # ros2 CLI 데몬은 처음 뜰 때의 RMW 를 계속 물고 있다. RMW 를 바꾼 뒤에도
    # 옛 데몬이 살아 있으면 `ros2 topic list` 가 예전 미들웨어 기준의 목록을
    # 돌려줘 "바꿨는데 그대로 보인다"는 착각을 만든다 (실제로 디버깅 중 속았음).
    # 데몬은 필요할 때 자동으로 다시 뜨므로 그냥 내려둔다.
    # ★ ROS_DOMAIN_ID 가 바뀐 경우에도 내려야 한다 — 데몬은 도메인도 물고 있다.
    # ★ timeout 필수. `ros2 daemon stop` 은 데몬이 없거나 CLI 가 콜드일 때 수십 초
    #   블록된다 (2026-08-10: rviz.sh 가 여기서 멈춰 Ctrl+C 를 눌러야 진행됐다).
    #   내려가든 말든 다음 단계에 지장이 없으므로 짧게 끊는다.
    if [ "${RMW_IMPLEMENTATION:-}" != "rmw_cyclonedds_cpp" ] \
       || [ "${_DDS_PREV_DOMAIN:-}" != "${ROS_DOMAIN_ID:-}" ]; then
        timeout 8 ros2 daemon stop >/dev/null 2>&1 || true
    fi
    export _DDS_PREV_DOMAIN="${ROS_DOMAIN_ID:-}"
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

    local subnet; subnet=$(_dds_yaml_get 'network.subnet' '192.168.50.')
    local peers;  peers=$(_dds_yaml_get 'network.peers' '')

    # 주행망 대역에서 내 IP 찾기
    local myip; myip=$(ip -4 -br addr 2>/dev/null \
        | grep -oE "${subnet//./\\.}[0-9]+" | head -1)

    if [ -z "$myip" ]; then
        # 주행망이 아님 (예: 인터넷용 WiFi). 인터페이스 고정과 유니캐스트 피어를
        # 포기하고 멀티캐스트 자동 탐색으로 둔다. RMW 는 그대로 Cyclone 유지 —
        # 여기서 FastDDS 로 되돌리면 상대 기기와 RMW 가 어긋나 통신이 끊긴다.
        unset CYCLONEDDS_URI
        echo "[dds] ⚠ CycloneDDS 폴백 (멀티캐스트 자동 탐색)"
        echo "      network.subnet = '${subnet}' 로 시작하는 내 IP 가 없습니다."
        echo "      내 IP: $(ip -4 -br addr 2>/dev/null \
            | awk '$1!="lo"{printf "%s(%s) ", $3, $1}')"
        echo ""
        echo "      ★ 상대 기기도 반드시 같은 폴백 상태여야 통신됩니다."
        echo "        한쪽만 고정되면 통신이 0 이 됩니다 — 고정된 쪽은"
        echo "        AllowMulticast=false 라 상대의 멀티캐스트 인사를 아예 못 듣고,"
        echo "        피어 목록엔 자기 IP 뿐이라 먼저 인사하지도 않습니다."
        echo "        (2026-08-04 실제로 겪음: 젯슨만 새 대역으로 바꾸자 RViz 먹통)"
        echo "        → 노트북에서도 git pull 로 race_script.yaml 을 맞추고,"
        echo "          양쪽에서 이 줄이 똑같이 나오는지 확인할 것."
        echo ""
        echo "      ※ 폴백은 '되는' 상태가 아니라 '덜 되는' 상태입니다."
        echo "        WiFi 에서 /map(67KB)·/tf_static 래치 전달이 자주 실패합니다"
        echo "        — CycloneDDS 를 도입한 이유가 바로 그 문제였습니다."
        return 0
    fi

    [ -f "$_DDS_TPL" ] || { echo "[dds] ⚠ 템플릿 없음: $_DDS_TPL"; return 0; }

    # 피어 목록을 XML 조각으로 (자기 자신은 빼도 무해하므로 그대로 둔다)
    local peer_xml=""
    local p
    while IFS= read -r p; do
        [ -n "$p" ] && peer_xml+="        <Peer address=\"$p\"/>"$'\n'
    done <<< "$peers"

    local out="/tmp/cyclonedds-${myip//./_}.xml"
    # 템플릿 치환 — @PEERS@ 는 여러 줄이라 awk 로 넣는다
    awk -v ip="$myip" -v peers="$peer_xml" '
        { gsub(/@LOCAL_IP@/, ip)
          if ($0 ~ /@PEERS@/) { printf "%s", peers; next }
          print }
    ' "$_DDS_TPL" > "$out" || { echo "[dds] ⚠ XML 생성 실패"; return 0; }

    export CYCLONEDDS_URI="file://$out"
    echo "[dds] CycloneDDS  |  인터페이스 $myip 고정  |  피어 $(echo "$peers" | grep -c .)개"
    echo "      (상대 기기에서도 이 줄이 떠야 통신됩니다. 한쪽만 고정 = 통신 0)"
}

setup_dds
