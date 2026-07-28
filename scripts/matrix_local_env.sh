#!/usr/bin/env bash

# Load Matrix host overrides as data. The Python parser accepts only a fixed
# variable allowlist and one shell-quoted value per line; nothing is evaluated.
load_matrix_local_env() {
    local project_root="$1"
    local local_env="$project_root/.matrix/local.env"
    if [[ ! -e "$local_env" && ! -L "$local_env" ]]; then
        return 0
    fi

    local payload
    payload="$(mktemp "${TMPDIR:-/tmp}/matrix-local-env.XXXXXX")"
    if ! /usr/bin/python3 -I \
        "$project_root/scripts/update_matrix_local_env.py" \
        --emit0 "$local_env" > "$payload"; then
        rm -f -- "$payload"
        echo "[ERROR] Refusing unsafe Matrix local env: $local_env" >&2
        return 1
    fi

    local -a fields=()
    mapfile -d '' -t fields < "$payload"
    rm -f -- "$payload"
    if (( ${#fields[@]} % 2 != 0 )); then
        echo "[ERROR] Invalid parsed Matrix local env payload" >&2
        return 1
    fi

    local index name value
    for ((index = 0; index < ${#fields[@]}; index += 2)); do
        name="${fields[$index]}"
        value="${fields[$((index + 1))]}"
        if [[ ! "$name" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]]; then
            echo "[ERROR] Invalid parsed Matrix local env name: $name" >&2
            return 1
        fi
        printf -v "$name" '%s' "$value"
        export "$name"
    done
}

# Enforce the tRNA desktop BFM/Isaac topology after every local/profile load.
# This is deliberately scoped to the desktop sentinel: the legacy native
# launcher retains its explicit PICO qualification path for controlled tests.
matrix_bfm_isaac_enforce_desktop_sim_only() {
    local profile="$1"
    if [[ "${MATRIX_BFM_ISAAC_DESKTOP_SIM_ONLY:-0}" != "1" ]]; then
        return 0
    fi
    if [[ "$profile" != "trna" ]]; then
        echo "[ERROR] Desktop sim-only mode is restricted to the trna profile" >&2
        return 2
    fi

    local assignment name expected actual
    local -a topology_contract=(
        "MATRIX_PICO_INPUT_ENABLED=0"
        "MATRIX_EXTERNAL_STATE=1"
        "MATRIX_DISABLE_MC=1"
        "MATRIX_SONIC=0"
        "MATRIX_SONIC_CONTROL_SOURCE=external"
        "MATRIX_GAME_INPUT_SOURCE=keyboard"
        "MATRIX_GAME_NO_INPUT_PROVIDER=1"
    )
    for assignment in "${topology_contract[@]}"; do
        name="${assignment%%=*}"
        expected="${assignment#*=}"
        actual="${!name-}"
        if [[ "$actual" != "$expected" ]]; then
            echo "[ERROR] Desktop sim-only topology rejected $name=$actual (expected $expected)" >&2
            return 2
        fi
    done

    # .matrix/local.env intentionally supports PICO artifact paths for legacy
    # qualification, so scrub again after loading it.  DDS/OpenXR settings are
    # also excluded from this renderer/Isaac-only process tree.
    unset MATRIX_PICO_PYTHON MATRIX_PICO_WHEEL
    unset FASTRTPS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI RMW_IMPLEMENTATION
    unset ROS_DOMAIN_ID ROS_LOCALHOST_ONLY
    unset XR_RUNTIME_JSON XR_API_LAYER_PATH
    export MATRIX_PICO_INPUT_ENABLED=0
}
