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
