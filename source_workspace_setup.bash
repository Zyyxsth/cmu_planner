#!/bin/bash

# Safely source the generated colcon workspace setup even when the caller has
# `set -u` enabled. The generated setup scripts are not nounset-safe.

_codex_source_workspace_setup() {
  local script_dir restore_nounset rc d1_setup setup_stderr
  script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
  restore_nounset=0
  case $- in
    *u*)
      restore_nounset=1
      set +u
      ;;
  esac

  : "${COLCON_TRACE:=}"
  : "${COLCON_CURRENT_PREFIX:=}"
  : "${COLCON_PYTHON_EXECUTABLE:=}"
  : "${COLCON_PREFIX_PATH:=}"
  : "${AMENT_CURRENT_PREFIX:=}"
  : "${AMENT_SHELL:=bash}"
  : "${AMENT_PYTHON_EXECUTABLE:=}"
  : "${AMENT_PREFIX_PATH:=}"
  : "${AMENT_TRACE_SETUP_FILES:=}"
  : "${AMENT_RETURN_ENVIRONMENT_HOOKS:=}"
  : "${AMENT_ENVIRONMENT_HOOKS:=}"
  : "${CMAKE_PREFIX_PATH:=}"

  setup_stderr="$(mktemp)"
  . "$script_dir/install/setup.bash" 2>"$setup_stderr"
  rc=$?
  if [ -s "$setup_stderr" ]; then
    grep -Fv \
      "$script_dir/install/arise_slam_mid360/share/arise_slam_mid360/local_setup.bash" \
      "$setup_stderr" >&2 || true
  fi
  rm -f "$setup_stderr"

  d1_setup="${D1_ROS2_SETUP:-/opt/d1_ros2/local_setup.bash}"
  if [ "$rc" -eq 0 ] && [ "${SOURCE_D1_ROS2_SETUP:-1}" != "0" ] && [ -f "$d1_setup" ]; then
    . "$d1_setup"
    rc=$?
  fi

  if [ "$restore_nounset" -eq 1 ]; then
    set -u
  fi

  return "$rc"
}

_codex_source_workspace_setup
unset -f _codex_source_workspace_setup
