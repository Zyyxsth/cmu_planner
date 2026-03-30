#!/bin/bash

# Safely source the generated colcon workspace setup even when the caller has
# `set -u` enabled. The generated setup scripts are not nounset-safe.

_codex_source_workspace_setup() {
  local script_dir restore_nounset rc
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

  . "$script_dir/install/setup.bash"
  rc=$?

  if [ "$restore_nounset" -eq 1 ]; then
    set -u
  fi

  return "$rc"
}

_codex_source_workspace_setup
unset -f _codex_source_workspace_setup
