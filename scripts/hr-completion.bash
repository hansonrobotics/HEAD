#!/usr/bin/env bash
#
# To enable the completions either:
#  - place this file in /etc/bash_completion.d
#  or
#  - copy this file to e.g. ~/.hr/hr-completion.sh and add the line
#    below to your .bashrc after bash completion features are loaded
#    . ~/.hr/hr-completion.sh

_hr() {
  local command
  local hr_commands="build install run"
  local cur=${COMP_WORDS[COMP_CWORD]}
  local words=${COMP_WORDS[@]}
  local cword=$COMP_CWORD

  _init_completion || return

  for (( i=1 ; i < ${cword} ; i++ )) ; do
    if [[ ${words[i]} == -* ]] ; then continue; fi
    if [[ ${hr_commands} == *${words[i]}* ]] ; then
      command=${words[i]}
    fi
  done

  case ${command} in
    install)
      local components=$(hr run _hr_list_install_components 2> /dev/null)
      COMPREPLY=($(compgen -W "${components}" -- ${cur}))
      ;;
    build)
      local components=$(hr run _hr_list_build_components 2> /dev/null)
      COMPREPLY=($(compgen -W "${components}" -- ${cur}))
      ;;
    run)
      COMPREPLY=()
      ;;
    *)
      COMPREPLY=($(compgen -W "${hr_commands}" -- ${cur}))
      ;;
  esac
}

complete -F _hr hr
