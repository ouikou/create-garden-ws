#!/bin/bash

clear
for folder in `ls -d rule_*/`; do
  # echo "###################### ${PWD}/${folder}"
  python $PWD/.generator/ZRBCodeGeneratorSwicher.py "$PWD/$folder" "$(echo ${folder##rule_} | rev | cut -c 2- | rev)"
done

