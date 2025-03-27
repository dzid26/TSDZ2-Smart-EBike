#!/bin/bash

set -e

INCLUDES="-I src/STM8S_StdPeriph_Lib/inc"
UNINCLUDES="-i STM8S_StdPeriph_Lib"
DEFINES="-DSTM8S105 -DHSE_Value=16000000 -D_SDCC_ -DUSE_STDPERIPH_DRIVER -USTM8AF52Ax -USTM8AF622x -USTM8AF626x"
UNDEFINES="-USTM8AF62Ax -USTM8S003 -USTM8S005 -USTM8S007 -USTM8S103 -USTM8S207 -USTM8S208 -USTM8S903 -U__CSMC__ \
    -U__ICCSTM8__ -U__RCST7__ -U__CDT_PARSER__  -URAM_EXECUTION -UHALL_DEBUG -UTIME_DEBUG  -UUSE_FULL_ASSERT"


cppcheck src --cppcheck-build-dir=bin -j8 --enable=all --check-level=exhaustive --inline-suppr \
    --std=c99 --suppress=missingIncludeSystem --suppress=variableScope --checkers-report=cppchecklist.tmp \
    $DEFINES $UNDEFINES $INCLUDES $UNINCLUDES --force
