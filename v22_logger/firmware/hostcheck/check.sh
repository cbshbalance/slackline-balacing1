#!/bin/bash
# usage: bash firmware/hostcheck/check.sh firmware/v22_raw/v22_raw.ino — 툴체인 없이 g++ -fsyntax-only 로 문법·선언 순서만 검사 (OpenCR/Dynamixel2Arduino API 표면 스텁). 진짜 컴파일은 아두이노 IDE.
S=$(dirname "$0"); f="$1"
( echo '#include "Arduino.h"'; cat "$f" ) > "$S/_t.cpp"
g++ -std=gnu++11 -fsyntax-only -Wall -Wno-unused -I"$S" "$S/_t.cpp"
