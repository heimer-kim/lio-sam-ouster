#!/bin/bash

# 📝 출력 파일 초기화
OUTPUT="project_summary.md"
> $OUTPUT

# 🔍 프로젝트 루트 확인
PROJECT_ROOT=$(pwd)
echo "# 📁 Project Summary - $(basename $PROJECT_ROOT)" >> $OUTPUT
echo "" >> $OUTPUT

# 1. 📂 전체 트리 구조 저장 (제한 없음)
echo "## 📂 Directory Structure" >> $OUTPUT
echo '```' >> $OUTPUT
tree -a >> $OUTPUT
echo '```' >> $OUTPUT
echo "" >> $OUTPUT

# 2. 🔍 특정 파일 요약 함수 정의
summarize_file() {
  local filepath="$1"
  local filename=$(basename "$filepath")

  echo "### 📄 $filename (in ${filepath%/*})" >> $OUTPUT
  echo '```' >> $OUTPUT

  if [[ $filename == *".launch" ]]; then
    # launch 파일은 주석 제외한 노드 이름/패키지 요약
    grep -E "<node|<param|<arg" "$filepath" | sed 's/^[ \t]*//' >> $OUTPUT
  elif [[ $filename == "params.yaml" ]]; then
    grep -Ev "^[ \t]*#|^$" "$filepath" | head -n 20 >> $OUTPUT
  elif [[ $filename == "CMakeLists.txt" ]]; then
    grep -E "(find_package|catkin_package|add_executable|target_link_libraries)" "$filepath" >> $OUTPUT
  elif [[ $filename == "package.xml" ]]; then
    grep -E "<(name|version|description|depend)>" "$filepath" >> $OUTPUT
  else
    head -n 20 "$filepath" >> $OUTPUT
  fi

  echo '```' >> $OUTPUT
  echo "" >> $OUTPUT
}

# 3. 📌 요약할 타겟 파일 목록 정의 및 루프
echo "## 📌 주요 설정 파일 요약" >> $OUTPUT
echo "" >> $OUTPUT

find "$PROJECT_ROOT" -type f \( \
    -name "params.yaml" -o \
    -name "*.launch" -o \
    -name "package.xml" -o \
    -name "CMakeLists.txt" \) | while read -r file; do
  summarize_file "$file"
done

echo "✅ Summary generated at $OUTPUT"

